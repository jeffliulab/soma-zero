"""gazebo-chess 裁判：世界内部的棋规真值（v0.7）。

角色（对齐 sim-chess「世界=现实、可以懂棋规」的先例）：
- 持一份 python-chess 真值棋盘；大脑发来的每个物理原语（move/remove/place）**动臂之前**先过
  「前置合法闸」——非法直接拒绝（臂不动，省 26 秒/次的无效物理动作），合法才放行；
- 物理核实成功后世界回报 `commit`，裁判推进「标准拆解序列」；一手棋的最后一个原语完成时
  真值棋盘才 push（**真值绝不提前走**）；物理失败回报 `note_failure`，序列指针不动、
  只登记「进行中的这步失败在哪」，供修复原语放宽校验（目标格一致即接受）。
- 终局判定 + 对局落档（logs/games-*.jsonl，格式对齐 sim-chess，追加 world/white/black/physical_fails）。

标准拆解表（与 server.py guidance 的文案约定一致，升级为可校验的规约）：
- 普通走子：move(from,to)
- 吃子：remove(to) → move(from,to)
- 过路兵：remove(被吃兵所在格) → move(from,to)
- 王车易位：move(王两格) → move(车)（王先车后）
- 升变：move(from,to) → remove(to) → place(to, 升变子字母)；吃子升变最前面再加 remove(to)

设计要点：**纯逻辑、零 ROS、零 config 依赖**——所有参数（开局 FEN、落档目录、双方标签）由 world
注入；物理成败一律由 world 核实后回报，裁判自己不看物理。第三方依赖仅 python-chess（世界 venv 已有）。
本模块可完全离线单测（tests/test_gz_referee.py）。

大脑视角红线：裁判只活在世界里。perceive 的 state 仍是空 {}；裁判信息只通过两条道出去——
① 原语的 ok/fail 消息（拒绝原因/对局播报，属于「现实的反馈」）；② 人类调试台 /status（上帝视角）。
"""
from __future__ import annotations

import json
import os
import time

import chess

# 原语的规范形态（world 传入前先归一化格名小写；place 的字母保留大小写=颜色）
Prim = tuple  # ("move", frm, to) | ("remove", sq) | ("place", sq, letter)


def _sq(name: int) -> str:
    return chess.square_name(name)


def expansion(board: chess.Board, m: chess.Move) -> list[Prim]:
    """一手合法棋的标准拆解序列（见模块头的表）。board 必须是走这手**之前**的局面。"""
    if board.is_castling(m):
        kingside = chess.square_file(m.to_square) == 6
        rank = chess.square_rank(m.from_square)
        rf = chess.square(7 if kingside else 0, rank)
        rt = chess.square(5 if kingside else 3, rank)
        return [("move", _sq(m.from_square), _sq(m.to_square)), ("move", _sq(rf), _sq(rt))]
    seq: list[Prim] = []
    if board.is_en_passant(m):
        cap = chess.square(chess.square_file(m.to_square), chess.square_rank(m.from_square))
        seq.append(("remove", _sq(cap)))
    elif board.is_capture(m):
        seq.append(("remove", _sq(m.to_square)))
    seq.append(("move", _sq(m.from_square), _sq(m.to_square)))
    if m.promotion:
        letter = chess.piece_symbol(m.promotion)
        letter = letter.upper() if board.turn == chess.WHITE else letter
        seq.append(("remove", _sq(m.to_square)))
        seq.append(("place", _sq(m.to_square), letter))
    return seq


def board_from_setup(fen: str) -> chess.Board:
    """从 GZCHESS_SETUP_FEN 建真值棋盘：支持完整 FEN 或仅摆放字段（补默认轮次/易位权，
    再按实际王车位置收敛易位权——摆放字段推不出的历史信息取最宽合法值）。"""
    fen = (fen or "").strip()
    if not fen:
        return chess.Board()
    if len(fen.split()) == 1:
        fen = f"{fen} w KQkq - 0 1"
    b = chess.Board(fen)
    b.castling_rights &= b.clean_castling_rights()
    return b


class Referee:
    """前置合法闸 + 物理收敛推进。world 的调用契约（都在世界锁内）：

        ok, why = ref.check(prim)      # 动臂之前；不合法→直接拒绝返回 why，臂不动
        ...物理执行 + 落点核实...
        info = ref.commit(prim)        # 物理成功后；可能推进一手棋（info["advanced"]）
        ref.note_failure(prim, fail, piece_square)   # 物理失败后；登记修复上下文 + 计数
    """

    def __init__(self, setup_fen: str = "", *, white: str = "anima", black: str = "bot",
                 games_dir: str = "", world_name: str = "gazebo-chess", log_games: bool = True) -> None:
        self.board = board_from_setup(setup_fen)
        self.white, self.black = white, black
        self.games_dir, self.world_name, self.log_games = games_dir, world_name, log_games
        self.over = False
        self.result = ""                      # "white"/"black"/"draw"/""（未分胜负）
        self.fails: dict[str, int] = {}       # 物理失败计数（grip_miss/place_offset/drop/...）
        self._pending: list[tuple[chess.Move, list[Prim]]] | None = None   # 进行中的一手（候选着法们）
        self._step = 0                        # 已完成几个原语
        self._attempt: dict | None = None     # 上一次物理失败的原语 {"prim":..., "square":...}
        self._game_seq = 0
        self._logged = False

    # ---------- 查询 ----------
    def _turn_cn(self) -> str:
        return "白方" if self.board.turn == chess.WHITE else "黑方"

    def _next_prims(self) -> list[Prim]:
        """当前期待的下一个原语集合（去重保序）。"""
        if self._pending is None:
            seen, out = set(), []
            for m in self.board.legal_moves:
                p = expansion(self.board, m)[0]
                if p not in seen:
                    seen.add(p)
                    out.append(p)
            return out
        seen, out = set(), []
        for _m, seq in self._pending:
            p = seq[self._step]
            if p not in seen:
                seen.add(p)
                out.append(p)
        return out

    def _pending_desc(self) -> str:
        if self._pending is None:
            return ""
        sans = []
        for m, _seq in self._pending[:3]:
            try:
                sans.append(self.board.san(m))
            except Exception:  # noqa: BLE001  san 对个别候选算不出时退回 uci，不让描述崩掉拒绝流程
                sans.append(m.uci())
        return "/".join(sans)

    @staticmethod
    def _prim_desc(p: Prim) -> str:
        if p[0] == "move":
            return f"move({p[1]}→{p[2]})"
        if p[0] == "remove":
            return f"remove({p[1]})"
        return f"place({p[1]},{p[2]})"

    # ---------- 前置闸 ----------
    def check(self, prim: Prim) -> tuple[bool, str]:
        """这个原语现在能不能做。只读不写（失败/放弃都不留状态）。"""
        if self.over:
            return False, f"对局已结束（{self._result_cn()}）——在世界网页点「开新局」再下。"
        if self._matches_next(prim):
            return True, ""
        if self._attempt is not None and self._repair_match(prim):
            return True, ""
        return False, self._refusal(prim)

    def _matches_next(self, prim: Prim) -> bool:
        return prim in self._next_prims()

    def _repair_match(self, prim: Prim) -> bool:
        """物理失败后的修复放宽：目标格必须与期待一致；**源格必须是世界报的子实际所在格**
        （夹空=还在原格、放偏=报了实际落格），只有世界也不知道子在哪（drop 未定位）才放开源格。
        教训（2026-07-06 对局 2 实锤）：最初「move 源格全自由」被大脑钻了空子——f1 车连夹 8 次
        失败后它把 g1 的**王**搬去目标格，裁判照单全收记成 Rxf2，真值与物理大面积分叉。
        修复的本意是「同一颗子从它实际的位置挪到本来要去的地方」，源格校验就是「同一颗子」的代理。"""
        exp: Prim = self._attempt["prim"]
        if prim[0] != exp[0]:
            return False
        known = self._attempt.get("square")
        if prim[0] == "move":
            if prim[2] != exp[2]:
                return False
            return known is None or prim[1] == known
        if prim[0] == "remove":
            return known is None or prim[1] == known
        return prim == exp

    def _refusal(self, prim: Prim) -> str:
        """拒绝原因，说人话（帮大脑自我纠偏——它只能靠这句话理解哪里不对）。"""
        if self._attempt is not None:
            exp: Prim = self._attempt["prim"]
            known = self._attempt.get("square")
            if prim[0] == exp[0] == "move" and prim[2] == exp[2] and known and prim[1] != known:
                return (f"修复要用**同一颗子**：上次失败后那颗子在 {known}，"
                        f"请 move({known},{exp[2]})——不能拿别的子（{prim[1]} 上的）顶替。")
        if self._pending is not None:
            nxt = "、".join(self._prim_desc(p) for p in self._next_prims())
            return (f"上一手（{self._pending_desc()}）还没做完——期待下一步：{nxt}。"
                    f"先把这手完成（若刚才失败了，按世界报的实际位置修复），再走别的。")
        turn = self._turn_cn()
        if prim[0] == "move":
            frm, to = prim[1], prim[2]
            pc = self.board.piece_at(chess.parse_square(frm))
            if pc is None:
                return f"{frm} 上没有子（按棋规记录）。先看清盘面再走。"
            if pc.color != self.board.turn:
                return f"现在轮到{turn}走，{frm} 上是对方的子。"
            cap = chess.Move(chess.parse_square(frm), chess.parse_square(to))
            if any(m.from_square == cap.from_square and m.to_square == cap.to_square
                   for m in self.board.legal_moves if self.board.is_capture(m)):
                return f"{frm}→{to} 是吃子——按拆解先 remove({to}) 拿掉被吃的子，再 move({frm},{to})。"
            # 注：易位不需要专门指路——能易位时「单走车」本身总是合法普通着（车路必空），
            # 车先动就按普通车着记账（易位权随之自然消失，python-chess 管）；王两格只能按易位拆解走。
            return f"{frm}→{to} 不是{turn}现在的合法走法。"
        if prim[0] == "remove":
            sq = prim[1]
            pc = self.board.piece_at(chess.parse_square(sq))
            if pc is None:
                return f"{sq} 是空格（按棋规记录），没子可拿。"
            if pc.color == self.board.turn:
                return f"{sq} 上是{turn}自己的子——remove 只用于吃对方的子（作为一手吃子的第一步）。"
            return f"现在没有任何合法着法能吃到 {sq} 上的子。"
        return ("place 只在升变的最后一步用（move 兵到底线 → remove 兵 → place 升变子），"
                "现在不在这一步。")

    # ---------- 推进 ----------
    def commit(self, prim: Prim) -> dict:
        """物理核实成功后调用。返回 {advanced, move, san, over, result, message}。"""
        if self._matches_next(prim):
            canonical = prim
        elif self._attempt is not None and self._repair_match(prim):
            canonical = self._attempt["prim"]
        else:   # 防御：world 必须先 check 再 commit；走到这说明调用序错了，如实报不装死
            return {"advanced": False, "move": None, "san": None, "over": self.over,
                    "result": self.result, "message": "（裁判内部：commit 与前置闸不一致，本原语未记账）"}
        self._attempt = None
        if self._pending is None:
            self._pending = [(m, expansion(self.board, m))
                             for m in self.board.legal_moves
                             if expansion(self.board, m)[0] == canonical]
            self._step = 1
        else:
            self._pending = [(m, seq) for m, seq in self._pending if seq[self._step] == canonical]
            self._step += 1
        done = [(m, seq) for m, seq in self._pending if len(seq) == self._step]
        if not done:
            nxt = "、".join(self._prim_desc(p) for p in self._next_prims())
            return {"advanced": False, "move": None, "san": None, "over": False, "result": "",
                    "message": f"这手棋（{self._pending_desc()}）还差：{nxt}"}
        m = done[0][0]
        san = self.board.san(m)
        self.board.push(m)
        self._pending, self._step = None, 0
        mover = "白方" if not self.board.turn else "黑方"     # push 后 turn 已翻转
        msg = f"棋局：{mover} {san}"
        if self.board.is_game_over(claim_draw=True):
            self.over = True
            out = self.board.outcome(claim_draw=True)
            self.result = ("draw" if out is None or out.winner is None
                           else ("white" if out.winner == chess.WHITE else "black"))
            self._log_game()
            msg += f"，对局结束：{self._result_cn()}"
        else:
            msg += f"，轮到{self._turn_cn()}"
        return {"advanced": True, "move": m.uci(), "san": san, "over": self.over,
                "result": self.result, "message": msg}

    def note_failure(self, prim: Prim, fail: str = "", piece_square: str | None = None) -> None:
        """物理失败回报：登记修复上下文（期待的原语 + 子的实际位置）并计数。序列指针不动。"""
        if self._matches_next(prim):
            canonical = prim
        elif self._attempt is not None and self._repair_match(prim):
            canonical = self._attempt["prim"]
        else:
            canonical = None
        if canonical is not None:
            self._attempt = {"prim": canonical, "square": piece_square}
        key = fail or "other"
        self.fails[key] = self.fails.get(key, 0) + 1

    def restoration_ok(self, square: str, letter: str) -> bool:
        """「备用子恢复」判定：真值棋盘上 square 格正好是这枚子（字母含大小写=颜色）。
        场景：子被物理弄丢（掉出棋盘找不回）而真值仍有它——下真棋这时就是拿备用子摆回原格。
        **物理上该格是否为空由世界核实**（裁判是盲的）；恢复不推进真值、不清进行中序列——
        盘面补回后，该走的那手棋照旧要走。2026-07-06 三盘耐力跑里两盘死于没有这条出路，
        且大脑三次本能地尝试 place 补子（被旧规则拒绝）——直觉正确，规则补上。"""
        try:
            pc = self.board.piece_at(chess.parse_square(square))
        except ValueError:
            return False
        return pc is not None and pc.symbol() == letter

    # ---------- 对手侧推进（W2 内置电脑用：走法不经物理原语，直接 push）----------
    def push_direct(self, m: chess.Move) -> dict:
        """把一手（对手瞬移走的）合法棋直接推进真值。返回同 commit 的 advanced 分支。"""
        assert self._pending is None, "有进行中的拆解序列时不该轮到对手"
        san = self.board.san(m)
        self.board.push(m)
        msg = ""
        if self.board.is_game_over(claim_draw=True):
            self.over = True
            out = self.board.outcome(claim_draw=True)
            self.result = ("draw" if out is None or out.winner is None
                           else ("white" if out.winner == chess.WHITE else "black"))
            self._log_game()
            msg = f"对局结束：{self._result_cn()}"
        return {"advanced": True, "move": m.uci(), "san": san, "over": self.over,
                "result": self.result, "message": msg}

    # ---------- 终局 / 落档 / 复位 ----------
    def _result_cn(self) -> str:
        return {"white": "白方胜", "black": "黑方胜", "draw": "和棋"}.get(self.result, "未分胜负")

    def _log_game(self) -> None:
        """完整对局落一行 games-*.jsonl（格式对齐 sim-chess，追加 world/white/black/physical_fails）。
        真数据：moves 直接取 move_stack 的 UCI；一盘只记一次；落档失败绝不影响对局。"""
        if self._logged or not self.log_games or not self.board.move_stack or not self.games_dir:
            return
        try:
            self._game_seq += 1
            bot_side = ("white" if self.white == "bot"
                        else "black" if self.black == "bot" else None)
            rec = {
                "ts": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "game_id": self._game_seq,
                "game": "chess",
                "world": self.world_name,
                "white": self.white,
                "black": self.black,
                "bot_side": bot_side,
                "result": self.result,
                "plies": len(self.board.move_stack),
                "moves": [m.uci() for m in self.board.move_stack],
                "physical_fails": dict(self.fails),
            }
            os.makedirs(self.games_dir, exist_ok=True)
            path = os.path.join(self.games_dir, "games-" + time.strftime("%Y-%m-%d") + ".jsonl")
            with open(path, "a", encoding="utf-8") as f:
                f.write(json.dumps(rec, ensure_ascii=False) + "\n")
            self._logged = True
        except Exception:  # noqa: BLE001
            pass

    def reset(self, setup_fen: str = "") -> None:
        """开新局（W2 的 /reset 用）：未落档的残局先如实落档（result=""），再换新棋盘。"""
        if self.board.move_stack and not self._logged:
            self._log_game()
        seq = self._game_seq
        self.__init__(setup_fen, white=self.white, black=self.black, games_dir=self.games_dir,
                      world_name=self.world_name, log_games=self.log_games)
        self._game_seq = seq

    def status(self) -> dict:
        """上帝视角（只走人类 /status，绝不进 perceive）。"""
        return {
            "fen": self.board.fen(),
            "turn": "white" if self.board.turn == chess.WHITE else "black",
            "over": self.over,
            "result": self.result,
            "pending": self._pending_desc() or None,
            "pending_next": [self._prim_desc(p) for p in self._next_prims()] if self._pending else None,
            "last_failure": dict(self._attempt) if self._attempt else None,
            "physical_fails": dict(self.fails),
            "moves": len(self.board.move_stack),
        }

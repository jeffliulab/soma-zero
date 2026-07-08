"""gazebo-chess 世界本体：一个「纯物理」的机械臂棋盘世界（新框架）。

对大脑（ANIMA）只给**画面**（俯视相机帧）+ **三个物理原语**——不含棋规、不含开局仪式、不判合法：
- `move(from,to)` ：把 from 格的子**裸搬**到 to 格（真夹真放，不判棋规——大脑才是裁判）。
- `remove(square)`：把 square 格的子夹起来、放到棋盘一侧的**弃子区**（吃子/清子）。
- `place(square,piece)`：从**备用子区**取一枚该色的新子、摆到 square（摆盘/升变）。

大脑自己把一手逻辑棋拆成这几个原语的序列（吃子=remove+move、升变=move+remove+place、易位=王先车后两次 move）。

裁判（v0.7，referee.py）：摆了 FEN 的对局模式下，世界内部持一份棋规真值（对齐 sim-chess
「世界=现实、可以懂棋规」的先例）——每个原语**动臂之前**先过合法闸（非法直接拒绝，臂不动），
物理核实成功后才推进真值，终局判定 + 棋谱落档都在世界内。无 FEN 的单演示子模式裁判自动关，
v0.4-0.6 的「裸物理原语」行为原样保留（T0）。

state（perceive 随画面给大脑的结构化部分）= **空 `{}`**：这个世界没有该给大脑的结构化真值，棋盘全靠画面看。
棋盘真值（每子在哪格 + 裁判的局面记录）只走人类调试台 /status（debug_state），绝不给 ANIMA；
大脑能听到的只有原语的 ok/fail 消息（拒绝原因/对局播报——那是「现实的反馈」，不是真值通道）。

线程（v0.5 wave 0）：ROS 的 spin 收敛到**唯一一个专职线程**（SingleThreadedExecutor，init 时启动）——
相机帧、/joint_states、service/action 回包全由它持续送达；请求工作线程对 ROS future 只「挂事件+带超时等」
（见 arm_controller._wait_future），**绝不自己 spin**（从请求线程 spin 会和 DDS/executor 撞线程，实测卡死）。
世界状态的互斥仍靠 self.lock。observe（喂大脑）**故意**等锁——动作没做完不该把"半空中的子"当棋盘状态；
stream_jpeg（人类直播）**故意不**等锁——帧由 spin 线程持续写入 CameraFeed，读一下是原子引用读，
人就是要看臂动的全程（v0.5 前直播也抢锁，臂一动画面就冻成"前后快照"，实锤修掉）。
长动作进度（v0.5 wave 0）：`invoke` 声明了 keyword-only `_progress`——awi_mcp 签名探测到它，就把
MCP progress 上报函数传进来；三个原语分阶段报人话进度（定位→抓取→搬运→放置→核对），大脑靠这些
"生命迹象"续命等待，用户在仪表盘/对弈面板实时看到臂在干什么。不带 _progress 调用（如测试）行为不变。
"""
from __future__ import annotations

import math
import os
import threading
import time

import rclpy

import chess

import config
import geometry
import gz_bot
import referee as referee_mod
import render
import spawn
from arm_controller import ArmController

# ---- AWI 工具声明：三个物理原语（大脑侧靠这仨的 expand_move 拆一手棋）----
MOVE_TOOL = {
    "name": "move",
    "description": "把盘上 from 格的子搬到 to 格（目标格必须是空格；有子会拒绝并提示——想换掉它就先 remove(to) 再 move）。"
                   "机械臂真夹真放（裸搬，不判棋规），放完核对落点，只回成败。",
    "parameters": {"type": "object",
                   "properties": {"from": {"type": "string", "description": "起格，如 e2"},
                                  "to": {"type": "string", "description": "目标格，如 e4"}},
                   "required": ["from", "to"]},
    "kind": "tool",
}
REMOVE_TOOL = {
    "name": "remove",
    "description": "把某格的子**从棋盘上拿下去**（夹到盘边弃子区）。用户说\"拿走/拿下去/清掉某格的子\"就是这个。只回成败。",
    "parameters": {"type": "object",
                   "properties": {"square": {"type": "string", "description": "要清掉的格，如 e5"}},
                   "required": ["square"]},
    "kind": "tool",
}
PLACE_TOOL = {
    "name": "place",
    "description": "**拿一枚新子上盘**：从盘边备用子区取一枚、摆到某个空格。用户说\"放一个子上来/在某格摆个子\"就是这个。"
                   "piece 用棋子字母：大写=白、小写=黑（如 Q/q/P/p）。只回成败。",
    "parameters": {"type": "object",
                   "properties": {"square": {"type": "string", "description": "要摆到的格，如 e1"},
                                  "piece": {"type": "string", "description": "棋子字母，大写白/小写黑，如 Q/q/P/p"}},
                   "required": ["square", "piece"]},
    "kind": "tool",
}
_TOOLS = [MOVE_TOOL, REMOVE_TOOL, PLACE_TOOL]

# perceive 的 state 契约声明（给 /awi 面板读）——只有相机名单（图↔相机的对应关系），绝无棋盘真值。
STATE_SCHEMA: dict = {"cameras": "本帧包含哪几路相机画面（名字顺序 = 图片顺序）"}

# 停臂驻位（让出俯视相机视野；和 _tune_camera 一致）
PARK = [float(x) for x in os.getenv("GZCHESS_PARK_JOINTS", "2.5,0,0,0,0,0").split(",")]

# 进度上报的粗颗粒阶段占比（0~1）：只是给人看的进度语义（定位→抓→搬→放→核对），不是精确测量。
_P_LOCATE, _P_PICK, _P_CARRY, _P_PLACE, _P_VERIFY = 0.1, 0.25, 0.5, 0.7, 0.9


class GazeboChessWorld:
    def __init__(self, demo_piece_square: str | None = None) -> None:
        self.lock = threading.RLock()
        self.last = ""
        self._discard_n = 0                 # 已用掉几个弃子槽（remove 递增）
        # 重试多样性（v0.7）：同一动作目标连续物理失败的计数——传给 pick_at/place_at 轮换候选起点
        # （确定性失手的解，重试一万次也一样；换姿态才是独立重试）。成功或换目标即清零。
        self._retry_n = 0
        self._retry_key = ""
        # 裁判（v0.7）：对局模式（摆 FEN）才建；标签=非 bot 侧是 anima（bot 关掉时对面标 opponent）。
        self.referee = None
        self._bot = None
        if config.referee_enabled():
            labels = {"white": "anima", "black": "opponent"}
            if config.BOT_SIDE in ("white", "black"):
                labels = ({"white": "bot", "black": "anima"} if config.BOT_SIDE == "white"
                          else {"white": "anima", "black": "bot"})
                self._bot = gz_bot.AI(depth=config.BOT_DEPTH, time_limit=config.BOT_TIME)
            self.referee = referee_mod.Referee(config.SETUP_FEN, games_dir=config.GAMES_LOG_DIR,
                                               **labels)
        # 失败注入状态（once 模式注入一次即耗尽；见 config.FAIL_*）
        self._fail_grip = config.FAIL_GRIP_MISS
        self._fail_place = config.FAIL_PLACE_MODE
        # ROS：一个节点（ArmController）+ 挂相机订阅
        if not rclpy.ok():
            rclpy.init()
        self.arm = ArmController()
        # 多相机：一路一个 CameraFeed、一路一个独立话题（config.cam_names 定几路；顺序=observation 图片顺序）
        self.cams = {n: render.CameraFeed(self.arm, config.cam_topic(n)) for n in config.cam_names()}
        # 专职 ROS spin 线程：唯一允许 spin 这个节点的地方（回调异常只记日志，不许杀线程）。
        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self.arm)
        self._spin_alive = True
        threading.Thread(target=self._spin_forever, name="gzchess-ros-spin", daemon=True).start()
        self.ready = self.arm.wait_ready(config.READY_TIMEOUT_S)
        # 布场景：棋盘 + 相机 +（演示子）；停臂驻位
        self._demo_square = demo_piece_square or os.getenv("GZCHESS_DEMO_PIECE", "e2")
        self._setup(self._demo_square)

    def _spin_forever(self) -> None:
        while self._spin_alive and rclpy.ok():
            try:
                self._executor.spin_once(timeout_sec=config.SPIN_STEP_S)
            except Exception:  # noqa: BLE001  回调抛错不许杀掉唯一的 spin 线程
                pass

    def _spin(self, n: int = 5) -> None:
        """等专职 spin 线程送达新数据的节拍（历史名保留：以前是自己 spin，现在只等）。"""
        time.sleep(n * config.SPIN_STEP_S)

    _ALL_CAM_KINDS = ("oblique", "overhead")   # 已知相机全集（换模式重启后要把不用的残留也清掉）

    def _setup(self, demo_square: str) -> None:
        with self.lock:
            spawn.spawn_board()
            # 相机：先把**所有已知相机名**的残留删净（含换模式前留下的），再按本模式逐路 spawn。
            for kind in self._ALL_CAM_KINDS:
                spawn.purge_model(f"{kind}_cam")
            for kind in config.cam_names():
                spawn.spawn_camera(kind)
                n_pub = spawn.publisher_count(config.cam_topic(kind))
                if n_pub != 1:   # 发布者数自检：>1 = 残留相机/孤儿 gz 进程在抢话题（画面会交替混流）
                    print(f"[gazebo-chess] ⚠️ 话题 {config.cam_topic(kind)} 有 {n_pub} 个发布者"
                          f"（应为 1）——查残留相机模型或孤儿 gz sim 进程（pgrep -af 'gz sim'）")
            # 先清掉上一次会话/脚本残留的棋子模型（和相机清扫同理：gz 世界是长活的，不清=污染摆盘）
            for nm in list(spawn.all_model_poses()):
                if nm.startswith("piece_"):
                    spawn.purge_model(nm)
            fen = config.SETUP_FEN.strip()
            if fen:                                     # v0.5 多子：按 FEN 摆放字段摆盘
                self._spawn_fen(fen)
            elif demo_square:                           # v0.4 单演示子路径原样保留（追加不替换）
                spawn.spawn_piece(demo_square, "white")
            if self.ready:
                self.arm.goto_arm(PARK, config.MOVE_TIME_APPROACH_S)
            time.sleep(config.SETTLE_S)
            self._spin(20)
            # 开局就轮到对手（bot 执白）→ 它先瞬移走一手
            if self.referee is not None and self._bot is not None and not self.referee.over:
                bot_color = chess.WHITE if config.BOT_SIDE == "white" else chess.BLACK
                if self.referee.board.turn == bot_color:
                    print("[gazebo-chess] 对手执白先行：", self._bot_reply())

    @staticmethod
    def _spawn_fen(fen: str) -> None:
        """按 FEN 的【摆放字段】摆多子（大写白/小写黑；不管轮次/易位权——那是大脑的事）。
        手工解析、不引 python-chess（世界 venv 不为此加依赖；FEN 摆放字段语法是域常量级的简单格式）。"""
        rows = fen.split()[0].split("/")
        for i, row in enumerate(rows[:8]):
            rank = 7 - i                                 # FEN 第一行是 rank8
            file = 0
            for ch in row:
                if ch.isdigit():
                    file += int(ch)
                    continue
                if ch.lower() in "pnbrqk" and file < 8:
                    sq = geometry.square_name(file, rank)
                    spawn.spawn_piece(sq, "white" if ch.isupper() else "black", kind=ch.lower())
                    file += 1

    # ---------- AWI ----------
    def capabilities(self) -> dict:
        return {"name": "gazebo-chess", "version": config.WORLD_VERSION, "tools": _TOOLS,
                "state_schema": STATE_SCHEMA}

    def debug_state(self) -> dict:
        """【人类调试台专用·世界真值，绝不给 ANIMA】走世界本地 /status。
        返回每个棋子现在真实在哪格 + 精确位姿——这是人的『上帝视角』，和 perceive（给空 state）明确分开。"""
        with self.lock:
            self._spin(4)
            pieces = {}
            for nm, pp in spawn.all_model_poses(window_s=config.STATUS_POSE_WINDOW_S).items():   # 短窗口:调试台要快点响应
                if not nm.startswith("piece_"):
                    continue
                pieces[nm] = {"square": geometry.base_xy_to_square(pp[0], pp[1]),
                              "xyz": [round(v, 4) for v in pp]}
            out = {"pieces": pieces, "discard_used": self._discard_n}
            if self.referee:   # 裁判真值（fen/轮次/终局/进行中序列/物理失败计数）——上帝视角，允许
                out["referee"] = self.referee.status()
            return out

    def observe(self) -> tuple[dict, list[tuple[str, bytes]]]:
        """给画面 + 极简 state。多相机：每路一张命名图，state.cameras 按序列名字（这就是图↔相机的
        对应关系，交给大脑）。绝不给棋盘真值——state 里只有相机名单，没有任何局面信息。"""
        with self.lock:
            self._spin(6)
            images = []
            for name, feed in self.cams.items():
                png = render.to_png(feed.frame)
                if png:
                    images.append((name, png))
        return {"cameras": [n for n, _ in images]}, images

    def stream_jpeg(self, cam: str = "") -> bytes | None:
        """某一路相机的直播帧（人类页用）。cam 空 = 第一路。
        **不取 self.lock、不等节拍**：帧由专职 spin 线程持续更新，读 feed.frame 是原子引用读；
        直播若等锁，机械臂动作（几十秒持锁）期间画面会整段冻结——人恰恰要看臂动的全程。"""
        feed = self.cams.get(cam) or next(iter(self.cams.values()), None)
        return render.to_jpeg(feed.frame) if feed else None

    def cleanup_cameras(self) -> None:
        """进程退出前把本世界 spawn 的相机模型删净（uvicorn 重启也走这里——防残留相机抢话题）。"""
        for kind in self._ALL_CAM_KINDS:
            spawn.purge_model(f"{kind}_cam")

    def invoke(self, name: str, *, _progress=None, **args) -> dict:
        # `_progress(比例, 人话消息)` 由 awi_mcp 签名探测注入（MCP progress 上报）；没有就静默。
        note = _progress or (lambda p, m="": None)
        if name == "move":
            return self._move(args, note)
        if name == "remove":
            return self._remove((args.get("square", "") or "").strip().lower(), note)
        if name == "place":
            return self._place((args.get("square", "") or "").strip().lower(), args.get("piece", ""), note)
        return {"ok": False, "message": f"未知能力：{name}"}

    # ---------- 按当前位置找某格上的子 ----------
    def _piece_at(self, square: str):
        """返回 (model_name, (x,y,z))，没有则 (None, None)。判据：piece_* 模型里 (x,y) 离该格中心最近且在半格内。"""
        ex, ey, _ = geometry.square_surface_xyz(square)
        best, bestp, bestd = None, None, 1e9
        for nm, pp in spawn.all_model_poses().items():
            if not nm.startswith("piece_"):
                continue
            d = math.hypot(pp[0] - ex, pp[1] - ey)
            if d < bestd:
                best, bestd, bestp = nm, d, pp
        if best is not None and bestd <= config.CELL_M * config.PIECE_MATCH_CELL_FRAC:
            return best, bestp
        return None, None

    def _park(self) -> None:
        if self.ready:
            self.arm.goto_arm(PARK, config.MOVE_TIME_APPROACH_S)
            self._spin(10)

    @staticmethod
    def _others_xy(exclude_name: str) -> list[tuple[float, float]]:
        """除目标子外，盘上其它棋子的水平坐标——给抓取规划避撞（世界=物理，用自己的真值是本分）。"""
        return [(pp[0], pp[1]) for nm, pp in spawn.all_model_poses().items()
                if nm.startswith("piece_") and nm != exclude_name]

    def _consume_inject(self, attr: str) -> bool:
        """失败注入调度：off→False；once→注入一次后自动关；always→每次注入。"""
        mode = getattr(self, attr)
        if mode == "once":
            setattr(self, attr, "off")
            return True
        return mode == "always"

    # ---------- 重试多样性（v0.7）----------
    def _retry_rotation(self, key: str) -> int:
        """本次动作的候选轮换位数：同一目标（key=动作+格）连上次失败相同 → 用当前计数；换目标 → 清零。"""
        if key != self._retry_key:
            self._retry_key, self._retry_n = key, 0
        return self._retry_n

    def _retry_note(self, key: str, ok: bool) -> None:
        """动作收尾登记：成功清零；失败且同目标 → 计数 +1（下次轮换到新首选）。"""
        if ok or key != self._retry_key:
            self._retry_key, self._retry_n = ("", 0) if ok else (key, 1)
        else:
            self._retry_n += 1

    # ---------- 裁判记账 + 内置对手应手（持锁内调用）----------
    def _after_commit(self, prim: tuple, note) -> str:
        """原语物理核实成功后的收尾：裁判记账；凑完一手且轮到对手 → 对手瞬移应一手。
        返回要追加进 ok 消息的播报（前带 ｜）。"""
        info = self.referee.commit(prim)
        msg = "｜" + info["message"]
        if info["advanced"] and not info["over"] and self._bot is not None:
            bot_color = chess.WHITE if config.BOT_SIDE == "white" else chess.BLACK
            if self.referee.board.turn == bot_color:
                note(_P_VERIFY, "对手思考中…")
                msg += "｜" + self._bot_reply()
        if self.referee.over and config.AUTO_RESET:
            threading.Timer(config.AUTO_RESET_DELAY_S, self.reset_board).start()
            msg += f"｜{config.AUTO_RESET_DELAY_S:.0f} 秒后自动开新局"
        return msg

    def _bot_reply(self) -> str:
        """内置电脑对手应一手：算棋（gz_bot 副本）→ 物理瞬移（set_pose/purge/spawn，不用机械臂）
        → 真值推进（push_direct）。**物理应用全部成功才推进真值**——半路失败就如实报、真值不动，
        人可按「开新局」收拾。返回播报（按设计不说走了哪步——你下次看画面自己认）。"""
        m = self._bot.best_move(self.referee.board.copy())
        if m is None:                      # 未终局却无棋可走不该发生（终局在 commit 已判）
            return "对手没有可走的棋（异常，请查 /status）"
        ok, why = self._apply_bot_move(m)
        if not ok:
            return f"对手应手失败：{why}（真值未推进——物理和棋规记录可能已不一致，建议开新局）"
        info = self.referee.push_direct(m)
        time.sleep(config.SPAWN_SETTLE_S)  # 瞬移后给物理/画面一个稳定节拍再返回
        tail = f"，{info['message']}" if info["message"] else ""
        return f"对手已应一手（看画面认它走了哪步）{tail}"

    def _apply_bot_move(self, m: "chess.Move") -> tuple[bool, str]:
        """把对手的一手棋物理落到 Gazebo（瞬移）。board 仍是走这手之前的局面。"""
        board = self.referee.board
        frm, to = chess.square_name(m.from_square), chess.square_name(m.to_square)
        name, _ = self._piece_at(frm)
        if name is None:
            return False, f"{frm} 上找不到对手的子（物理与棋规记录分叉）"
        # 1) 吃子：先把被吃的子从世界删掉（对手侧不走弃子区，直接消失——它不是机械臂）
        cap_sq = None
        if board.is_en_passant(m):
            cap_sq = chess.square_name(chess.square(chess.square_file(m.to_square),
                                                    chess.square_rank(m.from_square)))
        elif board.is_capture(m):
            cap_sq = to
        if cap_sq:
            victim, _ = self._piece_at(cap_sq)
            if victim is None:
                return False, f"被吃格 {cap_sq} 上找不到子（物理与棋规记录分叉）"
            spawn.purge_model(victim)
        # 2) 升变：兵模型换成升变子；否则瞬移本体
        if m.promotion:
            color = "white" if board.turn == chess.WHITE else "black"
            spawn.purge_model(name)
            ok, out = spawn.spawn_piece(to, color, kind=chess.piece_symbol(m.promotion))
            if not ok:
                return False, f"升变子 spawn 失败：{out}"
        else:
            tx, ty, tz = geometry.square_surface_xyz(to)
            ok, out = spawn.set_model_pose(name, (tx, ty, tz))
            if not ok:
                return False, f"瞬移失败：{out}"
            spawn.note_square(name, to)
        # 3) 易位：车也跟着瞬移
        if board.is_castling(m):
            kingside = chess.square_file(m.to_square) == 6
            rank = chess.square_rank(m.from_square)
            rf = chess.square_name(chess.square(7 if kingside else 0, rank))
            rt = chess.square_name(chess.square(5 if kingside else 3, rank))
            rook, _ = self._piece_at(rf)
            if rook is None:
                return False, f"易位的车不在 {rf}（物理与棋规记录分叉）"
            rx, ry, rz = geometry.square_surface_xyz(rt)
            ok, out = spawn.set_model_pose(rook, (rx, ry, rz))
            if not ok:
                return False, f"车瞬移失败：{out}"
            spawn.note_square(rook, rt)
        return True, ""

    # ---------- move = 裸搬（真夹真放，不判棋规）----------
    def _move(self, args: dict, note=lambda p, m="": None) -> dict:
        frm = (args.get("from", "") or "").strip().lower()
        to = (args.get("to", "") or "").strip().lower()
        with self.lock:
            if not self.ready:
                return {"ok": False, "message": "机械臂/MoveIt 没就绪"}
            try:
                geometry.parse_square(frm); geometry.parse_square(to)
            except ValueError as e:
                return {"ok": False, "message": f"格名非法：{e}"}
            name, p = self._piece_at(frm)          # 按当前位置找（子走一步后名字不变、位置才准）
            if name is None:
                return {"ok": False, "message": f"{frm} 格上没有子"}
            occ, _ = self._piece_at(to)            # 目标格占用检查：叠子=物理事故，如实拒绝、指路两步走
            if occ is not None and occ != name:
                return {"ok": False,
                        "message": f"{to} 格上已经有子——想换掉它就先 remove({to}) 再 move；想放旁边就换个空格。"}
            if self.referee:                       # 前置合法闸：非法零物理动作（一次臂动 26 秒，别浪费）
                legal, why = self.referee.check(("move", frm, to))
                if not legal:
                    return {"ok": False, "message": f"裁判拒绝：{why}", "data": {"refused": "referee"}}
            note(_P_LOCATE, f"已定位 {frm} 上的棋子，正在规划抓取")
            rkey = f"move:{frm}->{to}"
            rot = self._retry_rotation(rkey)     # 重试多样性：同目标连续失败 → 换候选姿态起点
            gx, gy, gz = p[0], p[1], p[2] + config.PIECE_GRASP_WAIST_M
            avoid = self._others_xy(name)
            ok, msg = self.arm.pick_at(gx, gy, gz, progress=lambda m: note(_P_PICK, m),
                                       inject_miss=self._consume_inject("_fail_grip"), avoid_xy=avoid,
                                       rotate_candidates=rot)
            if not ok:
                self._retry_note(rkey, False)
                if self.referee:
                    self.referee.note_failure(("move", frm, to), "pick_fail", frm)  # 子没动，还在 frm
                self._park(); return {"ok": False, "message": f"抓取失败：{msg}"}
            note(_P_CARRY, f"已夹取，正在移向 {to}")
            dx, dy, dz = geometry.grasp_xyz(to)
            if self._consume_inject("_fail_place"):     # 放偏注入：放置点加偏移（物理后果如实保留）
                dx += config.FAIL_PLACE_OFFSET_M
            ok, msg = self.arm.place_at(dx, dy, dz, progress=lambda m: note(_P_PLACE, m), avoid_xy=avoid,
                                        rotate_candidates=rot)
            if not ok:
                self._retry_note(rkey, False)
                if self.referee:   # 放置中途失败：子可能还夹着/掉在路上，实际位置世界也说不准 → 不报格
                    self.referee.note_failure(("move", frm, to), "place_fail", None)
                self._park(); return {"ok": False, "message": f"放置失败：{msg}"}
            note(_P_VERIFY, "已放下，正在核对落点")
            time.sleep(config.SETTLE_S); self._spin(10)
            p2 = spawn.model_pose(name)
            exp = geometry.square_surface_xyz(to)
            err = math.hypot(p2[0] - exp[0], p2[1] - exp[1]) if p2 else 9.99
            self._park()
            if err <= config.PLACE_TOLERANCE_M:
                self._retry_note(rkey, True)
                self.last = f"move {frm}->{to}"
                suffix = ""
                if self.referee:   # 物理核实成功 → 裁判记账（凑完一手 → 真值前进 + 可能触发对手应手）
                    suffix = self._after_commit(("move", frm, to), note)
                return {"ok": True,
                        "message": f"已把子从 {frm} 搬到 {to}（落点误差 {err * 100:.1f}cm）{suffix}"}
            self._retry_note(rkey, False)
            res = self._classify_move_failure(name, frm, to, p2, err)
            if self.referee:       # 失败自检结果同步给裁判：登记修复上下文（序列指针不动）
                self.referee.note_failure(("move", frm, to), res["data"]["fail"],
                                          res["data"]["piece_square"])
            return res

    def _classify_move_failure(self, name: str, frm: str, to: str, p2, err: float) -> dict:
        """执行自检分类（v1.1：自检是早停提示，最终判据仍是大脑的视觉裁判）。
        返回的 data 是【动作结果自检】——失败类别 + 子的实际落格，供大脑规划补救；
        不是感知通道，不泄露整盘真值（红线不破）。物理后果如实保留，绝不摆回去装成功。"""
        if p2 is None:
            return {"ok": False, "message": f"放完找不到这枚子了（{frm}→{to}），可能掉出场地",
                    "data": {"fail": "drop", "piece_square": None}}
        actual = geometry.base_xy_to_square(p2[0], p2[1])
        fx, fy, _ = geometry.square_surface_xyz(frm)
        if math.hypot(p2[0] - fx, p2[1] - fy) <= config.CELL_M * config.PIECE_MATCH_CELL_FRAC:
            return {"ok": False, "message": f"夹空了：子还留在 {frm}（没夹起来）",
                    "data": {"fail": "grip_miss", "piece_square": frm}}
        if actual is None:
            return {"ok": False, "message": f"子掉到棋盘外了（{frm}→{to} 途中）",
                    "data": {"fail": "drop", "piece_square": None}}
        return {"ok": False, "message": f"放偏了：落点离 {to} 中心 {err * 100:.1f}cm（实际在 {actual}）",
                "data": {"fail": "place_offset", "piece_square": actual}}

    # ---------- remove = 夹起丢弃子区 ----------
    def _remove(self, square: str, note=lambda p, m="": None) -> dict:
        with self.lock:
            if not self.ready:
                return {"ok": False, "message": "机械臂/MoveIt 没就绪"}
            try:
                geometry.parse_square(square)
            except ValueError as e:
                return {"ok": False, "message": f"格名非法：{e}"}
            name, p = self._piece_at(square)
            if name is None:
                return {"ok": False, "message": f"{square} 格上没有子"}
            if self.referee:                       # 前置合法闸：remove 只用于合法吃子的第一步
                legal, why = self.referee.check(("remove", square))
                if not legal:
                    return {"ok": False, "message": f"裁判拒绝：{why}", "data": {"refused": "referee"}}
            note(_P_LOCATE, f"已定位 {square} 上的棋子，准备夹去弃子区")
            rkey = f"remove:{square}"
            rot = self._retry_rotation(rkey)
            gx, gy, gz = p[0], p[1], p[2] + config.PIECE_GRASP_WAIST_M
            avoid = self._others_xy(name)
            ok, msg = self.arm.pick_at(gx, gy, gz, progress=lambda m: note(_P_PICK, m), avoid_xy=avoid,
                                       rotate_candidates=rot)
            if not ok:
                self._retry_note(rkey, False)
                if self.referee:
                    self.referee.note_failure(("remove", square), "pick_fail", square)
                self._park(); return {"ok": False, "message": f"抓 {square} 的子失败：{msg}"}
            note(_P_CARRY, "已夹取，正在移向弃子区")
            # bin 模式（v0.7 默认）：固定「弃子袋」一个点，放稳后模型销毁（袋子吞掉）——
            # 一盘最多 30 次吃子，槽位摆开第 2 排就超臂展；slots 模式=旧行为原样保留（T0）。
            if config.DISCARD_MODE == "bin":
                bx, by = config.DISCARD_BIN_XY
                dx, dy, dz = bx, by, config.OFFBOARD_SURFACE_Z + config.PIECE_GRASP_WAIST_M
            else:
                dx, dy, dz = geometry.discard_grasp_xyz(self._discard_n)
            ok, msg = self.arm.place_at(dx, dy, dz, progress=lambda m: note(_P_PLACE, m), avoid_xy=avoid,
                                        rotate_candidates=rot)
            self._park()
            if not ok:
                self._retry_note(rkey, False)
                if self.referee:   # 半路失败：子在哪世界说不准（可能还夹着/掉在路上），修复放开格名
                    self.referee.note_failure(("remove", square), "discard_fail", None)
                return {"ok": False, "message": f"丢到弃子区失败：{msg}"}
            self._retry_note(rkey, True)
            if config.DISCARD_MODE == "bin":
                spawn.purge_model(name)     # 放稳后销毁=袋子吞掉；搬运的真实代价已如实花掉
            self._discard_n += 1
            self.last = f"remove {square}"
            suffix = ""
            if self.referee:
                suffix = self._after_commit(("remove", square), note)
            return {"ok": True, "message": f"已把 {square} 的子移出棋盘（第 {self._discard_n} 个）{suffix}"}

    # ---------- place = 从备用子区取子摆上盘 ----------
    def _place(self, square: str, piece: str, note=lambda p, m="": None) -> dict:
        with self.lock:
            if not self.ready:
                return {"ok": False, "message": "机械臂/MoveIt 没就绪"}
            try:
                geometry.parse_square(square)
            except ValueError as e:
                return {"ok": False, "message": f"格名非法：{e}"}
            letter = (piece or "").strip()
            if not letter or letter.lower() not in "pnbrqk":
                return {"ok": False, "message": f"piece 非法：{piece!r}（应为棋子字母 P/N/B/R/Q/K，大写白小写黑）"}
            restoration = False
            if self.referee:                       # 前置合法闸：place 用于升变最后一步，或「备用子恢复」
                legal, why = self.referee.check(("place", square, letter))
                if not legal:
                    # 备用子恢复：真值该格正是这枚子、而物理上格是空的（子被弄丢了）——
                    # 下真棋这时就是拿备用子摆回原格。恢复不动真值、不清进行中的那手棋。
                    occ, _ = self._piece_at(square)
                    if occ is None and self.referee.restoration_ok(square, letter):
                        restoration = True
                    else:
                        return {"ok": False, "message": f"裁判拒绝：{why}", "data": {"refused": "referee"}}
            color = "white" if letter.isupper() else "black"
            note(_P_LOCATE, f"在备用子区备一枚{color}子")
            rx, ry, rz = geometry.reservoir_spawn_xyz()
            ok, nm = spawn.spawn_piece_at((rx, ry, rz), color, kind=letter.lower())
            if not ok:
                return {"ok": False, "message": f"备用子区取子失败：{nm}"}
            time.sleep(config.SPAWN_SETTLE_S); self._spin(8)
            grx, gry, grz = geometry.reservoir_grasp_xyz()
            rkey = f"place:{square}"
            rot = self._retry_rotation(rkey)
            avoid = self._others_xy(nm)
            ok, msg = self.arm.pick_at(grx, gry, grz, progress=lambda m: note(_P_PICK, m), avoid_xy=avoid,
                                       rotate_candidates=rot)
            if not ok:
                self._retry_note(rkey, False)
                if self.referee:
                    self.referee.note_failure(("place", square, letter), "pick_fail", None)
                self._park(); return {"ok": False, "message": f"从备用区抓子失败：{msg}"}
            note(_P_CARRY, f"已从备用区夹取，正在移向 {square}")
            dx, dy, dz = geometry.grasp_xyz(square)
            ok, msg = self.arm.place_at(dx, dy, dz, progress=lambda m: note(_P_PLACE, m), avoid_xy=avoid,
                                        rotate_candidates=rot)
            self._park()
            if not ok:
                self._retry_note(rkey, False)
                if self.referee:
                    self.referee.note_failure(("place", square, letter), "place_fail", None)
                return {"ok": False, "message": f"摆到 {square} 失败：{msg}"}
            self._retry_note(rkey, True)
            self.last = f"place {letter}@{square}"
            if restoration:   # 恢复不进裁判记账：真值本来就有这枚子，物理补回=盘面重新对齐
                return {"ok": True,
                        "message": f"已把一枚{color}子补回 {square}（备用子恢复：盘面已与棋局记录对齐，"
                                   f"接着走你该走的那手棋）"}
            suffix = ""
            if self.referee:
                suffix = self._after_commit(("place", square, letter), note)
            return {"ok": True, "message": f"已把一枚{color}子摆到 {square}{suffix}"}

    def reset_board(self) -> dict:
        """开新局（人类侧：POST /reset / 网页「开新局」按钮；**不进 MCP**——大脑不许重置现实，
        与 sim-chess「复位是网页的事」同构）。残局先如实落档（result=""）→ 清盘 → 按 FEN 重摆 →
        裁判复位 → 弃子计数清零 → 停臂驻位；bot 执白则它先走。"""
        with self.lock:
            if self.referee is not None:
                self.referee.reset(config.SETUP_FEN)
            for nm in list(spawn.all_model_poses()):
                if nm.startswith("piece_"):
                    spawn.purge_model(nm)
            self._discard_n = 0
            fen = config.SETUP_FEN.strip()
            if fen:
                self._spawn_fen(fen)
            elif self._demo_square:
                spawn.spawn_piece(self._demo_square, "white")
            self._park()
            time.sleep(config.SETTLE_S)
            self._spin(10)
            first = ""
            if self.referee is not None and self._bot is not None and not self.referee.over:
                bot_color = chess.WHITE if config.BOT_SIDE == "white" else chess.BLACK
                if self.referee.board.turn == bot_color:
                    first = self._bot_reply()
            self.last = "reset"
            return {"ok": True, "message": "已开新局" + (f"｜{first}" if first else "")}

    def shutdown(self) -> None:
        """完整关停（uvicorn lifespan 退出时调）：先删净相机（防残留抢话题），再停 ROS spin/节点。
        顺序要紧：purge 用 gz CLI 不依赖本进程 ROS，放最前；spin 线程是 daemon、置 False 即自然停。"""
        self.cleanup_cameras()
        self._spin_alive = False
        try:
            self._executor.shutdown(timeout_sec=1.0)
        except Exception:  # noqa: BLE001
            pass
        try:
            self.arm.destroy_node()
        except Exception:  # noqa: BLE001
            pass

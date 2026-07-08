"""gazebo-chess 裁判（../referee.py）的离线测试：
标准拆解表（普通/吃子/过路兵/易位/升变/吃子升变）、前置闸拒绝（不轮到/半途换招/错用原语）、
物理失败修复放宽（目标格一致即接受）、终局判定 + 对局落档格式。

纯逻辑、零 ROS：referee.py 不 import 世界 config，用 importlib 按文件路径加载。

⚠️ 本测试随 gazebo-chess 从 anima-zero 迁入 soma-zero/sim（2026-07-08）。soma-zero 尚未搭好
pytest 环境（无 venv/依赖），暂时不跑；需要 python-chess + pytest 到位后即可运行。
"""
from __future__ import annotations

import importlib.util
import json
import os

import chess
import pytest

_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "referee.py")
_spec = importlib.util.spec_from_file_location("gz_referee", os.path.abspath(_PATH))
gz_referee = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(gz_referee)

START = chess.STARTING_FEN


def _ref(fen: str = START, **kw) -> "gz_referee.Referee":
    kw.setdefault("log_games", False)
    return gz_referee.Referee(fen, **kw)


def _do(ref, prim):
    """check 必须放行，然后 commit（模拟物理成功）。返回 commit 结果。"""
    ok, why = ref.check(prim)
    assert ok, f"{prim} 被拒：{why}"
    return ref.commit(prim)


# ---------- 拆解表 ----------

def test_expansion_table():
    b = chess.Board()
    assert gz_referee.expansion(b, chess.Move.from_uci("e2e4")) == [("move", "e2", "e4")]
    b = chess.Board("rnbqkbnr/ppp1pppp/8/3p4/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2")
    assert gz_referee.expansion(b, chess.Move.from_uci("e4d5")) == [
        ("remove", "d5"), ("move", "e4", "d5")]
    # 过路兵：白 e5 兵吃 d5 兵到 d6，被吃兵在 d5
    b = chess.Board("rnbqkbnr/ppp1pppp/8/3pP3/8/8/PPPP1PPP/RNBQKBNR w KQkq d6 0 3")
    assert gz_referee.expansion(b, chess.Move.from_uci("e5d6")) == [
        ("remove", "d5"), ("move", "e5", "d6")]
    # 白短易位：王先车后
    b = chess.Board("r1bqkbnr/pppp1ppp/2n5/4p3/2B1P3/5N2/PPPP1PPP/RNBQK2R w KQkq - 4 4")
    assert gz_referee.expansion(b, chess.Move.from_uci("e1g1")) == [
        ("move", "e1", "g1"), ("move", "h1", "f1")]
    # 升变（不吃子）：move → remove 兵 → place 新子（白=大写字母）
    b = chess.Board("8/P7/8/8/8/8/8/K1k5 w - - 0 1")
    assert gz_referee.expansion(b, chess.Move.from_uci("a7a8q")) == [
        ("move", "a7", "a8"), ("remove", "a8"), ("place", "a8", "Q")]
    # 吃子升变：最前面再加 remove(to)
    b = chess.Board("1n6/P7/8/8/8/8/8/K1k5 w - - 0 1")
    assert gz_referee.expansion(b, chess.Move.from_uci("a7b8q")) == [
        ("remove", "b8"), ("move", "a7", "b8"), ("remove", "b8"), ("place", "b8", "Q")]


def test_board_from_setup_placement_only():
    b = gz_referee.board_from_setup("4k3/8/8/8/8/8/4P3/4K3")
    assert b.turn == chess.WHITE and not b.castling_rights   # 王车不在位 → 易位权收敛为空
    assert gz_referee.board_from_setup("").fen() == chess.STARTING_FEN
    assert gz_referee.board_from_setup(START).fen() == START


# ---------- 普通走子 / 拒绝 ----------

def test_plain_move_advances_turn():
    ref = _ref()
    info = _do(ref, ("move", "e2", "e4"))
    assert info["advanced"] and info["move"] == "e2e4" and not info["over"]
    assert ref.status()["turn"] == "black"
    assert "轮到黑方" in info["message"]


def test_illegal_and_wrong_turn_refused():
    ref = _ref()
    ok, why = ref.check(("move", "e2", "e5"))
    assert not ok and "不是白方现在的合法走法" in why
    ok, why = ref.check(("move", "e7", "e5"))          # 黑子，没轮到
    assert not ok and "对方的子" in why
    ok, why = ref.check(("move", "e4", "e5"))          # 空格
    assert not ok and "没有子" in why
    assert ref.status()["moves"] == 0                  # 拒绝不留任何状态


def test_capture_requires_remove_first_and_pending_locks():
    ref = _ref("rnbqkbnr/ppp1pppp/8/3p4/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2")
    ok, why = ref.check(("move", "e4", "d5"))          # 直接 move 吃子 → 指路 remove 先
    assert not ok and "先 remove(d5)" in why
    info = _do(ref, ("remove", "d5"))
    assert not info["advanced"] and "还差" in info["message"]
    ok, why = ref.check(("move", "g1", "f3"))          # 半途换招 → 拒绝
    assert not ok and "还没做完" in why
    info = _do(ref, ("move", "e4", "d5"))
    assert info["advanced"] and info["move"] == "e4d5"


def test_remove_refusals():
    ref = _ref()
    ok, why = ref.check(("remove", "e2"))              # 自己的子
    assert not ok and "自己的子" in why
    ok, why = ref.check(("remove", "e5"))              # 空格
    assert not ok and "空格" in why
    ok, why = ref.check(("remove", "e7"))              # 对方子但吃不到
    assert not ok and "没有任何合法着法能吃" in why


# ---------- 易位 / 过路兵 / 升变 ----------

def test_castling_king_first():
    fen = "r1bqkbnr/pppp1ppp/2n5/4p3/2B1P3/5N2/PPPP1PPP/RNBQK2R w KQkq - 4 4"
    # 先动车不是错——那就是一步合法的普通车着（Rf1），易位权随之消失（棋规语义，python-chess 管）
    ref = _ref(fen)
    info = _do(ref, ("move", "h1", "f1"))
    assert info["advanced"] and info["san"] == "Rf1"
    assert not (ref.board.castling_rights & chess.BB_H1)
    # 真易位：王两格（只能按易位拆解）→ 车跟上，两原语凑一手 O-O
    ref = _ref(fen)
    info = _do(ref, ("move", "e1", "g1"))
    assert not info["advanced"] and "还差" in info["message"]
    info = _do(ref, ("move", "h1", "f1"))
    assert info["advanced"] and info["san"] == "O-O"
    assert ref.board.piece_at(chess.parse_square("g1")).piece_type == chess.KING


def test_en_passant_sequence():
    ref = _ref("rnbqkbnr/ppp1pppp/8/3pP3/8/8/PPPP1PPP/RNBQKBNR w KQkq d6 0 3")
    _do(ref, ("remove", "d5"))                          # 被吃兵在 d5（不是落点 d6）
    info = _do(ref, ("move", "e5", "d6"))
    assert info["advanced"] and info["move"] == "e5d6"


def test_promotion_sequence_and_letter_case():
    ref = _ref("8/P7/8/8/8/8/8/K1k5 w - - 0 1")
    _do(ref, ("move", "a7", "a8"))
    _do(ref, ("remove", "a8"))
    ok, why = ref.check(("place", "a8", "q"))          # 黑字母 → 拒（白方升变）
    assert not ok
    info = _do(ref, ("place", "a8", "Q"))
    assert info["advanced"] and info["move"] == "a7a8q"
    assert ref.board.piece_at(chess.parse_square("a8")).piece_type == chess.QUEEN


def test_capture_promotion_sequence():
    ref = _ref("1n6/P7/8/8/8/8/8/K1k5 w - - 0 1")
    _do(ref, ("remove", "b8"))
    _do(ref, ("move", "a7", "b8"))
    _do(ref, ("remove", "b8"))
    info = _do(ref, ("place", "b8", "Q"))
    assert info["advanced"] and info["move"] == "a7b8q"


def test_place_out_of_context_refused():
    ref = _ref()
    ok, why = ref.check(("place", "e4", "Q"))
    assert not ok and "升变" in why


# ---------- 物理失败修复 ----------

def test_place_offset_repair_target_must_match():
    ref = _ref()
    assert ref.check(("move", "e2", "e4"))[0]
    ref.note_failure(("move", "e2", "e4"), "place_offset", "d4")   # 物理放偏到 d4
    ok, _ = ref.check(("move", "d4", "e4"))            # 从实际位置修，目标一致 → 放行
    assert ok
    ok, _ = ref.check(("move", "d4", "d5"))            # 目标不一致 → 拒
    assert not ok
    ok, why = ref.check(("move", "g1", "e4"))          # 目标一致但**不是那颗子** → 拒并指路
    assert not ok and "同一颗子" in why and "d4" in why
    info = ref.commit(("move", "d4", "e4"))            # 修复成功按原手记账
    assert info["advanced"] and info["move"] == "e2e4"
    assert ref.status()["physical_fails"] == {"place_offset": 1}


def test_repair_cannot_substitute_another_piece():
    """反面教材回归（2026-07-06 对局 2 实锤）：车夹不起来后大脑拿王顶替去目标格，
    旧规则（move 源格全自由）照单全收 → 真值/物理大面积分叉。收紧后必须拒。"""
    ref = _ref("r4rk1/ppp2ppp/8/8/8/8/PP3PPP/R4RK1 w - - 0 18")
    assert ref.check(("move", "f1", "e1"))[0]
    ref.note_failure(("move", "f1", "e1"), "grip_miss", "f1")      # 车还在 f1
    ok, why = ref.check(("move", "g1", "e1"))          # 拿王顶替 → 拒（王 g1→e1 本身也非法）
    assert not ok
    ok, _ = ref.check(("move", "f1", "e1"))            # 同一颗子原样重试 → 放行
    assert ok


def test_grip_miss_exact_retry():
    ref = _ref()
    ref.note_failure(("move", "e2", "e4"), "grip_miss", "e2")
    info = _do(ref, ("move", "e2", "e4"))
    assert info["advanced"]


def test_remove_repair_needs_reported_square():
    ref = _ref("rnbqkbnr/ppp1pppp/8/3p4/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2")
    assert ref.check(("remove", "d5"))[0]
    ref.note_failure(("remove", "d5"), "drop", "c4")   # 搬运半路掉在 c4
    assert ref.check(("remove", "c4"))[0]              # 报了实际格 → 只认这格
    assert not ref.check(("remove", "b3"))[0]
    _do(ref, ("remove", "c4"))                          # 修复成功按 remove(d5) 记账
    info = _do(ref, ("move", "e4", "d5"))
    assert info["advanced"] and info["move"] == "e4d5"


def test_abandoning_repair_by_other_legal_move_clears_attempt():
    ref = _ref()
    ref.note_failure(("move", "e2", "e4"), "grip_miss", "e2")
    _do(ref, ("move", "g1", "f3"))                     # 改主意走马（合法，pending 未开）
    assert not ref.check(("move", "d4", "e4"))[0]      # attempt 已清，修复通道关闭


# ---------- 终局 / 落档 ----------

def test_fools_mate_terminal_and_game_record(tmp_path):
    ref = gz_referee.Referee(START, white="anima", black="bot", games_dir=str(tmp_path))
    for prim in [("move", "f2", "f3"), ("move", "e7", "e5"),
                 ("move", "g2", "g4")]:
        _do(ref, prim)
    info = _do(ref, ("move", "d8", "h4"))              # 黑后杀
    assert info["over"] and info["result"] == "black" and "对局结束" in info["message"]
    ok, why = ref.check(("move", "e2", "e4"))
    assert not ok and "已结束" in why
    files = list(tmp_path.glob("games-*.jsonl"))
    assert len(files) == 1
    rec = json.loads(files[0].read_text().strip())
    assert rec["world"] == "gazebo-chess" and rec["game"] == "chess"
    assert rec["white"] == "anima" and rec["black"] == "bot" and rec["bot_side"] == "black"
    assert rec["result"] == "black" and rec["plies"] == 4
    assert rec["moves"] == ["f2f3", "e7e5", "g2g4", "d8h4"]
    assert rec["physical_fails"] == {}


def test_reset_logs_unfinished_and_starts_fresh(tmp_path):
    ref = gz_referee.Referee(START, games_dir=str(tmp_path))
    _do(ref, ("move", "e2", "e4"))
    ref.reset(START)
    files = list(tmp_path.glob("games-*.jsonl"))
    rec = json.loads(files[0].read_text().strip())
    assert rec["result"] == "" and rec["moves"] == ["e2e4"]   # 残局如实落档，不编结果
    assert ref.status()["moves"] == 0 and not ref.over


def test_push_direct_for_bot():
    ref = _ref()
    _do(ref, ("move", "e2", "e4"))
    info = ref.push_direct(chess.Move.from_uci("e7e5"))        # 对手瞬移应手
    assert info["advanced"] and ref.status()["turn"] == "white"
    with pytest.raises(AssertionError):
        _do(ref, ("remove", "d5"))                             # 没这种吃法，check 先拒


def test_restoration_ok_matches_truth_only():
    """备用子恢复判定：真值该格正是这枚子才放行（物理是否为空由世界核实，不在裁判职责内）。"""
    ref = _ref()
    assert ref.restoration_ok("a2", "P")           # 真值 a2=白兵
    assert not ref.restoration_ok("a2", "p")       # 颜色不对
    assert not ref.restoration_ok("a3", "P")       # 空格
    assert not ref.restoration_ok("e8", "K")       # e8 是黑王不是白王
    assert not ref.restoration_ok("zz", "P")       # 非法格名不炸
    # 恢复不影响进行中的修复上下文/序列（纯查询）
    ref.note_failure(("move", "a2", "a4"), "drop", None)
    assert ref.restoration_ok("a2", "P")
    assert ref.check(("move", "a2", "a4"))[0], "恢复后原来那手棋照旧可走"

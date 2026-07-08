# ⚠️ 角色：gazebo-chess 世界自带的「内置电脑对手」引擎——世界专用副本（第三份，v0.7）。
# 另两份独立副本：services/boardgame_engine/chess_engine.py（大脑的棋理顾问）、
# world/sim-chess/chess_bot.py（sim-chess 世界的对手）。三份【禁止去重合并】：
# 各是各的角色，零共享代码是有意为之的边界——关掉引擎服务(:8108)或 sim-chess，本世界对手照走；
# 日后可各自独立调强弱。本副本手抄自 chess_bot.py（算法未改）；它走子靠瞬移（world.py 执行），不用机械臂。
"""
国际象棋核心引擎 —— 纯搜索算法（无神经网络）。

算法概要：
  * 静态评估 (evaluation)：子力价值 + 子力位置表 (piece-square tables)。
  * Alpha-Beta 负极大搜索 (negamax + alpha-beta pruning)。
  * 迭代加深 + 时间上限 (iterative deepening)。
  * 静止搜索 (quiescence search)：只在吃子序列稳定处才停手估分，避免“地平线效应”。
  * 移动排序 (MVV-LVA + 历史最优着) 让剪枝更狠。

规则部分全部交给 python-chess 库处理（合法走子、将军/将死、和棋、
吃过路兵、王车易位、升变……），本文件只负责“怎么挑一步好棋”。
"""

import time
import chess

# ---------- 子力价值（厘兵 centipawn，1 兵 = 100） ----------
PIECE_VALUE = {
    chess.PAWN:   100,
    chess.KNIGHT: 320,
    chess.BISHOP: 330,
    chess.ROOK:   500,
    chess.QUEEN:  900,
    chess.KING:   0,        # 王不计入子力（用将死判断），但有位置表
}

MATE = 1_000_000           # 将死分值
MATE_THRESHOLD = MATE - 1000

# ---------- 子力位置表（从白方视角，a1 在左下；索引 0..63 = a1..h8） ----------
# 数值表示“某种子站在某格的额外加成（厘兵）”。黑方读取时上下镜像。
PAWN_PST = [
     0,  0,  0,  0,  0,  0,  0,  0,
     5, 10, 10,-20,-20, 10, 10,  5,
     5, -5,-10,  0,  0,-10, -5,  5,
     0,  0,  0, 20, 20,  0,  0,  0,
     5,  5, 10, 25, 25, 10,  5,  5,
    10, 10, 20, 30, 30, 20, 10, 10,
    50, 50, 50, 50, 50, 50, 50, 50,
     0,  0,  0,  0,  0,  0,  0,  0,
]
KNIGHT_PST = [
   -50,-40,-30,-30,-30,-30,-40,-50,
   -40,-20,  0,  5,  5,  0,-20,-40,
   -30,  5, 10, 15, 15, 10,  5,-30,
   -30,  0, 15, 20, 20, 15,  0,-30,
   -30,  5, 15, 20, 20, 15,  5,-30,
   -30,  0, 10, 15, 15, 10,  0,-30,
   -40,-20,  0,  0,  0,  0,-20,-40,
   -50,-40,-30,-30,-30,-30,-40,-50,
]
BISHOP_PST = [
   -20,-10,-10,-10,-10,-10,-10,-20,
   -10,  5,  0,  0,  0,  0,  5,-10,
   -10, 10, 10, 10, 10, 10, 10,-10,
   -10,  0, 10, 10, 10, 10,  0,-10,
   -10,  5,  5, 10, 10,  5,  5,-10,
   -10,  0,  5, 10, 10,  5,  0,-10,
   -10,  0,  0,  0,  0,  0,  0,-10,
   -20,-10,-10,-10,-10,-10,-10,-20,
]
ROOK_PST = [
     0,  0,  0,  5,  5,  0,  0,  0,
    -5,  0,  0,  0,  0,  0,  0, -5,
    -5,  0,  0,  0,  0,  0,  0, -5,
    -5,  0,  0,  0,  0,  0,  0, -5,
    -5,  0,  0,  0,  0,  0,  0, -5,
    -5,  0,  0,  0,  0,  0,  0, -5,
     5, 10, 10, 10, 10, 10, 10,  5,
     0,  0,  0,  0,  0,  0,  0,  0,
]
QUEEN_PST = [
   -20,-10,-10, -5, -5,-10,-10,-20,
   -10,  0,  5,  0,  0,  0,  0,-10,
   -10,  5,  5,  5,  5,  5,  0,-10,
     0,  0,  5,  5,  5,  5,  0, -5,
    -5,  0,  5,  5,  5,  5,  0, -5,
   -10,  0,  5,  5,  5,  5,  0,-10,
   -10,  0,  0,  0,  0,  0,  0,-10,
   -20,-10,-10, -5, -5,-10,-10,-20,
]
# 王在中残局更想往中心走（开局这张表鼓励缩在底线后易位）
KING_PST = [
    20, 30, 10,  0,  0, 10, 30, 20,
    20, 20,  0,  0,  0,  0, 20, 20,
   -10,-20,-20,-20,-20,-20,-20,-10,
   -20,-30,-30,-40,-40,-30,-30,-20,
   -30,-40,-40,-50,-50,-40,-40,-30,
   -30,-40,-40,-50,-50,-40,-40,-30,
   -30,-40,-40,-50,-50,-40,-40,-30,
   -30,-40,-40,-50,-50,-40,-40,-30,
]
PST = {
    chess.PAWN: PAWN_PST,
    chess.KNIGHT: KNIGHT_PST,
    chess.BISHOP: BISHOP_PST,
    chess.ROOK: ROOK_PST,
    chess.QUEEN: QUEEN_PST,
    chess.KING: KING_PST,
}


def evaluate(board):
    """静态评估，返回“以当前轮到方的视角”的分数（正=对我有利，厘兵）。"""
    if board.is_checkmate():
        # 轮到我但已被将死 -> 极差
        return -MATE
    if board.is_stalemate() or board.is_insufficient_material() \
            or board.is_seventyfive_moves() or board.is_fivefold_repetition():
        return 0

    score = 0  # 先从白方视角累加
    for square, piece in board.piece_map().items():
        val = PIECE_VALUE[piece.piece_type]
        pst = PST[piece.piece_type]
        if piece.color == chess.WHITE:
            score += val + pst[square]
        else:
            # 黑方：子力取负，位置表上下镜像 (square ^ 56)
            score -= val + pst[square ^ 56]

    return score if board.turn == chess.WHITE else -score


# ---------- 移动排序 ----------
def _move_score(board, move, tt_move):
    """给一个走法估“先试价值”，越大越先搜。"""
    if tt_move is not None and move == tt_move:
        return 1_000_000
    s = 0
    if board.is_capture(move):
        victim = board.piece_at(move.to_square)
        attacker = board.piece_at(move.from_square)
        # MVV-LVA：优先“用小子吃大子”
        v = PIECE_VALUE[victim.piece_type] if victim else 100   # 吃过路兵 victim 为 None
        a = PIECE_VALUE[attacker.piece_type] if attacker else 0
        s += 10_000 + v * 10 - a
    if move.promotion:
        s += 9_000 + PIECE_VALUE.get(move.promotion, 0)
    if board.gives_check(move):
        s += 50
    return s


def _ordered_moves(board, tt_move=None):
    moves = list(board.legal_moves)
    moves.sort(key=lambda m: _move_score(board, m, tt_move), reverse=True)
    return moves


class AI:
    def __init__(self, depth=3, time_limit=3.0):
        self.depth = depth              # 最大搜索层数
        self.time_limit = time_limit    # 单步思考秒数上限
        self._deadline = 0.0
        self.nodes = 0

    # ---------- 静止搜索：只继续搜“吃子/升变”，让局面安静下来再估分 ----------
    def _quiesce(self, board, alpha, beta):
        self.nodes += 1
        if time.time() > self._deadline:
            raise TimeoutError

        stand_pat = evaluate(board)
        if abs(stand_pat) >= MATE_THRESHOLD:     # 已成杀棋，直接返回
            return stand_pat
        if stand_pat >= beta:
            return beta
        if stand_pat > alpha:
            alpha = stand_pat

        # 被将军时必须搜所有逃法，否则会漏算被将死
        if board.is_check():
            moves = _ordered_moves(board)
            if not moves:
                return -MATE
        else:
            moves = [m for m in _ordered_moves(board)
                     if board.is_capture(m) or m.promotion]

        for move in moves:
            board.push(move)
            score = -self._quiesce(board, -beta, -alpha)
            board.pop()
            if score >= beta:
                return beta
            if score > alpha:
                alpha = score
        return alpha

    # ---------- 主搜索：negamax + alpha-beta ----------
    def _negamax(self, board, depth, alpha, beta, ply):
        self.nodes += 1
        if time.time() > self._deadline:
            raise TimeoutError

        if board.is_checkmate():
            return -MATE + ply           # 越早被将死越糟（ply 越小越糟）
        if board.is_stalemate() or board.is_insufficient_material() \
                or board.is_seventyfive_moves() or board.is_fivefold_repetition():
            return 0

        if depth == 0:
            return self._quiesce(board, alpha, beta)

        best = -MATE * 2
        for move in _ordered_moves(board):
            board.push(move)
            val = -self._negamax(board, depth - 1, -beta, -alpha, ply + 1)
            board.pop()
            if val > best:
                best = val
            if val > alpha:
                alpha = val
            if alpha >= beta:
                break                    # alpha-beta 剪枝
        return best

    def best_move(self, board):
        """返回引擎认为的最佳走法 chess.Move；无棋可走返回 None。"""
        legal = list(board.legal_moves)
        if not legal:
            return None
        if len(legal) == 1:
            return legal[0]

        self._deadline = time.time() + self.time_limit
        self.nodes = 0
        best_move = legal[0]

        # 迭代加深：从浅到深，随时超时随时停，用上一轮结果
        for d in range(1, self.depth + 1):
            alpha, beta = -MATE * 2, MATE * 2
            current_best = None
            best_val = -MATE * 2
            try:
                for move in _ordered_moves(board, tt_move=best_move):
                    board.push(move)
                    val = -self._negamax(board, d - 1, -beta, -alpha, 1)
                    board.pop()
                    if val > best_val:
                        best_val = val
                        current_best = move
                    if val > alpha:
                        alpha = val
            except TimeoutError:
                break
            if current_best is not None:
                best_move = current_best
                if best_val >= MATE_THRESHOLD:    # 已找到必杀，不必再深挖
                    break
        return best_move

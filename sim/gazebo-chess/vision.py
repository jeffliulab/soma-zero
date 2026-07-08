"""认棋盘：从俯视相机帧里读出每格有没有子、是黑是白，输出和 sim-chess 同格式的"摆放表"。

做成可替换的「眼」：两块独立——
  1) 标定：world 棋盘平面坐标 → 像素 的单应矩阵 H（仿真里相机/棋盘位姿已知，但我们仍走"放几个已知点、
     检测像素、拟合 H"这条真实路径，结构和真机一致；真机换真实标定不动上层）。
  2) 读盘：对每格中心用 H 投到像素、采样小块、按颜色判 空/白/黑。

v0.4：棋子白/黑、棋盘绿，靠亮度区分够用。完整鲁棒识别（多子、遮挡）留 0.5。
"""
from __future__ import annotations

import numpy as np

import config
import geometry


def board_region_mask(frame: np.ndarray) -> np.ndarray:
    """棋盘区域(填实的绿色板)掩码——子是板上非绿的"洞"，板外的地面/机械臂不算。
    做法：绿掩码 → 取最大连通绿块 → 用凸包/填洞补上被子占掉的洞，得到整块板。"""
    import cv2
    rgb = frame[:, :, :3]
    g, r, b = rgb[:, :, 1].astype(int), rgb[:, :, 0].astype(int), rgb[:, :, 2].astype(int)
    green = ((g > r + 8) & (g > b + 8)).astype(np.uint8)
    green = cv2.morphologyEx(green, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    cnts, _ = cv2.findContours(green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    region = np.zeros(green.shape, np.uint8)
    if cnts:
        big = max(cnts, key=cv2.contourArea)
        cv2.drawContours(region, [cv2.convexHull(big)], -1, 1, -1)   # 填实凸包=整块板
    return region


def detect_blobs(frame: np.ndarray) -> list[tuple[int, int, str]]:
    """在画面里找棋子团块，返回 [(cx, cy, 'white'|'black'), ...]（像素）。
    只在棋盘区域内找（排除板外的浅灰地面、白机械臂）：板上亮团=白子，暗团=黑子。"""
    import cv2
    rgb = frame[:, :, :3]
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    g, r, b = rgb[:, :, 1].astype(int), rgb[:, :, 0].astype(int), rgb[:, :, 2].astype(int)
    greenish = (g > r + 12) & (g > b + 12)
    region = board_region_mask(frame).astype(bool)
    white = (gray > 170) & (~greenish) & region
    black = (gray < 75) & (~greenish) & region
    out: list[tuple[int, int, str]] = []
    for mask, color in ((white.astype(np.uint8), "white"), (black.astype(np.uint8), "black")):
        n, _, stats, cent = cv2.connectedComponentsWithStats(mask, 8)
        for i in range(1, n):
            if stats[i, cv2.CC_STAT_AREA] >= 60:       # 滤掉噪点
                out.append((int(cent[i][0]), int(cent[i][1]), color))
    return out


def calibrate_homography(world_xy: list[tuple[float, float]],
                         pixel_xy: list[tuple[float, float]]) -> np.ndarray:
    """由 ≥4 组 (world 棋盘平面 x,y) ↔ (像素 u,v) 拟合单应矩阵 H（world→pixel）。"""
    import cv2
    H, _ = cv2.findHomography(np.array(world_xy, np.float32), np.array(pixel_xy, np.float32))
    return H


def world_to_pixel(H: np.ndarray, x: float, y: float) -> tuple[int, int]:
    v = H @ np.array([x, y, 1.0])
    return int(v[0] / v[2]), int(v[1] / v[2])


def read_board(frame: np.ndarray, H: np.ndarray, patch: int = 14) -> dict[str, str]:
    """对每格中心用 H 投到像素、采样小块判 空/白/黑。返回 {square: 'w'|'b'}（空格不进表，对齐 sim-chess）。"""
    import cv2
    gray = cv2.cvtColor(frame[:, :, :3], cv2.COLOR_RGB2GRAY)
    rgb = frame[:, :, :3]
    h, w = gray.shape
    out: dict[str, str] = {}
    for f in range(config.BOARD_FILES):
        for r in range(config.BOARD_RANKS):
            sq = geometry.square_name(f, r)
            lx, ly = geometry.square_center_local(sq)
            u, v = world_to_pixel(H, lx, ly)
            if not (patch <= u < w - patch and patch <= v < h - patch):
                continue
            blk = rgb[v - patch:v + patch, u - patch:u + patch]
            gblk = gray[v - patch:v + patch, u - patch:u + patch]
            g, rr, bb = blk[:, :, 1].astype(int), blk[:, :, 0].astype(int), blk[:, :, 2].astype(int)
            greenish = np.mean((g > rr + 12) & (g > bb + 12))
            mean = float(np.mean(gblk))
            if greenish > 0.5:
                continue                                # 多是绿底=空
            if mean > 150:
                out[sq] = "w"
            elif mean < 80:
                out[sq] = "b"
    return out

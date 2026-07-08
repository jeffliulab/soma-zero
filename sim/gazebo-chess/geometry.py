"""gazebo-chess 几何换算：棋格名 ↔ 棋盘局部坐标 ↔ 机械臂 world(MoveIt规划帧) 坐标。

纯函数、无 ROS 依赖，可离线单测（见文件末 __main__ 自测）。所有尺寸/位姿从 config 取，禁写死。

坐标约定（都在注释里讲清，免得换人看不懂）：
- 格名：列 a-h（file，左到右）、行 1-8（rank）。a1 是 file=0,rank=0。
- 棋盘局部系：原点在棋盘**中心**，x 沿 file 增大方向（a→h），y 沿 rank 增大方向（1→8），z 朝上。
  某格中心局部坐标：x=(file-(N-1)/2)*cell, y=(rank-(N-1)/2)*cell（N=8，居中）。
- world(MoveIt规划帧) 系：把局部系绕 z 转 BOARD_YAW、再平移到 BOARD_ORIGIN。
- 棋盘上表面 z=BOARD_ORIGIN_Z；棋子"腰"（抓取高度）= 上表面 + PIECE_GRASP_WAIST_M。
"""
from __future__ import annotations

import math

import config


def parse_square(name: str) -> tuple[int, int]:
    """'e2' -> (file_idx=4, rank_idx=1)。非法格名抛 ValueError。"""
    s = name.strip().lower()
    if len(s) != 2 or s[0] not in "abcdefgh" or s[1] not in "12345678":
        raise ValueError(f"非法格名：{name!r}（应形如 e2）")
    return ord(s[0]) - ord("a"), int(s[1]) - 1


def square_name(file_idx: int, rank_idx: int) -> str:
    """(4,1) -> 'e2'。"""
    if not (0 <= file_idx < config.BOARD_FILES and 0 <= rank_idx < config.BOARD_RANKS):
        raise ValueError(f"格索引越界：file={file_idx}, rank={rank_idx}")
    return f"{chr(ord('a') + file_idx)}{rank_idx + 1}"


def square_center_local(name: str) -> tuple[float, float]:
    """格中心在棋盘局部系的 (x, y)（米），原点在棋盘中心。"""
    f, r = parse_square(name)
    cx = (f - (config.BOARD_FILES - 1) / 2.0) * config.CELL_M
    cy = (r - (config.BOARD_RANKS - 1) / 2.0) * config.CELL_M
    return cx, cy


def _local_to_base(x: float, y: float) -> tuple[float, float]:
    """棋盘局部 (x,y) → world(MoveIt规划帧) (x,y)：绕 z 转 yaw，再平移到棋盘原点。"""
    c, s = math.cos(config.BOARD_YAW_RAD), math.sin(config.BOARD_YAW_RAD)
    bx = config.BOARD_ORIGIN_X + x * c - y * s
    by = config.BOARD_ORIGIN_Y + x * s + y * c
    return bx, by


def square_surface_xyz(name: str) -> tuple[float, float, float]:
    """格中心、棋盘上表面那一点在 world(MoveIt规划帧) 的 (x,y,z)（米）。"""
    lx, ly = square_center_local(name)
    bx, by = _local_to_base(lx, ly)
    return bx, by, config.BOARD_ORIGIN_Z


def grasp_xyz(name: str) -> tuple[float, float, float]:
    """抓/放某格棋子时夹爪中心的目标点：格中心正上方、棋子腰高处。"""
    bx, by, bz = square_surface_xyz(name)
    return bx, by, bz + config.PIECE_GRASP_WAIST_M


def approach_xyz(name: str) -> tuple[float, float, float]:
    """某格上方的安全接近点（抓取点再抬高 APPROACH_SAFE_M）。"""
    bx, by, bz = grasp_xyz(name)
    return bx, by, bz + config.APPROACH_SAFE_M


def reservoir_spawn_xyz() -> tuple[float, float, float]:
    """备用子区里 spawn 一枚新子时的落点。⚠️ 备用区在棋盘外，支撑面是桌面（OFFBOARD_SURFACE_Z），
    不是棋盘面——按棋盘面高度生成会掉 28mm、夹爪随之抓空（2026-07-03 实测教训）。"""
    return config.RESERVOIR_ORIGIN_X, config.RESERVOIR_ORIGIN_Y, config.OFFBOARD_SURFACE_Z


def reservoir_grasp_xyz() -> tuple[float, float, float]:
    """从备用子区夹起一枚子时夹爪中心的目标点（备用子腰高处）。"""
    rx, ry, rz = reservoir_spawn_xyz()
    return rx, ry, rz + config.PIECE_GRASP_WAIST_M


def discard_slot_xyz(index: int) -> tuple[float, float, float]:
    """第 index 个弃子槽的落点（桌面高度，盘外）：沿 x 排开，满一排(DISCARD_SLOTS_PER_ROW)沿 y 外扩下一排。"""
    row = index // config.DISCARD_SLOTS_PER_ROW
    col = index % config.DISCARD_SLOTS_PER_ROW
    x = config.DISCARD_ORIGIN_X + col * config.DISCARD_PITCH_M
    y = config.DISCARD_ORIGIN_Y + row * config.DISCARD_PITCH_M
    return x, y, config.OFFBOARD_SURFACE_Z          # 弃子区同样在盘外=桌面高度


def discard_grasp_xyz(index: int) -> tuple[float, float, float]:
    """把子放进第 index 个弃子槽时夹爪中心的目标点（槽位腰高处）。"""
    dx, dy, dz = discard_slot_xyz(index)
    return dx, dy, dz + config.PIECE_GRASP_WAIST_M


def base_xy_to_square(bx: float, by: float) -> str | None:
    """world(MoveIt规划帧) (x,y) → 最近的棋格名；落在棋盘范围外返回 None。
    给 0.5 的位置评估/失败诊断用（一个子歪了/压线了，判它名义上属于哪格）。"""
    # 先逆变换回局部系
    c, s = math.cos(-config.BOARD_YAW_RAD), math.sin(-config.BOARD_YAW_RAD)
    dx, dy = bx - config.BOARD_ORIGIN_X, by - config.BOARD_ORIGIN_Y
    lx = dx * c - dy * s
    ly = dx * s + dy * c
    f = round(lx / config.CELL_M + (config.BOARD_FILES - 1) / 2.0)
    r = round(ly / config.CELL_M + (config.BOARD_RANKS - 1) / 2.0)
    if not (0 <= f < config.BOARD_FILES and 0 <= r < config.BOARD_RANKS):
        return None
    return square_name(int(f), int(r))


def recommended_cam_height() -> float:
    """按"拍全棋盘 + 四周留边"反算相机最低高度：半幅(含边距)/tan(半 FOV)。
    仅作参考/校验；实际用 config.CAM_HEIGHT_M（可被 env 覆盖）。"""
    half_span = config.BOARD_SIZE_M / 2.0 * 1.2          # 棋盘半幅 + 20% 边距
    return half_span / math.tan(config.CAM_FOV_RAD / 2.0)


if __name__ == "__main__":
    # 离线自测：往返一致 + 几个已知点的合理性。全过打印 OK，不需要 Gazebo。
    ok = True

    # 1) 格名 round-trip
    for nm in ("a1", "h8", "e2", "e4", "d4", "c6"):
        f, r = parse_square(nm)
        assert square_name(f, r) == nm, nm
    # 2) 局部坐标：中心四格应关于原点对称，a1 在负负角
    ax, ay = square_center_local("a1")
    hx, hy = square_center_local("h8")
    assert ax < 0 and ay < 0 and hx > 0 and hy > 0, (ax, ay, hx, hy)
    assert abs(ax + hx) < 1e-9 and abs(ay + hy) < 1e-9, "a1/h8 应关于棋盘中心对称"
    # 3) base 坐标 → 反查回同一格（默认 yaw=0）
    for nm in ("a1", "h8", "e2", "e4", "d5"):
        bx, by, _ = square_surface_xyz(nm)
        back = base_xy_to_square(bx, by)
        if back != nm:
            ok = False
            print(f"FAIL round-trip: {nm} -> base({bx:.3f},{by:.3f}) -> {back}")
    # 4) 抓取点比表面高一个腰高；接近点更高
    gx, gy, gz = grasp_xyz("e2")
    sx, sy, sz = square_surface_xyz("e2")
    apx, apy, apz = approach_xyz("e2")
    assert abs(gz - (sz + config.PIECE_GRASP_WAIST_M)) < 1e-9
    assert apz > gz > sz

    print("=== gazebo-chess geometry self-test ===")
    print(f"CELL_M = {config.CELL_M:.4f} m, BOARD_SIZE = {config.BOARD_SIZE_M} m")
    print(f"e2 surface (base) = {square_surface_xyz('e2')}")
    print(f"e2 grasp   (base) = {grasp_xyz('e2')}")
    print(f"e2 approach(base) = {approach_xyz('e2')}")
    print(f"e4 grasp   (base) = {grasp_xyz('e4')}")
    print(f"推荐相机最低高度 ≈ {recommended_cam_height():.3f} m（配置用 {config.CAM_HEIGHT_M} m）")
    # 可夹宽度区间提醒
    wmin = config.GRIP_FACE_GAP_CLOSED_M
    wmax = config.GRIP_FACE_GAP_CLOSED_M + 2 * config.GRIP_STROKE_M
    print(f"夹爪可夹宽度区间 ≈ [{wmin:.3f}, {wmax:.3f}] m；棋子抓取点宽 {config.PIECE_GRASP_WIDTH_M} m")
    assert wmin <= config.PIECE_GRASP_WIDTH_M <= wmax, "棋子抓取点宽度不在夹爪可夹区间内！"
    print("OK" if ok else "有 FAIL，见上")

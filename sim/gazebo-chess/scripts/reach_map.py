"""64 格可达性地图（W0 硬闸自检，正式工具；接替临时探针 _probe_reach.py 的角色）。

对每个棋格取抓取点（格中心、腰高），按 grasp_pose 的候选顺序逐个解 IK（接近位姿 + 抓取位姿
都要可达才算数），走与 arm_controller.compute_ik **同一条 IK+FK 复核链**（IKFast 假解防线）。
默认**只解不动臂**（快、无副作用）；`--execute` 抽查模式对指定格真抓真放一轮核对落点。

用法（前提：episode 仿真栈在跑，先 source ROS2）：
    python3 scripts/reach_map.py                     # 全 64 格地图；64/64 可达 exit 0，否则 exit 1
    python3 scripts/reach_map.py --json out.json     # 另存 JSON（默认打到 stdout 尾部）
    python3 scripts/reach_map.py --execute a1,h1,h8  # 对这些格真抓真放抽查（spawn 一枚子）

地图格子含义：每格显示「首个 IK 可达候选的倾角档」（0/15/30/45/60/75，·=0 竖直档，X=全候选不可达）。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import rclpy  # noqa: E402

import config  # noqa: E402
import geometry  # noqa: E402
import grasp_pose  # noqa: E402
from arm_controller import ArmController  # noqa: E402


def probe_square(arm: ArmController, square: str) -> tuple[str | None, str | None]:
    """解某格：返回 (首个可达候选的标签, None) 或 (None, 失败说明)。只解不动臂。"""
    gx, gy, gz = geometry.grasp_xyz(square)
    for label, approach, grasp in grasp_pose.candidates_for_point(gx, gy, gz):
        if arm.compute_ik(approach) is None:
            continue
        if arm.compute_ik(grasp) is None:
            continue
        return label, None
    return None, "所有候选姿态（含全部倾角档）IK+FK 复核均不可达"


def execute_square(arm: ArmController, square: str) -> tuple[bool, str]:
    """真抓真放抽查：在该格 spawn 一枚子 → pick → 原格 place → 静置 → 核对落点误差。"""
    import spawn  # 延迟 import：只在 --execute 时需要 gz service

    spawn.spawn_board()
    spawn.spawn_piece(square, "white")
    time.sleep(config.SPAWN_SETTLE_S + 1.0)
    p0 = spawn.model_pose("piece_" + square)
    if p0 is None:
        return False, "spawn 后读不到棋子位姿"
    ok, msg = arm.pick_at(p0[0], p0[1], p0[2] + config.PIECE_GRASP_WAIST_M,
                          progress=lambda m: print(f"    [{square}] {m}"))
    if not ok:
        return False, f"pick 失败: {msg}"
    dx, dy, dz = geometry.grasp_xyz(square)
    ok, msg = arm.place_at(dx, dy, dz, progress=lambda m: print(f"    [{square}] {m}"))
    if not ok:
        return False, f"place 失败: {msg}"
    time.sleep(config.SETTLE_S)
    p1 = spawn.model_pose("piece_" + square)
    exp = geometry.square_surface_xyz(square)
    if p1 is None:
        return False, "放后读不到棋子位姿（掉出场地？）"
    err = math.hypot(p1[0] - exp[0], p1[1] - exp[1])
    if err > config.PLACE_TOLERANCE_M:
        return False, f"落点误差 {err*100:.1f}cm 超容差 {config.PLACE_TOLERANCE_M*100:.1f}cm"
    return True, f"抓放到位（误差 {err*100:.1f}cm）"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--json", default="", help="把结果另存为 JSON 文件（默认只打 stdout）")
    ap.add_argument("--execute", default="", help="逗号分隔的格名清单：对这些格真抓真放抽查")
    args = ap.parse_args()

    rclpy.init()
    arm = ArmController()
    # 专职 spin 线程（与 world.py 同模式）：脚本独立跑时必须自己 spin，否则 IK future 永不完成。
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(arm)
    spin_t = threading.Thread(target=executor.spin, daemon=True)
    spin_t.start()
    if not arm.wait_ready(config.READY_TIMEOUT_S):
        print("FAIL: 连不上 MoveIt/控制器（episode 仿真栈起了吗？）")
        return 1

    result: dict[str, dict] = {}
    t0 = time.time()
    for rank in range(config.BOARD_RANKS):
        for file in range(config.BOARD_FILES):
            sq = geometry.square_name(file, rank)
            label, fail = probe_square(arm, sq)
            gx, gy, _ = geometry.grasp_xyz(sq)
            result[sq] = {
                "reachable": label is not None,
                "candidate": label,
                "radius_m": round(math.hypot(gx - config.ARM_BASE_XY[0], gy - config.ARM_BASE_XY[1]), 4),
                **({"fail": fail} if fail else {}),
            }
            print(f"{sq}: {label or 'X 不可达'}  (r={result[sq]['radius_m']:.3f}m)")

    # 8×8 地图（rank8 在上，a 列在左；·=竖直档，数字=倾角，X=不可达）
    print(f"\n=== 可达性地图（{time.time()-t0:.0f}s；格值 = 首个可达候选的倾角档）===")
    print(f"棋盘中心 x={config.BOARD_ORIGIN_X:.3f}  格宽={config.CELL_M}  间隙={ARM_GAP_DESC}")
    for rank in range(config.BOARD_RANKS - 1, -1, -1):
        row = []
        for file in range(config.BOARD_FILES):
            sq = geometry.square_name(file, rank)
            r = result[sq]
            if not r["reachable"]:
                row.append("  X")
            else:
                tilt = r["candidate"].split("_")[0].removeprefix("tilt")
                row.append("  ·" if tilt == "0" else f"{int(tilt):3d}")
            # 每格 3 字符宽
        print(f"  {rank+1} |" + "".join(row))
    print("      " + "".join(f"  {c}" for c in "abcdefgh"))

    unreachable = [sq for sq, r in result.items() if not r["reachable"]]
    n_ok = 64 - len(unreachable)
    print(f"\n可达 {n_ok}/64" + (f"；不可达：{','.join(unreachable)}" if unreachable else "（全格可达 ✅）"))

    if args.json:
        with open(args.json, "w") as f:
            json.dump({"squares": result, "board_origin_x": config.BOARD_ORIGIN_X,
                       "cell_m": config.CELL_M, "tilts_deg": config.APPROACH_TILT_DEG}, f,
                      ensure_ascii=False, indent=1)
        print(f"JSON 已存 {args.json}")

    exec_fail = False
    if args.execute:
        print("\n=== 实抓抽查 ===")
        for sq in [s.strip() for s in args.execute.split(",") if s.strip()]:
            ok, msg = execute_square(arm, sq)
            print(f"{sq}: {'PASS' if ok else 'FAIL'} —— {msg}")
            exec_fail = exec_fail or not ok

    # 有序关停：先停 executor（等 spin 线程退出）再销节点，避免 daemon 线程在解释器收尾时崩溃。
    executor.shutdown()
    spin_t.join(timeout=3.0)
    arm.destroy_node()
    rclpy.shutdown()
    return 0 if (n_ok == 64 and not exec_fail) else 1


ARM_GAP_DESC = (f"轴心→板边 {config.ARM_BOARD_GAP_M}m"
                f"(外缘净空 {config.ARM_BOARD_GAP_M - config.ARM_BASE_FRONT_X_M:.3f}m)")

if __name__ == "__main__":
    sys.exit(main())

"""W2 自测：机械臂把一个子从 src 格真夹起、搬到 dst 格放下，用棋子真实位姿核对到位。
全过 exit 0、失败 exit 1。前提：episode 仿真栈在跑（headless 即可）。
用法：source ROS 后  python3 scripts/test_pick_place.py [src=e2] [dst=e4]"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import rclpy  # noqa: E402

import config  # noqa: E402
import geometry  # noqa: E402
import spawn  # noqa: E402
from arm_controller import ArmController  # noqa: E402

SRC = sys.argv[1] if len(sys.argv) > 1 else "e2"
DST = sys.argv[2] if len(sys.argv) > 2 else "e4"


def main() -> int:
    rclpy.init()
    arm = ArmController()
    if not arm.wait_ready(20):
        print("FAIL: 连不上 MoveIt/控制器（仿真起了吗？）")
        return 1
    # 干净起一枚子在 src
    spawn.spawn_board()
    spawn.spawn_piece(SRC, "white")
    time.sleep(1.5)
    p0 = spawn.model_pose("piece_" + SRC)
    if p0 is None:
        print("FAIL: 没 spawn 出棋子")
        return 1
    gx, gy, gz = p0[0], p0[1], p0[2] + config.PIECE_GRASP_WAIST_M
    print(f"起始 {SRC}={tuple(round(v,3) for v in p0)}")

    ok, msg = arm.pick_at(gx, gy, gz)
    print("pick:", ok, msg)
    if not ok:
        return 1
    # 放到 dst 格中心（同样腰高）
    dx, dy, dz = geometry.grasp_xyz(DST)
    ok, msg = arm.place_at(dx, dy, dz)
    print("place:", ok, msg)
    if not ok:
        return 1
    time.sleep(1.5)

    p1 = spawn.model_pose("piece_" + SRC)
    exp = geometry.square_surface_xyz(DST)
    err = math.hypot(p1[0] - exp[0], p1[1] - exp[1]) if p1 else 9.99
    print(f"终态 piece={tuple(round(v,3) for v in p1) if p1 else None}  目标 {DST} 中心={tuple(round(v,3) for v in exp)}")
    print(f"落点水平误差={err*100:.1f} cm  (容差 {config.PLACE_TOLERANCE_M*100:.1f} cm)")
    arm.destroy_node()
    rclpy.shutdown()
    if err <= config.PLACE_TOLERANCE_M:
        print(f"PASS: {SRC}->{DST} 抓放到位")
        return 0
    print(f"FAIL: 落点偏差超容差")
    return 1


if __name__ == "__main__":
    sys.exit(main())

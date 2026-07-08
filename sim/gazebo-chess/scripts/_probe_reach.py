"""临时探针：扫几个高度的"竖直朝下"位姿，执行后读实际 link6 位姿，看臂到底能朝下够到多低。
回答关键问题：棋盘平铺在桌面(臂基座同高)时，小臂能不能朝下够到板面附近的棋子。调好删。"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import rclpy  # noqa: E402
import grasp_pose  # noqa: E402
import spawn  # noqa: E402
from arm_controller import ArmController, ARM_JOINTS  # noqa: E402

rclpy.init()
arm = ArmController()
arm.wait_ready(15)
px, py = 0.28, 0.0
down = grasp_pose.quat_from_rpy(3.14159, 0, 0)
for z in (0.35, 0.30, 0.25, 0.20, 0.15, 0.12):
    pose = ((px, py, z), down)
    j = arm.compute_ik(pose)
    if j is None:
        print(f"z={z:.2f} 朝下: IK 无解")
        continue
    arm.goto_arm(j, 3.0)
    time.sleep(1.0)
    P = spawn.all_model_poses()
    l6 = P.get("link6")
    lf = P.get("left_finger_link")
    ok = l6 and abs(l6[0] - px) < 0.03 and abs(l6[1] - py) < 0.03 and abs(l6[2] - z) < 0.03
    print(f"z={z:.2f} 朝下: IK有解, 实际 link6={tuple(round(v,3) for v in l6) if l6 else None}, "
          f"指尖z={round(lf[2],3) if lf else None}, 匹配请求={ok}")
arm.destroy_node()
rclpy.shutdown()

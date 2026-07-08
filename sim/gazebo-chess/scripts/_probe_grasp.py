"""临时标定探针：把臂移到 e2 抓取位姿，量一下指尖实际到哪、和棋子差多少，用来校准 TCP_OFFSET。
（不是正式自测，调好后会删/并入正式脚本。）用法：source ROS 后 python3 scripts/_probe_grasp.py"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rclpy  # noqa: E402

import config  # noqa: E402
import grasp_pose  # noqa: E402
import spawn  # noqa: E402
from arm_controller import ArmController  # noqa: E402


def main():
    rclpy.init()
    arm = ArmController()
    arm.wait_ready(15)
    if spawn.model_pose("piece_e2") is None:
        spawn.spawn_piece("e2", "white")
        time.sleep(1.5)
    p = spawn.model_pose("piece_e2")
    gx, gy, gz = p[0], p[1], p[2] + config.PIECE_GRASP_WAIST_M
    _, _, grp = grasp_pose.candidates_for_point(gx, gy, gz)[0]
    arm.open_gripper()
    jg = arm.compute_ik(grp)
    if jg is None:
        print("IK 解不出 grasp 位姿")
        return
    print("IK 解 jg =", [round(v, 3) for v in jg])
    moved = arm.goto_arm(jg, 4.0)
    print("goto_arm 返回:", moved)
    time.sleep(1.5)
    from arm_controller import ARM_JOINTS
    cur = arm.current_arm_positions()
    print("执行后实际关节:", [round(cur.get(j, 0), 3) for j in ARM_JOINTS])
    P = spawn.all_model_poses()
    print("piece_e2 :", tuple(round(v, 3) for v in P.get("piece_e2", (0, 0, 0))))
    for n in ("link6", "left_finger_link", "right_finger_link", "gripper_base_link"):
        if n in P:
            print(f"{n:20s}:", tuple(round(v, 3) for v in P[n]))
    print("目标抓取点 z =", round(gz, 3), " 当前 TCP_OFFSET =", config.TCP_OFFSET_M)
    print("指尖底 ≈ finger_link.z - finger_z/2(0.0225)；想让它到棋子腰 ≈", round(p[2] + 0.018, 3))
    arm.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

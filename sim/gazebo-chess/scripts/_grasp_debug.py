"""临时：把臂带到 e2 抓取位姿，抓帧 + 量指尖/棋子位姿，诊断为什么夹不起来。调好删。"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402

import config  # noqa: E402
import grasp_pose  # noqa: E402
import spawn  # noqa: E402
from arm_controller import ArmController  # noqa: E402

SP = "/tmp/claude-1000/-home-jeff-2026-summer-career-projects/5eb7d587-7fe5-4cbf-8ac3-b0481b715371/scratchpad"


def main():
    rclpy.init()
    arm = ArmController()
    arm.wait_ready(15)
    got = {}
    arm.create_subscription(Image, "/gazebo_chess/overhead/image",
                            lambda m: got.__setitem__("img", np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, -1)), 10)

    def grab(fn):
        got.pop("img", None)
        t0 = time.time()
        while "img" not in got and time.time() - t0 < 6:
            rclpy.spin_once(arm, timeout_sec=0.2)
        if "img" in got:
            import cv2
            cv2.imwrite(fn, cv2.cvtColor(got["img"][:, :, :3], cv2.COLOR_RGB2BGR))

    spawn.spawn_piece("e2", "white")
    time.sleep(1.5)
    p = spawn.model_pose("piece_e2")
    gx, gy, gz = p[0], p[1], p[2] + config.PIECE_GRASP_WAIST_M
    print(f"piece_e2={tuple(round(v,3) for v in p)}  抓取点={round(gx,3)},{round(gy,3)},{round(gz,3)}")
    label, approach, grasp = grasp_pose.candidates_for_point(gx, gy, gz)[0]
    arm.open_gripper()
    ja = arm.compute_ik(approach)
    jg = arm.compute_ik(grasp)
    print("IK approach:", ja is not None, " grasp:", jg is not None)
    if jg is None:
        return
    arm.goto_arm(ja, 3.0)
    arm.goto_arm(jg, 2.0)
    time.sleep(1.0)
    grab(f"{SP}/grasp_atpose.png")
    P = spawn.all_model_poses()
    l6 = arm._fk_link6(jg)
    print("FK link6 =", tuple(round(v, 3) for v in l6) if l6 else None, " (请求", tuple(round(v, 3) for v in grasp[0]), ")")
    for n in ("piece_e2", "left_finger_link", "right_finger_link"):
        if n in P:
            print(f"  {n:18s}=", tuple(round(v, 3) for v in P[n]))
    # 闭爪 + 抬起，看子有没有跟上来
    arm.close_gripper()
    time.sleep(0.5)
    arm.goto_arm(ja, 2.0)
    time.sleep(1.0)
    p2 = spawn.model_pose("piece_e2")
    print("抬起后 piece_e2 =", tuple(round(v, 3) for v in p2) if p2 else None,
          " z抬升=", round(p2[2] - p[2], 3) if p2 else None)
    grab(f"{SP}/grasp_lifted.png")
    arm.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

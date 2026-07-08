"""临时：把臂停到一边、布好场景(板+子+相机)、抓一帧存盘，用来调相机视角。
用法：source ROS 后 python3 scripts/_tune_camera.py [out.png]"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402

import spawn  # noqa: E402
from arm_controller import ArmController  # noqa: E402

OUT = sys.argv[1] if len(sys.argv) > 1 else "/tmp/cam_view.png"
PARK = [2.5, 0.0, 0.0, 0.0, 0.0, 0.0]   # 把臂甩到后侧，让出俯视相机视野


def main():
    rclpy.init()
    arm = ArmController()
    arm.wait_ready(15)
    # 1) 停臂到一边
    arm.goto_arm(PARK, 3.0)
    # 2) 布场景（spawn 现在幂等=先删同名再建，所以每次都用最新 config 重建）
    spawn.spawn_board()
    spawn.spawn_piece("e2", "white")
    spawn.spawn_camera()
    time.sleep(2.0)
    # 3) 抓一帧
    got = {}

    def cb(m):
        if "img" not in got:
            got["img"] = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, -1)
    arm.create_subscription(Image, "/gazebo_chess/overhead/image", cb, 10)
    t0 = time.time()
    while "img" not in got and time.time() - t0 < 8:
        rclpy.spin_once(arm, timeout_sec=0.2)
    if "img" in got:
        import cv2
        cv2.imwrite(OUT, cv2.cvtColor(got["img"][:, :, :3], cv2.COLOR_RGB2BGR))
        print("SAVED", OUT, got["img"].shape)
    else:
        print("NO IMG")
    arm.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

"""W1 自测：标定 world↔像素单应 + 读盘。把白子逐个放到 4 角格→检测像素→拟合 H；
再摆几枚到已知格→read_board→核对认出来的格和真值一致。全过 exit 0、失败 exit 1。
前提：仿真 + image_bridge 在跑。会把标定出的 H 存到 calib_homography.npy 供世界复用。"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402

import geometry  # noqa: E402
import spawn  # noqa: E402
import vision  # noqa: E402
from arm_controller import ArmController  # noqa: E402

PARK = [2.5, 0, 0, 0, 0, 0]
CALIB_SQUARES = ["a1", "a8", "h1", "h8"]
TEST = {"e4": "w", "c5": "w", "f3": "b"}
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main() -> int:
    rclpy.init()
    arm = ArmController()
    arm.wait_ready(15)
    frame = {}
    arm.create_subscription(Image, "/gazebo_chess/overhead/image",
                            lambda m: frame.__setitem__("f", np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, -1)), 10)

    def grab():
        frame.pop("f", None)
        t0 = time.time()
        while "f" not in frame and time.time() - t0 < 6:
            rclpy.spin_once(arm, timeout_sec=0.2)
        return frame.get("f")

    spawn.spawn_board()
    spawn.spawn_camera()
    arm.goto_arm(PARK, 3.0)
    # 逐角标定：一次只放一颗，检测唯一团块的像素
    world_xy, pixel_xy = [], []
    for sq in CALIB_SQUARES:
        for nm in list(spawn.all_model_poses()):
            if nm.startswith("piece_"):
                spawn.remove_model(nm)
        time.sleep(0.5)
        spawn.spawn_piece(sq, "white", name="calib")
        time.sleep(1.2)
        blobs = vision.detect_blobs(grab())
        whites = [b for b in blobs if b[2] == "white"]
        if len(whites) != 1:
            print(f"FAIL: 标定 {sq} 检到 {len(whites)} 个白团（应为1）")
            return 1
        lx, ly = geometry.square_center_local(sq)
        world_xy.append((lx, ly))
        pixel_xy.append((whites[0][0], whites[0][1]))
        print(f"标定 {sq}: world({lx:.3f},{ly:.3f}) -> px{whites[0][:2]}")
    H = vision.calibrate_homography(world_xy, pixel_xy)
    np.save(os.path.join(HERE, "calib_homography.npy"), H)

    # 读盘测试：摆几枚到已知格
    for nm in list(spawn.all_model_poses()):
        if nm.startswith("piece_") or nm == "calib":
            spawn.remove_model(nm)
    time.sleep(0.5)
    for sq, c in TEST.items():
        spawn.spawn_piece(sq, "white" if c == "w" else "black", name=f"t_{sq}")
    time.sleep(1.5)
    placement = vision.read_board(grab(), H)
    print("read_board ->", placement)
    arm.destroy_node()
    rclpy.shutdown()

    ok = True
    for sq, c in TEST.items():
        if placement.get(sq) != c:
            print(f"  FAIL: {sq} 应={c}, 读到={placement.get(sq)}")
            ok = False
    extra = set(placement) - set(TEST)
    if extra:
        print(f"  注意：多读出 {extra}（可能噪点/边界）")
    print("PASS: 标定+读盘认出所有测试子" if ok else "FAIL: 读盘有错")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

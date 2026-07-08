"""合成数据管线：随机摆子 → 斜视相机抓帧 → 世界真值自动打标签，产出 CNN 训练数据。

用法（Gazebo 仿真栈 + image_bridge 在跑；本世界服务【不要】同时跑——两边都会 spawn/remove 抢实体）：
    cd world/gazebo-chess && source ROS 环境 && source .venv/bin/activate
    python scripts/gen_dataset.py

产出（GZCHESS_DATASET_OUT_DIR，默认 ~/gzchess-dataset）：
    frames/000123.png            斜视帧
    labels/000123.json           {"placement": {"e4": "P", "d5": "n", ...}}  ← 世界真值（spawn 即真值）
    meta.json                    相机/棋盘配置快照（复现实验用）

标签怎么用（wave 4 训练）：训练侧用和**推理完全相同**的板角检测+透视矫正（脑仓
`src/tools/boardgame/_occupancy_vision.py`）把帧切成 64 个格子小图，配上这里的每格标签
（12 子型 + 空 = 13 类）。几何只有一份实现（推理即训练），不给"训练/推理两套坐标"留漂移空间。

随机策略：每帧 DATASET_MIN..MAX_PIECES 个子、格子/子型/颜色均匀随机（识别是逐格分类，
不需要"合法局面"；随机摆放对分类器覆盖更均匀）。王/后少给点权重防不真实的满盘王。
"""
from __future__ import annotations

import json
import os
import random
import sys
import time
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # world/gazebo-chess

import rclpy  # noqa: E402

import config  # noqa: E402
import geometry  # noqa: E402
import render  # noqa: E402
import spawn  # noqa: E402
from rclpy.node import Node  # noqa: E402

# 子型抽样权重（大致贴近真实对局中的出现频率，防满盘都是王；域常量级配方）
KIND_WEIGHTS = {"p": 8, "n": 2, "b": 2, "r": 2, "q": 1, "k": 1}


def _random_placement(rng: random.Random) -> dict[str, str]:
    n = rng.randint(config.DATASET_MIN_PIECES, config.DATASET_MAX_PIECES)
    squares = rng.sample([geometry.square_name(f, r) for f in range(8) for r in range(8)], n)
    kinds = list(KIND_WEIGHTS)
    weights = list(KIND_WEIGHTS.values())
    out = {}
    for sq in squares:
        k = rng.choices(kinds, weights)[0]
        out[sq] = k.upper() if rng.random() < 0.5 else k    # 大写白 / 小写黑
    return out


def _clear_pieces() -> None:
    for name in list(spawn.registry()):
        if name.startswith("piece_"):
            spawn.remove_model(name)


def main() -> None:
    out_dir = Path(config.DATASET_OUT_DIR or (Path.home() / "gzchess-dataset"))
    (out_dir / "frames").mkdir(parents=True, exist_ok=True)
    (out_dir / "labels").mkdir(parents=True, exist_ok=True)
    rng = random.Random()

    rclpy.init()
    node = Node("gzchess_dataset")
    cam = render.CameraFeed(node)
    spawn.spawn_board()
    spawn.spawn_camera()

    def spin_for(seconds: float) -> None:
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(node, timeout_sec=config.SPIN_STEP_S)

    spin_for(1.0)
    (out_dir / "meta.json").write_text(json.dumps({
        "cam_mode": config.CAM_MODE, "azim_deg": config.CAM_OBL_AZIM_DEG,
        "elev_deg": config.CAM_OBL_ELEV_DEG, "dist_m": config.CAM_OBL_DIST_M,
        "board_size_m": config.BOARD_SIZE_M, "w": config.CAM_W, "h": config.CAM_H,
        "n_target": config.DATASET_N}, ensure_ascii=False, indent=1))

    saved = 0
    for i in range(config.DATASET_N):
        _clear_pieces()
        placement = _random_placement(rng)
        for sq, sym in placement.items():
            spawn.spawn_piece(sq, "white" if sym.isupper() else "black",
                              name=f"piece_{sq}", kind=sym.lower())
        spin_for(config.DATASET_SETTLE_S)
        png = render.to_png(cam.frame)
        if png is None:
            print(f"[{i}] 没有相机帧（image_bridge 在跑吗？），跳过", flush=True)
            continue
        (out_dir / "frames" / f"{i:06d}.png").write_bytes(png)
        (out_dir / "labels" / f"{i:06d}.json").write_text(
            json.dumps({"placement": placement}, ensure_ascii=False))
        saved += 1
        if saved % 20 == 0:
            print(f"已存 {saved}/{config.DATASET_N} 帧 → {out_dir}", flush=True)

    _clear_pieces()
    node.destroy_node()
    rclpy.shutdown()
    print(f"完成：{saved} 帧（目标 {config.DATASET_N}）→ {out_dir}")


if __name__ == "__main__":
    main()

"""棋盘贴图 + move 占用拒绝的离线自测（不连 Gazebo；用世界 venv 跑）：
    ./.venv/bin/python scripts/test_board_texture.py

守的东西：
1. 贴图能生成、尺寸=格区+双边框；
2. **方向保真**：64 个格中心经 geometry→_square_to_px 映射到贴图像素后，取到的颜色
   与 (file+rank) 奇偶完全一致（a1=深）——贴图↔几何漂移在这里立刻红；
3. 无字体环境不抛（诚实降级只画格）；
4. board_sdf 含贴图 visual 且 XML 合法。
"""
from __future__ import annotations

import os
import sys
import xml.dom.minidom as minidom

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import board_texture  # noqa: E402
import config  # noqa: E402
import geometry  # noqa: E402
import models  # noqa: E402

fails = 0


def check(cond: bool, msg: str) -> None:
    global fails
    print(("[ok] " if cond else "[FAIL] ") + msg)
    if not cond:
        fails += 1


img = board_texture.render()
m = board_texture._margin_px()
expect = config.BOARD_FILES * board_texture.CELL_PX + 2 * m
check(img.size == (expect, expect), f"贴图尺寸 {img.size} = 格区+双边框 ({expect})")

# 方向保真：64 格中心像素颜色 vs 奇偶（用含最终旋转的 square_to_px——与 render 输出一致）
bad = []
for f in range(config.BOARD_FILES):
    for r in range(config.BOARD_RANKS):
        name = geometry.square_name(f, r)
        px, py = board_texture.square_to_px(name)
        got = img.getpixel((int(px), int(py)))
        want = board_texture.LIGHT_SQ if (f + r) % 2 == 1 else board_texture.DARK_SQ
        if got != want:
            bad.append(name)
check(not bad, f"64 格明暗与 (file+rank) 奇偶全部一致（a1=深）{'；错格：' + ','.join(bad[:5]) if bad else ''}")

# 角落抽查：a1 深、h1 浅、a8 浅、h8 深（标准棋盘）
for name, want in (("a1", board_texture.DARK_SQ), ("h1", board_texture.LIGHT_SQ),
                   ("a8", board_texture.LIGHT_SQ), ("h8", board_texture.DARK_SQ)):
    px, py = board_texture.square_to_px(name)
    check(img.getpixel((int(px), int(py))) == want, f"{name} 颜色符合标准棋盘")

# 无字体降级：指个不存在的字体路径且屏蔽发现候选 → 不抛、仍出图
os.environ["GZCHESS_BOARD_FONT"] = "/nonexistent.ttf"
real_exists = os.path.exists
try:
    board_texture.os.path.exists = lambda p: False  # 屏蔽所有候选
    img2 = board_texture.render()
    check(img2.size == img.size, "找不到字体时诚实降级（只画格），不抛异常")
finally:
    board_texture.os.path.exists = real_exists
    del os.environ["GZCHESS_BOARD_FONT"]

# board_sdf：XML 合法 + 含贴图 visual
sdf, _ = models.board_sdf()
minidom.parseString(sdf)
check("albedo_map" in sdf and "top_skin" in sdf, "board_sdf 含顶面贴图 visual 且 XML 合法")
check(str(config.BOARD_SIZE_M + 2 * config.BOARD_MARGIN_M) in sdf, "板总尺寸 = 格区 + 2×边框")

# move 占用拒绝：mock _piece_at（不连 gazebo/ROS，只测判定分支）
import types  # noqa: E402
import world as world_mod  # noqa: E402

w = types.SimpleNamespace(
    ready=True, lock=__import__("threading").RLock(),
    _piece_at=lambda sq: ((f"piece_{sq}", (0, 0, 0)) if sq in ("e2", "e4") else (None, None)),
)
res = world_mod.GazeboChessWorld._move(w, {"from": "e2", "to": "e4"})
check(not res["ok"] and "已经有子" in res["message"] and "remove" in res["message"],
      f"move 到占用格 → 可读拒绝并指路两步走：{res['message']!r}")

print(("\n全部通过 ✅" if fails == 0 else f"\n{fails} 项失败 ❌"))
sys.exit(1 if fails else 0)

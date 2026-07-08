"""棋盘顶面贴图生成器：深浅格 + 四边 a–h/1–8 坐标（标准棋盘图样式）。

为什么存在：v0.5 前棋盘是纯绿单色板（占用视觉靠"绿=板"检测，格位靠几何标定，不需要画格）。
现在 **LLM 就是视觉系统**——人眼看无格纹的板报不出"e2"，所以把棋盘画成人（和 LLM）读得懂的样子：
真实棋盘本来就有深浅格和边缘坐标，这不是喂真值，是把现实做真。

【方向对齐——本文件的命门】
格子明暗、坐标标签的位置**全部**从 `geometry.square_center_local()` 反推（`_square_to_px`），
不另写一套 file/rank→位置 的映射；贴图↔几何一旦漂移，机械臂就会把子放到"印着别的名字"的格上。
像素约定：贴图从 +z 俯视棋盘，图像 x=板局部 +x（a→h），图像 y **向下** = 板局部 −y（rank8 在图上方）
——与 gz box 顶面 UV 的常规取向一致；万一实测发现翻转，只改 `_square_to_px` 里的一个符号。

生成物：`.cache/board_texture.png`（gitignore；SDF 以绝对路径引用）。找不到字体→只画格不画字 + 告警。
"""
from __future__ import annotations

import glob
import hashlib
import io
import os

from PIL import Image, ImageDraw, ImageFont

import config
import geometry

# 贴图分辨率（像素；只影响清晰度不影响几何，非物理量所以放本文件而非 config env）
CELL_PX = 96                        # 每格边长像素（720p 相机下每格约 40-80px，源头给足）
_HERE = os.path.dirname(os.path.abspath(__file__))
_CACHE_DIR = os.path.join(_HERE, ".cache")

# 标准棋盘图配色（lichess 风格绿系；白/黑子在两色上都清晰）
LIGHT_SQ = (238, 238, 210)          # 浅格（米白偏绿）
DARK_SQ = (118, 150, 86)            # 深格（绿）
MARGIN_BG = (60, 66, 56)            # 边框深底
LABEL_FG = (240, 240, 240)          # 坐标白字


def _margin_px() -> int:
    return max(1, round(config.BOARD_MARGIN_M / config.CELL_M * CELL_PX))


def _square_to_px(name: str) -> tuple[float, float]:
    """格中心 → 贴图像素坐标（唯一的 格↔像素 映射；由 geometry.square_center_local 派生）。"""
    lx, ly = geometry.square_center_local(name)
    half = config.BOARD_SIZE_M / 2.0 + config.BOARD_MARGIN_M   # 板（含边框）半宽，米
    scale = CELL_PX / config.CELL_M                            # 米 → 像素
    px = (lx + half) * scale
    py = (half - ly) * scale                                   # 图像 y 向下 = 局部 −y
    return px, py


def square_to_px(name: str) -> tuple[float, float]:
    """格中心在**最终贴图**（含整体旋转）上的像素坐标——自测/调试用（与 render 输出一致）。"""
    x, y = _square_to_px(name)
    m = _margin_px()
    w = config.BOARD_FILES * CELL_PX + 2 * m
    for _ in range(config.BOARD_TEX_QUARTER_TURNS % 4):
        x, y = y, w - 1 - x                    # 与 PIL rotate(90)（逆时针）逐档同构
    return x, y


def _find_font(size: int):
    """发现式找粗体字体（env GZCHESS_BOARD_FONT 覆盖）；找不到返回 None（调用方降级不画字）。"""
    env = os.getenv("GZCHESS_BOARD_FONT")
    candidates = ([env] if env else []) + [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/TTF/DejaVuSans-Bold.ttf",
    ]
    for p in candidates:
        if p and os.path.exists(p):
            try:
                return ImageFont.truetype(p, size)
            except Exception:  # noqa: BLE001
                continue
    return None


def render() -> Image.Image:
    """画出整张贴图（格区 + 四边坐标）。纯函数，便于单测。"""
    m = _margin_px()
    size = config.BOARD_FILES * CELL_PX + 2 * m
    img = Image.new("RGB", (size, size), MARGIN_BG)
    d = ImageDraw.Draw(img)

    # 8×8 深浅格：颜色按 (file+rank) 奇偶（a1=深，标准棋盘约定），位置一律经 _square_to_px
    for f in range(config.BOARD_FILES):
        for r in range(config.BOARD_RANKS):
            name = geometry.square_name(f, r)
            cx, cy = _square_to_px(name)
            color = LIGHT_SQ if (f + r) % 2 == 1 else DARK_SQ
            d.rectangle([cx - CELL_PX / 2, cy - CELL_PX / 2, cx + CELL_PX / 2, cy + CELL_PX / 2],
                        fill=color)

    font = _find_font(int(m * 0.7))
    if font is None:
        print("[board_texture] ⚠️ 找不到可用字体（可设 GZCHESS_BOARD_FONT）——只画格不画坐标字")
        return img

    def _text_center(x: float, y: float, s: str) -> None:
        bbox = d.textbbox((0, 0), s, font=font)
        d.text((x - (bbox[0] + bbox[2]) / 2, y - (bbox[1] + bbox[3]) / 2), s, fill=LABEL_FG, font=font)

    # 四边坐标：列字母印在 rank1 下侧 + rank8 上侧的边框带；行数字印在 a 列左侧 + h 列右侧。
    # 位置全部由对应格中心 ±(半格+半边框) 推出——仍是同一套映射，不手排。
    off = CELL_PX / 2 + m / 2
    for f in range(config.BOARD_FILES):
        letter = chr(ord("a") + f)
        x1, y1 = _square_to_px(geometry.square_name(f, 0))                        # rank1 一侧
        _text_center(x1, y1 + off, letter)
        x8, y8 = _square_to_px(geometry.square_name(f, config.BOARD_RANKS - 1))   # rank8 一侧
        _text_center(x8, y8 - off, letter)
    for r in range(config.BOARD_RANKS):
        digit = str(r + 1)
        xa, ya = _square_to_px(geometry.square_name(0, r))                        # a 列一侧
        _text_center(xa - off, ya, digit)
        xh, yh = _square_to_px(geometry.square_name(config.BOARD_FILES - 1, r))   # h 列一侧
        _text_center(xh + off, yh, digit)
    # 最终整体旋转：补偿 gz box 顶面 UV 取向（金标准实测定档；见 config.BOARD_TEX_QUARTER_TURNS）。
    # 整图旋转不破坏"格↔标签"的相对关系——只是让画好的棋盘落到世界的正确方位上。
    turns = config.BOARD_TEX_QUARTER_TURNS % 4
    if turns:
        img = img.rotate(90 * turns)          # PIL rotate=逆时针；方形图无损
    return img


def ensure() -> str:
    """生成贴图文件并返回绝对路径。世界服务启动时调一次。

    文件名带**内容哈希**（board_texture_<hash>.png）：gz/Ogre 按路径缓存贴图——长活的 gz 进程
    对同名文件永远用旧缓存（2026-07-03 实锤：旋转后的贴图落盘了，渲染却纹丝不动）。
    内容变 → 文件名变 → 必然重读；旧哈希文件顺手清掉。"""
    os.makedirs(_CACHE_DIR, exist_ok=True)
    buf = io.BytesIO()
    render().save(buf, format="PNG")
    data = buf.getvalue()
    path = os.path.join(_CACHE_DIR, f"board_texture_{hashlib.md5(data).hexdigest()[:8]}.png")
    for old in glob.glob(os.path.join(_CACHE_DIR, "board_texture*.png")):
        if old != path:
            try:
                os.remove(old)
            except OSError:
                pass
    with open(path, "wb") as f:
        f.write(data)
    return path


if __name__ == "__main__":
    print(f"[ok] 贴图已生成：{ensure()}")

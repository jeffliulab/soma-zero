"""按 config 生成 Gazebo SDF 模型字符串：棋盘、棋子、俯视相机。

为什么不用静态 .sdf 文件：尺寸/位姿都要可配（禁硬编码），所以从 config 用 f-string 拼出来，
spawn 时再塞进 Gazebo。生成的是单个 <model> 片段（含 <sdf> 包裹），可直接喂 ros_gz 的 create。

坐标：模型 spawn 到 world 帧（= MoveIt 规划帧）。棋盘上表面在 BOARD_ORIGIN_Z。
"""
from __future__ import annotations

import json
import os

import config

# 网格 manifest 缓存与「缺文件只警告一次」登记（见 _mesh_visual）
_MESH_MANIFEST: dict | None = None
_MESH_WARNED: set[str] = set()


def _inertia_box(m: float, x: float, y: float, z: float) -> str:
    ixx = m * (y * y + z * z) / 12.0
    iyy = m * (x * x + z * z) / 12.0
    izz = m * (x * x + y * y) / 12.0
    return (f"<inertia><ixx>{ixx:.6g}</ixx><iyy>{iyy:.6g}</iyy><izz>{izz:.6g}</izz>"
            f"<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>")


def _inertia_cyl(m: float, r: float, h: float) -> str:
    ixx = m * (3 * r * r + h * h) / 12.0
    izz = m * r * r / 2.0
    return (f"<inertia><ixx>{ixx:.6g}</ixx><iyy>{ixx:.6g}</iyy><izz>{izz:.6g}</izz>"
            f"<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>")


def board_sdf(name: str = "chessboard") -> tuple[str, tuple[float, float, float]]:
    """棋盘底板：薄长方体，坐在桌面(z=0)上，上表面在 BOARD_ORIGIN_Z。
    返回 (sdf 字符串, spawn 世界坐标 xyz=模型原点)。模型原点在板几何中心。

    板总尺寸 = 格区(BOARD_SIZE_M) + 2×边框(BOARD_MARGIN_M)；顶面另加一层**贴图薄皮 visual**
    （board_texture 生成的 格纹+四边坐标 图——LLM 就是视觉系统，棋盘必须画成人读得懂的样子）。
    只给顶面薄皮贴图（不贴整个 box）：box 六面共享 UV，整贴会把侧面拉花。
    薄皮无碰撞体，物理不变；格中心坐标也不变（格区始终居中，geometry 按中心对称展开）。
    """
    import board_texture

    size = config.BOARD_SIZE_M + 2 * config.BOARD_MARGIN_M
    th = config.BOARD_THICKNESS_M
    skin = 0.001                                   # 顶面薄皮厚度（纯视觉，防 z-fighting 微凸 0.5mm）
    tex = board_texture.ensure()
    # 模型原点在板中心 → spawn z = 上表面 - 厚度/2
    spawn_xyz = (config.BOARD_ORIGIN_X, config.BOARD_ORIGIN_Y, config.BOARD_ORIGIN_Z - th / 2.0)
    sdf = f"""<sdf version="1.10">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="col">
        <geometry><box><size>{size} {size} {th}</size></box></geometry>
        <surface><friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction></surface>
      </collision>
      <visual name="body">
        <geometry><box><size>{size} {size} {th}</size></box></geometry>
        <material><ambient>0.20 0.25 0.20 1</ambient><diffuse>0.25 0.32 0.25 1</diffuse></material>
      </visual>
      <visual name="top_skin">
        <pose>0 0 {th / 2.0} 0 0 0</pose>
        <geometry><box><size>{size} {size} {skin}</size></box></geometry>
        <material>
          <diffuse>1 1 1 1</diffuse>
          <pbr><metal><albedo_map>{tex}</albedo_map><roughness>0.9</roughness><metalness>0.0</metalness></metal></pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
    return sdf, spawn_xyz


# 每种棋子的「剪影配方」（视觉专用，见 piece_sdf）：总高相对 PIECE_HEIGHT_M 的倍数——
# 高度阶梯 兵<车<马<象<后<王 是国象实物的惯例，让斜视下 CNN/人眼都能按剪影分型。
_KIND_HEIGHT_FACTOR = {"p": 1.0, "r": 1.1, "n": 1.15, "b": 1.25, "q": 1.4, "k": 1.5}


def _mesh_visual(kind: str, mat: str) -> str | None:
    """某子型的真实网格 visual（v0.7）。config.PIECE_MESH_DIR 为空 / manifest 缺 / 文件缺 → None
    （调用方回退几何剪影，缺哪型只回退哪型）。缩放 = 身高梯度目标高 / 网格原始高——
    与剪影同一把「身高尺」，头部保持在手腕以下（王 6.75cm < 抓取时 link6 底 ~7.8cm）。"""
    global _MESH_MANIFEST
    d = config.PIECE_MESH_DIR
    if not d:
        return None
    if _MESH_MANIFEST is None:
        try:
            with open(os.path.join(d, "manifest.json"), encoding="utf-8") as f:
                _MESH_MANIFEST = json.load(f)
        except Exception as e:  # noqa: BLE001
            _MESH_MANIFEST = {}
            print(f"[gazebo-chess] ⚠️ 棋子网格 manifest 读不到（{e}）——全部回退几何剪影")
    info = (_MESH_MANIFEST.get("pieces") or {}).get(kind)
    path = os.path.join(d, info["file"]) if info else ""
    if not info or not os.path.isfile(path):
        if kind not in _MESH_WARNED:
            _MESH_WARNED.add(kind)
            print(f"[gazebo-chess] ⚠️ 子型 {kind} 无网格（{path or 'manifest 未登记'}）——该型回退几何剪影")
        return None
    scale = config.PIECE_HEIGHT_M * _KIND_HEIGHT_FACTOR[kind] / float(info["natural_height_m"])
    return (f"""      <visual name="mesh"><pose>0 0 0 0 0 0</pose>
        <geometry><mesh><uri>file://{os.path.abspath(path)}</uri>"""
            f"""<scale>{scale:.5f} {scale:.5f} {scale:.5f}</scale></mesh></geometry>
        <material>{mat}</material></visual>
""")


def piece_sdf(name: str, color: str = "white", kind: str = "p") -> tuple[str, tuple[float, float, float]]:
    """一枚棋子：底座(宽)+ 抓取腰(GRASP_WIDTH 宽，高摩擦，给夹爪夹)+ 分型的头。
    返回 (sdf, spawn 世界 xyz=模型原点)。模型原点在棋子底面中心，spawn z = 棋盘上表面。
    color: white/black → 材质色；kind: p/n/b/r/q/k → 头部剪影（v0.5：斜视下能看出子型）。

    ⚠️ **碰撞体六种子完全一致**（沿用 v0.4 验证过的 底座+腰+头 三段圆柱）——分型只改【视觉】：
    抓取物理/IK 不因子型而变，wave 0 验通的夹取对所有子型直接成立。
    """
    kind = (kind or "p").lower()
    if kind not in _KIND_HEIGHT_FACTOR:
        kind = "p"
    base_r = config.PIECE_BASE_DIAM_M / 2.0
    waist_r = config.PIECE_GRASP_WIDTH_M / 2.0
    base_h = 0.008
    waist_h = 0.020                                     # 腰段高度（够夹爪指接触）
    head_r = waist_r * 0.7
    total_h = config.PIECE_HEIGHT_M                     # 碰撞体总高：全型一致（物理不变）
    head_h = max(0.004, total_h - base_h - waist_h)
    mass = 0.020
    if color == "white":
        mat = "<ambient>0.85 0.82 0.72 1</ambient><diffuse>0.92 0.90 0.82 1</diffuse>"
    else:
        mat = "<ambient>0.06 0.06 0.07 1</ambient><diffuse>0.10 0.10 0.12 1</diffuse>"
    spawn_xyz = (0.0, 0.0, config.BOARD_ORIGIN_Z)        # x,y 由 spawn 调用者按棋格填
    # 三段叠起来：底座 [0,base_h]，腰 [base_h, base_h+waist_h]，头在其上。抓取点在腰中段。
    base_cz = base_h / 2.0
    waist_cz = base_h + waist_h / 2.0
    head_cz = base_h + waist_h + head_h / 2.0
    # 高摩擦让夹爪靠接触摩擦夹得住（腰段尤其重要）。
    fric = "<surface><friction><ode><mu>1.2</mu><mu2>1.2</mu2></ode></friction></surface>"
    # 视觉：优先真实网格（v0.7，只换外观）；无网格 → v0.5 几何剪影（该型单独回退，T0）。
    visuals = _mesh_visual(kind, mat)
    if visuals is None:
        visuals = (f"""      <visual name="base"><pose>0 0 {base_cz:.4f} 0 0 0</pose>
        <geometry><cylinder><radius>{base_r:.4f}</radius><length>{base_h}</length></cylinder></geometry>
        <material>{mat}</material></visual>
      <visual name="waist"><pose>0 0 {waist_cz:.4f} 0 0 0</pose>
        <geometry><cylinder><radius>{waist_r:.4f}</radius><length>{waist_h}</length></cylinder></geometry>
        <material>{mat}</material></visual>
""" + _head_visuals(kind, head_r, base_h + waist_h, total_h, mat))
    sdf = f"""<sdf version="1.10">
  <model name="{name}">
    <link name="link">
      <inertial><mass>{mass}</mass>{_inertia_cyl(mass, waist_r, total_h)}
        <pose>0 0 {total_h/2.0:.4f} 0 0 0</pose></inertial>
      <collision name="base"><pose>0 0 {base_cz:.4f} 0 0 0</pose>
        <geometry><cylinder><radius>{base_r:.4f}</radius><length>{base_h}</length></cylinder></geometry>
        {fric}</collision>
      <collision name="waist"><pose>0 0 {waist_cz:.4f} 0 0 0</pose>
        <geometry><cylinder><radius>{waist_r:.4f}</radius><length>{waist_h}</length></cylinder></geometry>
        {fric}</collision>
      <collision name="head"><pose>0 0 {head_cz:.4f} 0 0 0</pose>
        <geometry><cylinder><radius>{head_r:.4f}</radius><length>{head_h:.4f}</length></cylinder></geometry>
        {fric}</collision>
{visuals}    </link>
  </model>
</sdf>"""
    return sdf, spawn_xyz


def _head_visuals(kind: str, head_r: float, z0: float, collision_total_h: float, mat: str) -> str:
    """每种棋子的头部【视觉】剪影（碰撞体不在此、全型一致）。z0 = 腰顶高度。

    剪影配方（简单几何体，斜视下轮廓可分）：
      p 兵=矮柱+圆球  r 车=粗短柱+宽扁盖  n 马=柱+前倾斜块(不对称)  b 象=细高柱+小球尖
      q 后=高柱+球+小球冠  k 王=最高柱+十字
    """
    total = config.PIECE_HEIGHT_M * _KIND_HEIGHT_FACTOR[kind]
    body_h = max(0.004, total - z0)

    def cyl(nm, r, h, cz, extra_pose="0 0 0"):
        return (f'      <visual name="{nm}"><pose>0 0 {cz:.4f} {extra_pose}</pose>\n'
                f'        <geometry><cylinder><radius>{r:.4f}</radius><length>{h:.4f}</length></cylinder></geometry>\n'
                f'        <material>{mat}</material></visual>\n')

    def sph(nm, r, cz):
        return (f'      <visual name="{nm}"><pose>0 0 {cz:.4f} 0 0 0</pose>\n'
                f'        <geometry><sphere><radius>{r:.4f}</radius></sphere></geometry>\n'
                f'        <material>{mat}</material></visual>\n')

    def box(nm, x, y, z, cx, cy, cz, rpy="0 0 0"):
        return (f'      <visual name="{nm}"><pose>{cx:.4f} {cy:.4f} {cz:.4f} {rpy}</pose>\n'
                f'        <geometry><box><size>{x:.4f} {y:.4f} {z:.4f}</size></box></geometry>\n'
                f'        <material>{mat}</material></visual>\n')

    if kind == "r":       # 车：粗短柱 + 宽扁盖
        h1 = body_h * 0.7
        return (cyl("head", head_r * 1.1, h1, z0 + h1 / 2)
                + cyl("cap", head_r * 1.5, body_h * 0.3, z0 + h1 + body_h * 0.15))
    if kind == "n":       # 马：柱 + 前倾斜块（唯一不对称剪影）
        h1 = body_h * 0.55
        return (cyl("head", head_r * 0.9, h1, z0 + h1 / 2)
                + box("snout", head_r * 2.6, head_r * 1.2, head_r * 1.2,
                      head_r * 0.8, 0.0, z0 + h1 + head_r * 0.5, rpy="0 0.5 0"))
    if kind == "b":       # 象：细高柱 + 小球尖
        h1 = body_h * 0.8
        return (cyl("head", head_r * 0.75, h1, z0 + h1 / 2)
                + sph("tip", head_r * 0.55, z0 + h1 + head_r * 0.4))
    if kind == "q":       # 后：高柱 + 球 + 小球冠
        h1 = body_h * 0.65
        return (cyl("head", head_r * 0.9, h1, z0 + h1 / 2)
                + sph("orb", head_r * 1.05, z0 + h1 + head_r * 0.6)
                + sph("crown", head_r * 0.45, z0 + h1 + head_r * 1.9))
    if kind == "k":       # 王：最高柱 + 十字
        h1 = body_h * 0.75
        cz = z0 + h1
        return (cyl("head", head_r * 0.9, h1, z0 + h1 / 2)
                + box("cross_v", head_r * 0.5, head_r * 0.5, head_r * 2.2, 0, 0, cz + head_r * 1.0)
                + box("cross_h", head_r * 1.8, head_r * 0.5, head_r * 0.5, 0, 0, cz + head_r * 1.2))
    # 兵（默认）：矮柱 + 圆球
    h1 = body_h * 0.5
    return (cyl("head", head_r, h1, z0 + h1 / 2)
            + sph("ball", head_r * 0.95, z0 + h1 + head_r * 0.6))


def camera_sdf(kind: str = "oblique") -> tuple[str, tuple[float, float, float], tuple[float, float, float]]:
    """一路棋盘相机（kind = "oblique" 斜视 | "overhead" 正俯视；双相机=两次调用各自 spawn）。
    返回 (sdf, spawn xyz, spawn rpy)。像素发布在 config.cam_topic(kind)（每路独立 gz 话题），
    再用 image_bridge 逐路桥到 ROS。

    - oblique：从板局部方位角 AZIM（默认 -90°=白方/rank1 一侧）、俯角 ELEV、距离 DIST 看向棋盘中心。
      Gazebo 相机沿 +x 拍：yaw 指向棋盘中心、pitch=俯角。默认机位下图像下缘=rank1、左=a 列，
      与从前视觉桥的方向约定一致。
    - overhead：架在棋盘中心正上方 CAM_HEIGHT 处朝下拍（v0.4 原样）。
    """
    name = f"{kind}_cam"
    if kind == "oblique":
        import math
        azim = math.radians(config.CAM_OBL_AZIM_DEG) + config.BOARD_YAW_RAD   # 板局部方位 → world
        elev = math.radians(config.CAM_OBL_ELEV_DEG)
        d_xy = config.CAM_OBL_DIST_M * math.cos(elev)
        spawn_xyz = (config.BOARD_ORIGIN_X + d_xy * math.cos(azim),
                     config.BOARD_ORIGIN_Y + d_xy * math.sin(azim),
                     config.BOARD_ORIGIN_Z + config.CAM_OBL_DIST_M * math.sin(elev))
        spawn_rpy = (0.0, elev, azim + math.pi)          # 朝回棋盘中心、往下俯 elev
    else:
        spawn_xyz = (config.BOARD_ORIGIN_X, config.BOARD_ORIGIN_Y, config.BOARD_ORIGIN_Z + config.CAM_HEIGHT_M)
        spawn_rpy = tuple(config.CAM_RPY)
    sdf = f"""<sdf version="1.10">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <sensor name="cam" type="camera">
        <always_on>1</always_on>
        <update_rate>{config.CAM_FPS}</update_rate>
        <visualize>false</visualize>
        <topic>{config.cam_topic(kind)}</topic>
        <camera>
          <horizontal_fov>{config.CAM_FOV_RAD}</horizontal_fov>
          <image><width>{config.CAM_W}</width><height>{config.CAM_H}</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>5.0</far></clip>
        </camera>
      </sensor>
    </link>
  </model>
</sdf>"""
    return sdf, spawn_xyz, spawn_rpy


if __name__ == "__main__":
    # 离线校验：生成的 SDF 是合法 XML，且关键尺寸来自 config（不硬编码）。
    import xml.dom.minidom as _m

    for label, gen in (("board", lambda: board_sdf()[0]),
                       ("piece_white", lambda: piece_sdf("p", "white")[0]),
                       ("piece_black", lambda: piece_sdf("p", "black")[0]),
                       ("camera", lambda: camera_sdf()[0])):
        xml = gen()
        _m.parseString(xml)   # 不合法会抛异常
        print(f"[ok] {label} SDF 合法（{len(xml)} 字符）")
    print("board spawn xyz =", board_sdf()[1])
    print("camera spawn xyz/rpy =", camera_sdf()[1], camera_sdf()[2])
    print("piece spawn z (棋盘面) =", piece_sdf('p')[1][2])

"""算「在某个世界点抓一个子」时，末端 link6 该摆成的位姿（位置 + 朝向四元数），纯几何、可离线测。

要点：
- 夹爪的指沿 link6 的 +Z 伸出，所以从上往下抓 = link6 的 +Z 朝下（世界 -Z）。基础朝向 = 绕 x 转 180°。
- 两指中间的抓取点(TCP) 在 link6 +Z 方向 TCP_OFFSET 处。**link6 原点 = 抓取点 − TCP_OFFSET·ẑ_tool**
  （ẑ_tool = 倾斜后的工具轴方向）：竖直抓时 ẑ_tool=(0,0,-1)，link6 在抓取点正上方 TCP_OFFSET——与旧版一致；
  倾斜抓时 link6 挪到抓取点的斜上方、靠基座一侧。v0.4-0.6 的旧实现不论倾角都把 link6 放正上方，
  倾斜候选的 TCP 根本没对准抓取点（一直没暴露是因为旧版基本只用竖直档）。
- 倾斜是**径向**的（v0.7）：算出目标点相对臂基座的方位角 ψ，让工具从竖直朝下向「远离基座」的方向倒
  tilt 角——ẑ_tool = (sinT·cosψ, sinT·sinψ, −cosT)。这是「臂伸直去取远格」的姿态；
  旧实现固定绕工具 y 轴倾，方向和目标方位无关，远格基本解不出。
- 接近点 = 沿工具轴从抓取位姿退开 APPROACH_SAFE_M（竖直档 = 正上方抬高，与旧版一致；
  倾斜档 = 斜上方退开，进出都沿轴，不横扫）。
- 备用自由度：手腕自转（绕工具轴加 yaw）× 径向倾角档（APPROACH_TILT_DEG）。
- 返回若干候选位姿（最优在前：竖直、按邻子净空排 yaw），上层（arm_controller）逐个试 IK，挑能成的。
"""
from __future__ import annotations

import math

import config
import geometry

Quat = tuple[float, float, float, float]   # (x, y, z, w)
Pose = tuple[tuple[float, float, float], Quat]
Vec3 = tuple[float, float, float]


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> Quat:
    """RPY(绕固定轴 x→y→z 复合，R=Rz·Ry·Rx) → 四元数 (x,y,z,w)。"""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return (x, y, z, w)


def quat_mul(a: Quat, b: Quat) -> Quat:
    """四元数乘法（Hamilton 约定；R(a·b)=R(a)·R(b)，即先转 b 再转 a）。"""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def rot_vec(q: Quat, v: Vec3) -> Vec3:
    """用四元数旋转向量：v' = q·v·q*（展开为免开销的叉积形式）。"""
    qx, qy, qz, qw = q
    vx, vy, vz = v
    tx = 2 * (qy * vz - qz * vy)
    ty = 2 * (qz * vx - qx * vz)
    tz = 2 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def _quat_about_z(a: float) -> Quat:
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def _quat_about_y(a: float) -> Quat:
    return (0.0, math.sin(a / 2), 0.0, math.cos(a / 2))


def _down_quat(wrist_yaw_rad: float, tilt_rad: float = 0.0, azimuth_rad: float = 0.0) -> Quat:
    """朝下的末端朝向：基础 roll=π 让 +Z 朝下、wrist_yaw 绕竖直轴自转；
    tilt>0 时再整体施加径向倾斜 A = Rz(ψ)·Ry(−T)·Rz(−ψ)——把工具轴从 (0,0,-1)
    倒成 (sinT·cosψ, sinT·sinψ, −cosT)，即朝方位角 ψ（远离基座）的方向倒 T 角。"""
    q0 = quat_from_rpy(math.pi, 0.0, wrist_yaw_rad)
    if tilt_rad == 0.0:
        return q0
    tilt_q = quat_mul(_quat_about_z(azimuth_rad),
                      quat_mul(_quat_about_y(-tilt_rad), _quat_about_z(-azimuth_rad)))
    return quat_mul(tilt_q, q0)


def tool_axis(q: Quat) -> Vec3:
    """工具轴方向（link6 +Z 经 q 旋转后的世界方向；抓取时指向抓取点）。"""
    return rot_vec(q, (0.0, 0.0, 1.0))


def link6_pose_for_grasp(px: float, py: float, pz: float,
                         wrist_yaw_rad: float = 0.0, tilt_rad: float = 0.0,
                         azimuth_rad: float = 0.0) -> Pose:
    """给抓取点 (px,py,pz)，算 link6 该在的位姿：link6 = 抓取点 − TCP_OFFSET·ẑ_tool。
    竖直（tilt=0）时 = 抓取点正上方 TCP_OFFSET，与旧版完全一致。"""
    q = _down_quat(wrist_yaw_rad, tilt_rad, azimuth_rad)
    zx, zy, zz = tool_axis(q)
    t = config.TCP_OFFSET_M
    return ((px - t * zx, py - t * zy, pz - t * zz), q)


def _finger_clearance(px: float, py: float, q: Quat, avoid_xy) -> float:
    """某候选朝向下，两个指尖(全开)到最近邻子的水平距离（净空，越大越安全）。
    指展方向 = 工具 Y 轴经 q 旋转后的方向（真实几何，不再用 yaw 近似推断）；
    指尖水平位置 = 抓取点 ± 半指展 × 指展轴的水平分量。"""
    if not avoid_xy:
        return math.inf
    half_span = (config.GRIP_FACE_GAP_CLOSED_M + 2 * config.GRIP_OPEN_M) / 2
    fx, fy, _fz = rot_vec(q, (0.0, 1.0, 0.0))
    tips = ((px + fx * half_span, py + fy * half_span),
            (px - fx * half_span, py - fy * half_span))
    return min(math.hypot(tx - ax, ty - ay) for tx, ty in tips for ax, ay in avoid_xy)


def candidates_for_point(px: float, py: float, pz: float,
                         avoid_xy: list[tuple[float, float]] | None = None) -> list[tuple[str, Pose, Pose]]:
    """某抓取点的 (标签, 接近位姿, 抓取位姿) 候选列表，最优在前。
    接近位姿 = 抓取位姿沿工具轴退开 APPROACH_SAFE_M（竖直档=正上方，倾斜档=斜上方，进出沿轴）。
    先竖直(tilt=0)各 wrist yaw，再按 APPROACH_TILT_DEG 逐档径向倾斜（倾斜方位 = 基座→目标）。
    avoid_xy（可选）= 邻近棋子的水平坐标：同一 tilt 内按「指尖离邻子净空」从大到小排 yaw——
    指展方向避开有子的邻格（多子棋盘的防撞关键；世界=物理，用自己的真值做抓取规划是本分）。"""
    out: list[tuple[str, Pose, Pose]] = []
    base_x, base_y = config.ARM_BASE_XY
    azim = math.atan2(py - base_y, px - base_x)
    tilts = [0.0] + [math.radians(t) for t in config.APPROACH_TILT_DEG if t != 0]
    yaws = [math.radians(y) for y in config.WRIST_ROLL_DEG]
    for tilt in tilts:
        scored: list[tuple[float, float]] = []
        for yaw in yaws:
            q = _down_quat(yaw, tilt, azim)
            scored.append((_finger_clearance(px, py, q, avoid_xy or []), yaw))
        scored.sort(key=lambda cy: -cy[0])
        for _clr, yaw in scored:
            (gx, gy, gz), q = link6_pose_for_grasp(px, py, pz, yaw, tilt, azim)
            zx, zy, zz = tool_axis(q)
            s = config.APPROACH_SAFE_M
            grasp = ((gx, gy, gz), q)
            approach = ((gx - s * zx, gy - s * zy, gz - s * zz), q)
            label = f"tilt{round(math.degrees(tilt))}_yaw{round(math.degrees(yaw))}"
            out.append((label, approach, grasp))
    return out


def candidates_for_square(square: str) -> list[tuple[str, Pose, Pose]]:
    """某棋格的抓取候选（格中心、棋子腰高为抓取点）。"""
    gx, gy, gz = geometry.grasp_xyz(square)
    return candidates_for_point(gx, gy, gz)


if __name__ == "__main__":
    # 离线自测：四元数运算、径向倾斜方向、TCP/接近点几何、竖直档与旧行为一致。
    def norm(q):
        return math.sqrt(sum(c * c for c in q))

    # 1) 竖直档：单位四元数、+Z 朝下、link6 在抓取点正上方 TCP_OFFSET（与 v0.4-0.6 完全一致）
    q = _down_quat(0.0)
    assert abs(norm(q) - 1.0) < 1e-9, q
    assert rot_vec(q, (0, 0, 1))[2] < -0.999999, "+Z 没朝下"
    (gx, gy, gz), _ = link6_pose_for_grasp(0.30, -0.12, 0.028)
    assert abs(gx - 0.30) < 1e-12 and abs(gy + 0.12) < 1e-12
    assert abs(gz - (0.028 + config.TCP_OFFSET_M)) < 1e-12

    # 2) 径向倾斜：任意方位角/倾角下，工具轴 = (sinT·cosψ, sinT·sinψ, −cosT)
    for psi_deg in (0, 37, 90, 135, -60, 180):
        for tilt_deg in (15, 45, 60, 75):
            psi, T = math.radians(psi_deg), math.radians(tilt_deg)
            for yaw in (0.0, math.radians(90), math.radians(-45)):
                qq = _down_quat(yaw, T, psi)
                assert abs(norm(qq) - 1.0) < 1e-9
                zx, zy, zz = tool_axis(qq)
                exp = (math.sin(T) * math.cos(psi), math.sin(T) * math.sin(psi), -math.cos(T))
                assert all(abs(a - b) < 1e-9 for a, b in zip((zx, zy, zz), exp)), \
                    f"ψ={psi_deg} T={tilt_deg} yaw={math.degrees(yaw):.0f}: 工具轴 {(zx,zy,zz)} ≠ 期望 {exp}"

    # 3) 倾斜档 TCP 对准：link6 + TCP_OFFSET·ẑ_tool 必须落回抓取点（旧实现在这条上是错的）
    p = (0.52, 0.14, 0.048)
    psi = math.atan2(p[1] - config.ARM_BASE_XY[1], p[0] - config.ARM_BASE_XY[0])
    (lx, ly, lz), qq = link6_pose_for_grasp(*p, wrist_yaw_rad=0.3, tilt_rad=math.radians(60), azimuth_rad=psi)
    zx, zy, zz = tool_axis(qq)
    t = config.TCP_OFFSET_M
    back = (lx + t * zx, ly + t * zy, lz + t * zz)
    assert all(abs(a - b) < 1e-9 for a, b in zip(back, p)), (back, p)
    # 倾斜时 link6 应在抓取点的基座一侧（x 更小）且更高
    assert lx < p[0] and lz > p[2]

    # 4) 候选清单：接近点沿工具轴退开 APPROACH_SAFE_M；竖直档接近点 = 正上方抬高
    cands = candidates_for_square("e2")
    lbl, (ax, ay, az), _q1 = cands[0][0], cands[0][1][0], cands[0][1][1]
    _lbl, (ggx, ggy, ggz), _q2 = cands[0][0], cands[0][2][0], cands[0][2][1]
    assert lbl.startswith("tilt0"), lbl
    assert abs(ax - ggx) < 1e-12 and abs(ay - ggy) < 1e-12
    assert abs((az - ggz) - config.APPROACH_SAFE_M) < 1e-12
    for _label, (apos, aq), (gpos, gq) in cands:
        assert aq == gq
        d = math.dist(apos, gpos)
        assert abs(d - config.APPROACH_SAFE_M) < 1e-9, (_label, d)

    # 5) 邻子净空排序仍生效：有邻子时首选 yaw 的指展应避开它
    gx, gy, gz = geometry.grasp_xyz("e2")
    nx, ny, _ = geometry.grasp_xyz("e1")   # 南邻有子
    first = candidates_for_point(gx, gy, gz, avoid_xy=[(nx, ny)])[0]
    q_first = first[2][1]
    q_worst = _down_quat(0.0)              # yaw=0 指展沿 Y，正对南邻
    assert _finger_clearance(gx, gy, q_first, [(nx, ny)]) >= _finger_clearance(gx, gy, q_worst, [(nx, ny)]) - 1e-9

    n_tilt = len([0.0] + [x for x in config.APPROACH_TILT_DEG if x != 0])
    print(f"候选数 = {len(cands)}（tilt {n_tilt} 档 × yaw {len(config.WRIST_ROLL_DEG)} 档）")
    print("e2 抓取点(world) =", tuple(round(v, 3) for v in geometry.grasp_xyz("e2")))
    print("最优候选:", cands[0][0])
    far = candidates_for_square("h8")
    print("h8 抓取点(world) =", tuple(round(v, 3) for v in geometry.grasp_xyz("h8")),
          f" 离基座水平距 = {math.hypot(*geometry.grasp_xyz('h8')[:2]):.3f} m")
    print("OK")

"""把棋盘/棋子/相机 spawn 进正在运行的 Gazebo 世界，并维护「每子真实位姿登记表」。

走 CLI（`ros2 run ros_gz_sim create` / `gz`），所以只要 shell source 了 ROS2 就能用，不强依赖 rclpy。
登记表（_REGISTRY）记下每个 spawn 的棋子叫什么、在哪格、spawn 时的世界坐标——这是仿真里的"标准答案"，
世界做抓取定位、自检、ground-truth 兜底时用。读棋子当前真实位姿用 `gz model --pose`。

前提：episode 仿真栈已在跑（用户亲手起）；本模块只往世界里加东西/读位姿，不动机械臂。
"""
from __future__ import annotations

import re
import subprocess
import tempfile
import time

import config
import geometry
import models

# 棋子登记表：name -> {"square": "e2", "color": "white", "spawn_xyz": (x,y,z)}
_REGISTRY: dict[str, dict] = {}


def _run(cmd: list[str], timeout: float = 20.0) -> tuple[bool, str]:
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return p.returncode == 0, (p.stdout + p.stderr)
    except Exception as e:  # noqa: BLE001
        return False, f"{type(e).__name__}: {e}"


def _create(sdf: str, name: str, xyz: tuple[float, float, float],
            rpy: tuple[float, float, float] = (0.0, 0.0, 0.0), replace: bool = True) -> tuple[bool, str]:
    """用 ros_gz create 把一段 SDF spawn 进世界（写临时文件再 -file，避开 -string 的引号坑）。
    replace=True：先**删净**同名模型再 spawn（purge_model：反复删+确认消失）——幂等，
    避免反复 spawn 堆出一堆同名重复体污染世界（前科：5 个 overhead_cam 叠在一起；
    以及 gz remove 是异步的，单次 remove+固定 sleep 在慢机器上会删不干净）。"""
    if replace:
        purge_model(name)
    with tempfile.NamedTemporaryFile("w", suffix=".sdf", delete=False) as f:
        f.write(sdf)
        path = f.name
    cmd = ["ros2", "run", "ros_gz_sim", "create",
           "-world", config.GZ_WORLD_NAME, "-file", path, "-name", name,
           "-x", f"{xyz[0]:.5f}", "-y", f"{xyz[1]:.5f}", "-z", f"{xyz[2]:.5f}",
           "-R", f"{rpy[0]:.5f}", "-P", f"{rpy[1]:.5f}", "-Y", f"{rpy[2]:.5f}"]
    return _run(cmd)


def set_model_pose(name: str, xyz: tuple[float, float, float],
                   quat: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)) -> tuple[bool, str]:
    """直接改一个已有模型的世界位姿（gz set_pose 服务）。调相机朝向时用它，免得 remove+respawn。"""
    req = (f'name: "{name}" '
           f'position {{ x: {xyz[0]:.5f} y: {xyz[1]:.5f} z: {xyz[2]:.5f} }} '
           f'orientation {{ x: {quat[0]:.6f} y: {quat[1]:.6f} z: {quat[2]:.6f} w: {quat[3]:.6f} }}')
    return _run(["gz", "service", "-s", f"/world/{config.GZ_WORLD_NAME}/set_pose",
                 "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
                 "--timeout", "3000", "--req", req])


def remove_model(name: str) -> tuple[bool, str]:
    """从世界删掉一个模型（gz remove 服务）。"""
    req = f'name: "{name}" type: MODEL'
    ok, out = _run(["gz", "service", "-s", f"/world/{config.GZ_WORLD_NAME}/remove",
                    "--reqtype", "gz.msgs.Entity", "--reptype", "gz.msgs.Boolean",
                    "--timeout", "3000", "--req", req])
    _REGISTRY.pop(name, None)
    return ok, out


def purge_model(name: str, tries: int = 5) -> bool:
    """删净一个模型：反复 remove 并用位姿流确认它真的消失了才罢休（gz 删除是异步的）。
    返回是否确认删净；删不净也不抛——调用方 spawn 后自有发布者数自检兜底。"""
    for _ in range(tries):
        remove_model(name)
        time.sleep(0.3)
        if name not in all_model_poses(window_s=0.5):
            return True
    return False


def publisher_count(topic: str) -> int:
    """一个 gz 话题上有几个 Image 发布者。健康值=1；>1 说明有残留相机/孤儿 gz 进程在抢话题
    （两份不同画面会交替混流——2026-07-02 实锤过一次：15 小时的孤儿 gz sim server）。"""
    _, out = _run(["gz", "topic", "-i", "-t", topic], timeout=6)
    # 只数 Publishers 段——输出里 Subscribers 段（如 image_bridge）也带 gz.msgs.Image，不能算进来
    return out.split("Subscribers")[0].count("gz.msgs.Image")


def spawn_board(name: str = "chessboard") -> tuple[bool, str]:
    sdf, xyz = models.board_sdf(name)
    return _create(sdf, name, xyz)


def spawn_camera(kind: str = "oblique") -> tuple[bool, str]:
    """spawn 一路相机（kind=oblique/overhead，模型名 <kind>_cam，话题 config.cam_topic(kind)）。"""
    sdf, xyz, rpy = models.camera_sdf(kind)
    return _create(sdf, f"{kind}_cam", xyz, rpy)


def spawn_piece(square: str, color: str = "white", name: str | None = None,
                kind: str = "p") -> tuple[bool, str]:
    """在某格 spawn 一枚棋子。name 默认 piece_<square>。kind=p/n/b/r/q/k（视觉剪影，物理全型一致）。"""
    name = name or f"piece_{square}"
    sdf, base_xyz = models.piece_sdf(name, color, kind)
    bx, by, bz = geometry.square_surface_xyz(square)   # 格中心、棋盘上表面（= 棋子底面）
    xyz = (bx, by, bz)
    ok, out = _create(sdf, name, xyz)
    if ok:
        _REGISTRY[name] = {"square": square, "color": color, "kind": kind, "spawn_xyz": xyz}
    return ok, out


def spawn_piece_at(xyz: tuple[float, float, float], color: str = "white",
                   name: str | None = None, kind: str = "p") -> tuple[bool, str]:
    """在任意 world 坐标 spawn 一枚棋子（不绑格子）。给 place 从「备用子区」取新子用。
    返回 (ok, name)（成功时第二项是模型名，供随后夹取）；失败时第二项是错误输出。"""
    name = name or f"piece_res_{int(time.time() * 1000)}"
    sdf, _ = models.piece_sdf(name, color, kind)
    ok, out = _create(sdf, name, tuple(xyz))
    if ok:
        _REGISTRY[name] = {"square": None, "color": color, "kind": kind, "spawn_xyz": tuple(xyz)}
        return True, name
    return False, out


def note_square(name: str, square: str | None) -> None:
    """teleport（set_model_pose）后同步登记表的所在格——登记表保持诚实。
    （_piece_at 按实时位姿找子、不依赖登记表，这里只是别让台账烂掉。）"""
    if name in _REGISTRY:
        _REGISTRY[name]["square"] = square


def registry() -> dict[str, dict]:
    return dict(_REGISTRY)


def _parse_pose_vector(text: str) -> dict[str, tuple[float, float, float]]:
    """解析 `gz topic -e -t .../pose/info`（gz.msgs.Pose_V 文本）→ {name: (x,y,z)}，取每名最后一次。

    实测 gz.msgs.Pose_V 文本每块顺序是 `name: "X"  id: N  position{ x y z }  orientation{ x y z w }`——
    **name 在 position 之前**。所以按行扫：遇到 name 记住它，遇到本块 position 读 x,y,z 配给它（跳过 orientation）。"""
    # 注意：gz 文本会**省略值为 0 的字段**（比如 y=0 时整行 `y: 0` 不出现）。
    # 所以缺的坐标按 0 补——不能因为缺 y 行就丢掉这条。position 块结束（遇 orientation 或下个 name）时定稿。
    poses: dict[str, tuple[float, float, float]] = {}
    cur_name = None
    cur = {}
    mode = None       # "pos" 时才记 x/y/z

    def _flush():
        if cur_name is not None and mode == "pos":
            poses[cur_name] = (cur.get("x", 0.0), cur.get("y", 0.0), cur.get("z", 0.0))

    for raw in text.splitlines():
        ln = raw.strip()
        if ln.startswith("name:"):
            _flush()
            cur_name = ln.split('"')[1] if '"' in ln else ln.split(":", 1)[1].strip()
            mode = None
        elif ln.startswith("position"):
            mode = "pos"; cur = {}
        elif ln.startswith("orientation"):
            _flush(); mode = None
        elif mode == "pos" and ln[:2] in ("x:", "y:", "z:"):
            cur[ln[0]] = float(ln.split(":", 1)[1])
    _flush()
    return poses


def all_model_poses(window_s: float = 1.5) -> dict[str, tuple[float, float, float]]:
    """抓一小段 pose/info，解析出所有实体的世界位姿。`gz model --list/--pose` 是抢拍快照、不可靠，故走这个。"""
    _, out = _run(["timeout", str(window_s), "gz", "topic", "-e",
                   "-t", f"/world/{config.GZ_WORLD_NAME}/pose/info"], timeout=window_s + 5.0)
    return _parse_pose_vector(out)   # timeout 杀进程返回非 0，但 stdout 已有内容，直接解析


def model_pose(name: str, window_s: float = 1.5) -> tuple[float, float, float] | None:
    """读某模型当前真实世界位姿 (x,y,z)（米）。拿不到返回 None。"""
    return all_model_poses(window_s).get(name)


if __name__ == "__main__":
    # 离线只校验登记表/坐标逻辑（不连仿真）；连仿真的实测在 scripts/test_spawn.py。
    sdf, base_xyz = models.piece_sdf("piece_e2", "white")
    bx, by, bz = geometry.square_surface_xyz("e2")
    print(f"piece_e2 应 spawn 到 world ({bx:.3f},{by:.3f},{bz:.3f})")
    print(f"camera 应 spawn 到 {models.camera_sdf()[1]} rpy {models.camera_sdf()[2]}")
    print("（连仿真的 spawn 实测见 scripts/test_spawn.py）")

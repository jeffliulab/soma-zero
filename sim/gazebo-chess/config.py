"""gazebo-chess 世界的可调项集中地。

约定（对齐 anima-zero 各世界做法 + 开发指南「禁止硬编码」）：
- 所有可调值都在这里，用 GZCHESS_* 环境变量给默认值；别的模块从这里取，不要在代码里 inline 魔法数字。
- 世界进程独立运行，**不 import 大脑的 src/config.py**；这是世界自带的配置。
- 域常量（8×8、格名 a-h / 1-8、夹爪固有几何）属「定义」，不算硬编码。

单位：长度米、角度弧度（角度档用度，便于人读，取用时转弧度）。
"""
from __future__ import annotations

import os
from pathlib import Path


def _f(name: str, default: str) -> float:
    return float(os.getenv(name, default))


def _i(name: str, default: str) -> int:
    return int(os.getenv(name, default))


def _deg_list(name: str, default: str) -> list[float]:
    return [float(x) for x in os.getenv(name, default).split(",") if x.strip() != ""]


# ---- 世界标识 / 服务 ----
WORLD_VERSION = os.getenv("GZCHESS_VERSION", "0.7")
PORT = _i("GZCHESS_PORT", "8106")
STREAM_FPS = _i("GZCHESS_STREAM_FPS", "15")          # 人类页实时视频帧率

# ---- 棋盘几何（米；坐标系 = MoveIt 规划帧 `world` = Gazebo 世界帧）----
# 重要：episode 孪生给 base_link 上面焊了个 `world` 链接，MoveIt 规划帧因此是 `world`，
# 和 Gazebo 世界帧、spawn/读位姿同一个系。所以这里所有坐标都按 `world` 帧给（不是 base_link）。
# 机械臂 base 在 world 原点（底座落在安装底板顶 z=0.02），可达性从 base 算。
BOARD_FILES = 8                                       # 域常量：列 a-h
BOARD_RANKS = 8                                       # 域常量：行 1-8
# 格宽 / 板宽（v0.7 起格宽是主项 = Jeff 实测真棋盘 4.5cm；板宽默认派生 = 格宽×8）。
# 兼容旧用法（T0）：只设 GZCHESS_BOARD_SIZE_M 时行为与 v0.4-0.6 完全一致（格宽 = 板宽/8）；
# 两者都显式设置时各用各的（格区仍居中，边框相应变宽/变窄）。
_cell_env = os.getenv("GZCHESS_CELL_M")
_size_env = os.getenv("GZCHESS_BOARD_SIZE_M")
if _cell_env is not None:
    CELL_M = float(_cell_env)
    BOARD_SIZE_M = float(_size_env) if _size_env is not None else CELL_M * BOARD_FILES
elif _size_env is not None:
    BOARD_SIZE_M = float(_size_env)
    CELL_M = BOARD_SIZE_M / BOARD_FILES
else:
    CELL_M = 0.045                                    # Jeff 实测：真实棋盘格宽 4.5cm（2026-07-06）
    BOARD_SIZE_M = CELL_M * BOARD_FILES               # 派生：格区 0.36
BOARD_THICKNESS_M = _f("GZCHESS_BOARD_THICKNESS_M", "0.008")  # 棋盘底板厚度（坐在底板顶上）
# 印坐标的边框宽度：板总尺寸 = 格区(BOARD_SIZE_M) + 2×边框。格中心坐标不受边框影响（格区始终居中）。
BOARD_MARGIN_M = _f("GZCHESS_BOARD_MARGIN_M", "0.03")
# 贴图最终整体旋转（90° 一档，0-3）：补偿 gz box 顶面 UV 的取向差——金标准（/status 真值 ↔ 印刷坐标）
# 实测定档；换 gz 版本若 UV 约定变了改这一个数即可。
BOARD_TEX_QUARTER_TURNS = _i("GZCHESS_BOARD_TEX_QUARTER_TURNS", "1")
# 臂底座前缘在 world 的 x（米）。量法：episode1_urdf_1113/meshes/collision/base_link.STL 的包围盒
# x∈[-0.076,+0.076]（离线解析，2026-07-06），base_link 焊在 world 原点正上方 → 前缘 x=+0.076。
# 不参与派生，仅作自检/展示（底座外缘到板边的实际净空 = GAP − 此值）。
ARM_BASE_FRONT_X_M = _f("GZCHESS_ARM_BASE_FRONT_X_M", "0.076")
# 臂基座轴心在 world 的 xy（径向倾斜抓取的方位角原点；episode 孪生 base_link 就在原点）。
ARM_BASE_XY = tuple(float(v) for v in os.getenv("GZCHESS_ARM_BASE_XY", "0,0").split(","))
# 底座**轴心** → 棋盘物理近边（含印刷边框）的距离。口径 = Jeff 2026-07-06 拍板的方案 A：
# 10cm 从轴心量起（底座外缘到板边实际 ≈2.4cm）。曾试过「前缘量 10cm」——h 列离轴心 0.544-0.566m，
# 超出孪生臂全姿态抓取半径上限 0.53m（0-90° 径向倾斜逐档实测），64 格闸门不过，方案见 v0.7 开发日志。
ARM_BOARD_GAP_M = _f("GZCHESS_ARM_BOARD_GAP_M", "0.10")
# 棋盘中心在 world 里的位姿。默认按真实摆位派生：轴心 + 间距 + 边框 + 半幅格区（默认 0.31）；
# 直接设 GZCHESS_BOARD_ORIGIN_X 可整体覆盖（旧用法保留，T0）。
BOARD_ORIGIN_X = _f("GZCHESS_BOARD_ORIGIN_X",
                    str(ARM_BASE_XY[0] + ARM_BOARD_GAP_M + BOARD_MARGIN_M + BOARD_SIZE_M / 2))
BOARD_ORIGIN_Y = _f("GZCHESS_BOARD_ORIGIN_Y", "0.0")
# 棋盘上表面 z（= 棋子底面所在高度）。
# ⚠️ 实测：episode 的 40×80 安装底板(mount_plate)顶在 z=0.02，比薄棋盘高，会把平铺在 z=0.008 的棋盘**挡住**、
# 棋子还会穿过薄板落到底板上。所以把棋盘**架在底板顶上**：板底=底板顶(0.02)，板面=0.02+厚度(0.008)=0.028。
BOARD_ORIGIN_Z = _f("GZCHESS_BOARD_ORIGIN_Z", "0.028")
BOARD_YAW_RAD = _f("GZCHESS_BOARD_YAW_RAD", "0.0")    # 棋盘绕 z 转角：a→h 方向相对 world +x 的夹角

# ---- 棋子外观网格（v0.7：真实斯汤顿造型，仅视觉；碰撞体仍是三段圆柱，物理零重调）----
# 目录里要有 manifest.json + 六个 STL（来源/许可见 models/meshes/SOURCE.md）。
# 设为空串 "" → 完整回退 v0.5 的几何剪影（T0）；单个子型文件缺失也只该型回退。
PIECE_MESH_DIR = os.getenv("GZCHESS_PIECE_MESH_DIR",
                           str(Path(__file__).resolve().parent / "models" / "meshes"))

# ---- 棋子尺寸（米）----
# 注意夹爪可夹宽度区间见下（GRIP_*）：抓取点宽度必须落在该区间内，太细夹不住、太粗夹不下。
PIECE_BASE_DIAM_M = _f("GZCHESS_PIECE_BASE_DIAM_M", "0.030")     # 底座直径 ~3cm
PIECE_HEIGHT_M = _f("GZCHESS_PIECE_HEIGHT_M", "0.045")          # 高 ~4.5cm（兵）
PIECE_GRASP_WAIST_M = _f("GZCHESS_PIECE_GRASP_WAIST_M", "0.020")  # 抓取点离棋子底的高度（夹"腰"）
PIECE_GRASP_WIDTH_M = _f("GZCHESS_PIECE_GRASP_WIDTH_M", "0.035")  # 抓取点处棋子宽度（要落在夹爪可夹区间）

# ---- 弃子区 / 备用子区（棋盘外，机械臂真夹真放的落点/取点；world 帧，须落在臂可达范围内）----
# remove(square)：把吃掉的子从盘上夹起，放到【弃子区】的下一个空槽（在棋盘一侧排开）。
# place(square,piece)：在【备用子区】spawn 一枚该色的子，再夹起来摆到目标格（模拟"从备用盒取子摆上盘"）。
# ⚠️ 下面坐标是首版默认（env 可覆盖）；实际可达性/不压盘要对着仿真在 W6 里校准（占位登记，见 world.py 注释）。
# 弃子槽从靠近臂的一端排起（x 小→大）：臂展上限实测 ≈0.44m（h 列/远槽 IK 不可达），
# 槽位半径必须 ≤ 上限。首槽 x=0.10、每排 6 槽（x 至 0.30，臂距 ≤0.40 全可达）。
DISCARD_ORIGIN_X = _f("GZCHESS_DISCARD_ORIGIN_X", str(BOARD_ORIGIN_X - 0.18))  # 弃子区第 0 槽的 x
DISCARD_ORIGIN_Y = _f("GZCHESS_DISCARD_ORIGIN_Y", "0.27")                     # 板半幅=格区0.20+边框0.03=0.23，再留子底座半径+间隙
DISCARD_PITCH_M = _f("GZCHESS_DISCARD_PITCH_M", "0.04")                       # 相邻弃子槽间距
DISCARD_SLOTS_PER_ROW = _i("GZCHESS_DISCARD_SLOTS_PER_ROW", "6")             # 每排几个槽，满了换下一排（沿 y 外扩；排满 2 排后更远的槽可能超臂展）
RESERVOIR_ORIGIN_X = _f("GZCHESS_RESERVOIR_ORIGIN_X", str(BOARD_ORIGIN_X))    # 备用子区取子点 x
RESERVOIR_ORIGIN_Y = _f("GZCHESS_RESERVOIR_ORIGIN_Y", "-0.27")               # 棋盘另一侧（和弃子区分开）；同样避开加宽后的板边
# 盘外支撑面高度：备用区/弃子区在棋盘外面，脚下是桌面(z=0)而不是棋盘面(BOARD_ORIGIN_Z)。
# 2026-07-03 实测抓出的老 bug：按棋盘面高度在盘外生成子 → 子掉到桌面、夹爪按高 28mm 去抓 → 必抓空。
OFFBOARD_SURFACE_Z = _f("GZCHESS_OFFBOARD_SURFACE_Z", "0.0")

# ---- 相机 ----
# 默认**双相机**（both = 斜视 oblique + 正俯视 overhead 同时开，各自独立话题——
# 多相机绝不共用话题：共用会让不同机位的帧交替混流，前端/大脑都没法用）。
# 单相机模式完整保留：GZCHESS_CAM_MODE=oblique / overhead 一键切（T0：加新不丢旧）。
CAM_MODE = os.getenv("GZCHESS_CAM_MODE", "both")             # "both" | "oblique"（斜上方）| "overhead"（正俯视）
# 每路相机的 gz 话题 = f"{CAM_TOPIC_BASE}/<相机名>/image"（相机名即模式名；bridge 要逐路桥到 ROS）
CAM_TOPIC_BASE = os.getenv("GZCHESS_CAM_TOPIC_BASE", "/gazebo_chess")


def cam_names() -> list[str]:
    """本模式下开哪几路相机（名字顺序 = observation 里图片的顺序）。"""
    return ["oblique", "overhead"] if CAM_MODE == "both" else [CAM_MODE]


def cam_topic(name: str) -> str:
    return f"{CAM_TOPIC_BASE}/{name}/image"


# 斜视位姿（板局部系）：从棋盘中心出发，方位角 AZIM（-90°=白方/rank1 一侧）、俯角 ELEV、直线距离 DIST。
# 默认值的账：俯角 50° 能看出子身高度差；距离 0.85m 时 40cm 棋盘在 720p 竖直视野内整盘可见。
CAM_OBL_AZIM_DEG = _f("GZCHESS_CAM_OBL_AZIM_DEG", "-90")
CAM_OBL_ELEV_DEG = _f("GZCHESS_CAM_OBL_ELEV_DEG", "50")
CAM_OBL_DIST_M = _f("GZCHESS_CAM_OBL_DIST_M", "0.85")
# 俯视（overhead 模式用）：棋盘上方高度。⚠️ 约束是**竖直** FOV（16:9 下 vfov≈0.6rad < hfov 1.0）：
# board 0.40m 要全进画面、竖直视野 2*h*tan(vfov/2) 得 ≥ 0.40+留边，故 h≈0.85m。
CAM_HEIGHT_M = _f("GZCHESS_CAM_HEIGHT_M", "0.85")
CAM_FOV_RAD = _f("GZCHESS_CAM_FOV_RAD", "1.0")     # 垂直视野（约 57°）
CAM_W = _i("GZCHESS_CAM_W", "1280")
CAM_H = _i("GZCHESS_CAM_H", "720")
CAM_FPS = _i("GZCHESS_CAM_FPS", "15")

# ---- 摆盘（v0.5 多子）----
# 非空 → 启动按 FEN 摆多子（摆放字段；v0.7 起也接受完整 FEN，轮次/易位权给裁判用）；
# 空 → 只摆一枚演示子（GZCHESS_DEMO_PIECE，v0.4 路径保留）。
SETUP_FEN = os.getenv("GZCHESS_SETUP_FEN", "")

# ---- 裁判 / 对局（v0.7）----
# 裁判 = 世界内部的棋规真值（referee.py）：原语动臂之前过合法闸、物理核实后才推进真值、终局落档。
# auto（默认）= 摆了 FEN 才开；单演示子模式自动关，v0.4-0.6 的「裸物理原语」行为原样保留（T0）。
REFEREE_MODE = os.getenv("GZCHESS_REFEREE", "auto")            # auto / on / off


def referee_enabled() -> bool:
    if REFEREE_MODE == "on":
        return True
    if REFEREE_MODE == "off":
        return False
    return bool(SETUP_FEN.strip())


# 内置电脑对手走哪方（white/black/off）。裁判落档的 white/black 标签也据此定（非 bot 侧 = anima）。
# 对手走子=瞬移（直接改模型位姿，不用机械臂）；引擎是 gz_bot.py（第三份独立副本，禁与另两份合并）。
BOT_SIDE = os.getenv("GZCHESS_BOT_SIDE", "black")
BOT_DEPTH = _i("GZCHESS_BOT_DEPTH", "3")                       # 对手搜索深度（对齐 SIMCHESS 默认）
BOT_TIME = _f("GZCHESS_BOT_TIME", "2.0")                       # 对手单步思考秒数上限
# 对局棋谱落档目录（对齐 sim-chess：默认仓根 logs/，eval 记分台从这里读）。
GAMES_LOG_DIR = os.getenv("GZCHESS_GAMES_LOG_DIR") or str(
    Path(__file__).resolve().parents[2] / "logs")
# 弃子处理：bin（默认，v0.7）= 真夹真搬到固定「弃子袋」点，放稳后模型销毁（袋子吞掉——
# 一盘最多 30 次吃子，槽位摆开第 2 排起就超臂展、还会互相碰撞）；slots = v0.5 槽位摆开行为原样（T0）。
DISCARD_MODE = os.getenv("GZCHESS_DISCARD_MODE", "bin")
DISCARD_BIN_XY = tuple(float(v) for v in os.getenv(
    "GZCHESS_DISCARD_BIN_XY", f"{DISCARD_ORIGIN_X},{DISCARD_ORIGIN_Y}").split(","))
# 终局后自动复位（默认 off：让人看完终局盘面自己在网页按「开新局」；on 则终局几秒后自动重摆）。
AUTO_RESET = os.getenv("GZCHESS_AUTO_RESET", "off") == "on"
AUTO_RESET_DELAY_S = _f("GZCHESS_AUTO_RESET_DELAY_S", "5.0")

# ---- 失败注入（v0.5：测大脑补救链路用；默认全关，绝不影响正常运行）----
# 语义：off=关 / once=注入一次后自动归 off / always=每次都注入。
FAIL_GRIP_MISS = os.getenv("GZCHESS_FAIL_GRIP_MISS", "off")        # 夹空：走完动作但不闭爪（子留原地）
FAIL_PLACE_MODE = os.getenv("GZCHESS_FAIL_PLACE_MODE", "off")      # 放偏：放置点注入偏移
FAIL_PLACE_OFFSET_M = _f("GZCHESS_FAIL_PLACE_OFFSET_M", "0.05")    # 放偏量（默认一格=5cm，落到邻格）

# ---- 合成数据管线（scripts/gen_dataset.py：CNN 训练数据，世界真值自动打标签）----
DATASET_OUT_DIR = os.getenv("GZCHESS_DATASET_OUT_DIR", "")   # 空→脚本默认仓外 ~/gzchess-dataset
DATASET_N = _i("GZCHESS_DATASET_N", "300")                   # 采多少帧（ChessCog 量级是 5000，先小跑验管线）
DATASET_MIN_PIECES = _i("GZCHESS_DATASET_MIN_PIECES", "2")   # 每帧随机摆几个子（下限/上限）
DATASET_MAX_PIECES = _i("GZCHESS_DATASET_MAX_PIECES", "16")
DATASET_SETTLE_S = _f("GZCHESS_DATASET_SETTLE_S", "0.8")     # 摆完等物理/画面稳定再抓帧

# ---- 夹爪（来自 episode 夹爪 xacro 的固有几何，域常量；这里只记录、便于算夹持目标）----
# 手指为 prismatic，joint∈[0, GRIP_STROKE]；joint=0 闭合、=STROKE 张开。
# 两指内面"面对面"间距 = GRIP_FACE_GAP_CLOSED + 2*joint。
#   GRIP_FACE_GAP_CLOSED = 2*(finger_gap0 - finger_y/2) = 2*(0.018-0.005) = 0.026（2.6cm，能夹的最小宽度）
#   全开（joint=STROKE=0.022）= 0.026 + 0.044 = 0.070（7cm，能夹的最大宽度）
# 所以**可夹宽度区间 ≈ [0.026, 0.070] m**。棋子抓取点宽度必须落这里面。
GRIP_FACE_GAP_CLOSED_M = 0.026
GRIP_STROKE_M = 0.022

# ---- 抓取动作（米 / 弧度）----
APPROACH_SAFE_M = _f("GZCHESS_APPROACH_SAFE_M", "0.10")   # 目标格上方安全高度（抬起/接近用）
# 夹住时每根手指的目标位置（joint 值）；据抓取点宽度算：joint=(W - GRIP_FACE_GAP_CLOSED)/2 再留点挤压余量。
# 默认按 PIECE_GRASP_WIDTH=0.035 → (0.035-0.026)/2≈0.0045，挤压到 0.003。可被 env 直接覆盖。
GRIP_CLOSE_M = _f("GZCHESS_GRIP_CLOSE_M", "0.003")
# 张开度的账（v0.5 实测撞邻子后收窄）：指尖跨度=±(GRIP_FACE_GAP_CLOSED+2*OPEN)/2。
# 0.020 全开→±3.3cm，邻格子身表面只有 5-1.75=3.25cm 远→下探蹭到邻子（e2 旁的 e1 王被扫飞）。
# 0.012 →±2.5cm：不出本格（半格 2.5cm），而开口 5.0cm 仍远大于腰宽 3.5cm。
GRIP_OPEN_M = _f("GZCHESS_GRIP_OPEN_M", "0.012")          # 张开放子/接近时的手指位置
# 夹爪「到位」判定容差（米，对 /joint_states 实测指位；v0.7 夹爪改 topic+状态核实后使用）
GRIP_POS_TOL_M = _f("GZCHESS_GRIP_POS_TOL_M", "0.002")
PLACE_TOLERANCE_M = _f("GZCHESS_PLACE_TOLERANCE_M", "0.015")  # 落子到位容差 1.5cm

# ---- 找抓取姿态的备用自由度（够不着/会撞时逐档试）----
# 接近方向从竖直往外偏的档（度，v0.7 起为**径向**倾斜：朝远离基座方向倒）；末端 joint6 手腕自转的档（度）。
# 60/75 两档是 v0.7 新增：4.5cm 格 + 10cm 间隔后最远角格离基座 ≈0.57m，竖直取半径（≈0.44m）够不到，
# 大倾角（接近水平）才有解——对应真臂「伸直去取」的姿态。旧档全保留（T0）。
APPROACH_TILT_DEG = _deg_list("GZCHESS_APPROACH_TILT_DEG", "0,15,30,45,60,75")
WRIST_ROLL_DEG = _deg_list("GZCHESS_WRIST_ROLL_DEG", "0,45,90,-45,-90")

# ---- 动作阶段预算 / 节奏（秒）——每个环节的等待上限，超了就如实报失败，绝不无限挂 ----
# 背景（v0.5 wave 0）：一次 move 是长动作（几十秒），大脑靠 MCP progress「生命迹象」判活；
# 世界侧的责任是每个阶段自己有预算、卡死快败，liveness 只是最后防线，不是常规失败路径。
IK_TIMEOUT_S = _f("GZCHESS_IK_TIMEOUT_S", "1.0")            # MoveIt /compute_ik 的求解预算
# IKFast 假解防线的 FK 复核容差（米）。原来写死 0.02——太松：指全开跨度 5cm、棋子腰宽 3.5cm，
# 抓取点横向偏差 >(5-3.5)/2=0.75cm 指尖就会闭空；个别位姿的 IKFast 解带着 1-2cm 系统性偏差
# 通过复核，同一位姿每次选同一解 → 该格「每抓必空」（2026-07-06 两盘耐力跑在 a2/f1 复现实锤）。
# 收紧到 0.005：超差的解按不可达处理，候选自然落到下一档（如径向倾斜），那些档的解是准的。
IK_FK_TOL_M = _f("GZCHESS_IK_FK_TOL_M", "0.005")
IK_WAIT_EXTRA_S = _f("GZCHESS_IK_WAIT_EXTRA_S", "2.0")      # 等 IK 服务应答的额外余量（网络/调度）
FK_WAIT_S = _f("GZCHESS_FK_WAIT_S", "3.0")                  # FK 复核（IKFast 假解防线）的等待上限
TRAJ_ACCEPT_S = _f("GZCHESS_TRAJ_ACCEPT_S", "10.0")         # 轨迹 goal 被控制器接受的等待上限
TRAJ_EXTRA_S = _f("GZCHESS_TRAJ_EXTRA_S", "8.0")            # 轨迹执行完成的额外余量（叠在轨迹时长上）
MOVE_TIME_APPROACH_S = _f("GZCHESS_MOVE_TIME_APPROACH_S", "3.0")  # 大段移动（到接近点/回驻位）的轨迹时长
MOVE_TIME_SHORT_S = _f("GZCHESS_MOVE_TIME_SHORT_S", "2.0")  # 短段移动（下抓/抬起）的轨迹时长
GRIP_TIME_S = _f("GZCHESS_GRIP_TIME_S", "1.0")              # 夹爪开合的轨迹时长
GRIP_SETTLE_S = _f("GZCHESS_GRIP_SETTLE_S", "0.3")          # 夹爪动作后的短静置（等物理接触稳定）
SETTLE_S = _f("GZCHESS_SETTLE_S", "1.0")                    # 放子后静置再核对落点（等物理稳定）
SPAWN_SETTLE_S = _f("GZCHESS_SPAWN_SETTLE_S", "0.6")        # spawn 新子后静置（等实体落稳）
READY_TIMEOUT_S = _f("GZCHESS_READY_TIMEOUT_S", "20.0")     # 启动时等 MoveIt/控制器就绪的上限
SPIN_STEP_S = _f("GZCHESS_SPIN_STEP_S", "0.05")             # 每次 rclpy spin_once 的步长
STATUS_POSE_WINDOW_S = _f("GZCHESS_STATUS_POSE_WINDOW_S", "0.8")  # /status 读位姿的采样窗（调试台要快）
PIECE_MATCH_CELL_FRAC = _f("GZCHESS_PIECE_MATCH_CELL_FRAC", "0.6")  # 「格上有子」判定半径 = CELL_M × 此系数

# ---- ROS / Gazebo 接口名（可配，别在代码里散落写死）----
ARM_GROUP = os.getenv("GZCHESS_ARM_GROUP", "episode_arm")               # MoveIt 规划组(episode1_urdf_1113_moveit SRDF: base_link→link6)
PLANNING_FRAME = os.getenv("GZCHESS_PLANNING_FRAME", "world")           # MoveIt 规划帧（孪生加了 world 链接）
EEF_LINK = os.getenv("GZCHESS_EEF_LINK", "link6")                       # 给 MoveIt 下位姿目标的末端链接
# 末端 link6 原点 → 两指中间抓取点(TCP) 沿夹爪轴的距离（米）。竖直抓时 = link6 下方这么多。
# 约等于 mount(0.012)+base_z(0.024)+finger_z/2(0.0225) ≈ 0.058；先给默认，明天对着仿真校准。
TCP_OFFSET_M = _f("GZCHESS_TCP_OFFSET_M", "0.058")
GZ_WORLD_NAME = os.getenv("GZCHESS_GZ_WORLD", "episode_world")          # Gazebo 世界名
ARM_CONTROLLER = os.getenv("GZCHESS_ARM_CONTROLLER", "episode_arm_controller")
GRIPPER_CONTROLLER = os.getenv("GZCHESS_GRIPPER_CONTROLLER", "gripper_controller")
# 俯视相机的朝向（rpy 弧度）：默认 pitch=+90° 让相机 +x 轴朝下(-z)拍。图像上下/左右朝向需对着仿真确认。
CAM_RPY = [float(x) for x in os.getenv("GZCHESS_CAM_RPY", "0,1.5708,1.5708").split(",")]

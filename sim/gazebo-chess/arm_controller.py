"""机械臂抓放控制器（rclpy）：给 MoveIt 算 IK、用关节轨迹控制器执行、开合夹爪。

v0.4 走法（务实、可靠）：用 MoveIt `/compute_ik` 把"末端到某位姿"解成关节角，再用
`episode_arm_controller` 的 FollowJointTrajectory 执行；夹爪用 `gripper_controller` 同理。
（full move_action 规划+避障更稳，但目标构造复杂；0.4 一个子、开阔棋盘，IK+轨迹够用，且我已验证轨迹可执行。
 桌面/棋子避障靠把接近点抬高 + 抓取点在板面上方，不往桌里扎。更强避障留 0.5 换 move_action。）

抓取 = 真实夹爪物理夹取（闭合到夹持宽度，靠接触摩擦夹住子），不贴关节。

线程模型（v0.5 wave 0）：本节点由世界的**专职 spin 线程**（world.py 起的 SingleThreadedExecutor）
持续 spin——本文件里**任何方法都不自己 spin**，等 ROS future 一律「挂完成事件 + 带超时等待」
（_wait_future）。背景：从请求工作线程做 rclpy spin 会和 DDS/executor 撞线程（实测卡死在
take_message 里 100 秒不返回），spin 必须收敛到唯一一个专职线程（ROS 惯例）。
"""
from __future__ import annotations

import os
import threading
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import config
import grasp_pose

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
GRIPPER_JOINTS = ["left_finger_joint", "right_finger_joint"]


class ArmController(Node):
    def __init__(self) -> None:
        # 节点名带 PID：世界服务、reach_map、各探针都会各自实例化本节点——短命进程复用同一个
        # 节点名会留下 DDS「幽灵」，新进程的 service/action 应答被路由给死节点（实锤两次：
        # IK 全超时被误判不可达 / 臂轨迹 goal 石沉大海而 CLI 正常）。ROS2 要求图内节点名唯一。
        super().__init__(f"gazebo_chess_arm_{os.getpid()}")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self._arm = ActionClient(self, FollowJointTrajectory, f"/{config.ARM_CONTROLLER}/follow_joint_trajectory")
        # 夹爪走 **topic 发布**（JointTrajectoryController 原生订阅 ~/joint_trajectory），不用 action：
        # 2026-07-06 整盘耐力跑实锤——长命进程跑到 ~30 手后，夹爪 action 客户端单独瘫痪
        # （goal 应答不再送达；同节点的臂 action 客户端还活着、CLI 也正常），闭爪静默失效被误判成
        # 连环「夹空」。pub/sub 没有 service 应答链路可瘫；到没到位改由 /joint_states 实测核实。
        self._grip_pub = self.create_publisher(JointTrajectory,
                                               f"/{config.GRIPPER_CONTROLLER}/joint_trajectory", 10)
        self._ik = self.create_client(GetPositionIK, "/compute_ik")
        self._fk = self.create_client(GetPositionFK, "/compute_fk")
        self._js: JointState | None = None
        self.ik_no_reply = 0        # IK 服务「超时无应答」累计（≠ 无解；见 compute_ik）
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

    # ---------- 基础 ----------
    def _on_js(self, msg: JointState) -> None:
        self._js = msg

    def _wait_future(self, fut, timeout_s: float):
        """等一个 rclpy future 完成（完成由专职 spin 线程驱动），超时返回 None——绝不自己 spin。"""
        done = threading.Event()
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout_s):
            return None
        return fut.result()

    def wait_ready(self, timeout: float = 15.0) -> bool:
        ok = (self._arm.wait_for_server(timeout_sec=timeout)
              and self._ik.wait_for_service(timeout_sec=timeout))
        self._fk.wait_for_service(timeout_sec=timeout)   # FK 用于复核 IK 解，不强制（缺了退化为信任 IK）
        t0 = time.time()
        while self._js is None and time.time() - t0 < timeout:   # /joint_states 由专职 spin 线程送达
            time.sleep(config.SPIN_STEP_S)
        return ok and self._js is not None

    def current_arm_positions(self) -> dict[str, float]:
        if self._js is None:
            return {}
        return {n: p for n, p in zip(self._js.name, self._js.position)}

    # ---------- IK ----------
    def compute_ik(self, pose: grasp_pose.Pose, timeout_s: float = config.IK_TIMEOUT_S) -> list[float] | None:
        """把 link6 的目标位姿（world 帧）解成 6 个臂关节角；解不出返回 None。
        无解 vs 服务无应答是两种失败：后者记在 self.ik_no_reply 计数上（调用方据此把
        「基础设施没应答」和「运动学不可达」报成不同的错——混为一谈会把补救方向带偏）。"""
        (px, py, pz), (qx, qy, qz, qw) = pose
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = config.ARM_GROUP
        r.ik_link_name = config.EEF_LINK
        r.avoid_collisions = True
        r.timeout = Duration(sec=int(timeout_s), nanosec=int((timeout_s % 1) * 1e9))
        # 用当前关节作种子（提高成功率、保持解连续）
        rs = RobotState()
        if self._js is not None:
            rs.joint_state = self._js
        r.robot_state = rs
        ps = PoseStamped()
        ps.header.frame_id = config.PLANNING_FRAME
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = px, py, pz
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = qx, qy, qz, qw
        r.pose_stamped = ps
        res = self._wait_future(self._ik.call_async(req), timeout_s + config.IK_WAIT_EXTRA_S)
        if res is None:                              # 服务超时没应答（≠ 无解）：单独计数
            self.ik_no_reply += 1
            return None
        if res.error_code.val != 1:                  # 1 = SUCCESS；其余 = 真·无解/求解失败
            return None
        sol = {n: p for n, p in zip(res.solution.joint_state.name, res.solution.joint_state.position)}
        if not all(j in sol for j in ARM_JOINTS):
            return None
        joints = [sol[j] for j in ARM_JOINTS]
        # ⚠️ 实测：这台臂的 IKFast 插件会对**够不着的位姿也返回 error_code=SUCCESS + 一个不匹配的解**。
        # 所以必须用 FK 复核：解出来的关节 FK 回 link6，和请求位姿差超容差就判不可达。
        # 容差见 config.IK_FK_TOL_M（原写死 0.02 导致个别位姿带 1-2cm 偏差的解通过、该格每抓必空）。
        fk = self._fk_link6(joints)
        if fk is None:
            return joints   # FK 服务不可用时退化为信任 IK（至少别更糟）
        tol = config.IK_FK_TOL_M
        if (abs(fk[0] - px) > tol or abs(fk[1] - py) > tol or abs(fk[2] - pz) > tol):
            return None
        return joints

    def _fk_link6(self, joints: list[float]) -> tuple[float, float, float] | None:
        """对一组臂关节角做正运动学，返回 link6 在规划帧的 (x,y,z)。"""
        if not self._fk.service_is_ready():
            return None
        req = GetPositionFK.Request()
        req.header.frame_id = config.PLANNING_FRAME
        req.fk_link_names = [config.EEF_LINK]
        js = JointState()
        js.name = list(ARM_JOINTS)
        js.position = [float(v) for v in joints]
        req.robot_state.joint_state = js
        res = self._wait_future(self._fk.call_async(req), config.FK_WAIT_S)
        if res is None or res.error_code.val != 1 or not res.pose_stamped:
            return None
        p = res.pose_stamped[0].pose.position
        return (p.x, p.y, p.z)

    # ---------- 执行 ----------
    def _send_traj(self, client: ActionClient, joints: list[str], positions: list[float],
                   duration_s: float) -> bool:
        jt = JointTrajectory(joint_names=joints)
        pt = JointTrajectoryPoint(positions=[float(p) for p in positions])
        pt.time_from_start = Duration(sec=int(duration_s), nanosec=int((duration_s % 1) * 1e9))
        jt.points = [pt]
        goal = FollowJointTrajectory.Goal(trajectory=jt)
        gh = self._wait_future(client.send_goal_async(goal), config.TRAJ_ACCEPT_S)
        if gh is None or not gh.accepted:
            return False
        return self._wait_future(gh.get_result_async(), duration_s + config.TRAJ_EXTRA_S) is not None

    def goto_arm(self, positions: list[float], duration_s: float = config.MOVE_TIME_APPROACH_S) -> bool:
        return self._send_traj(self._arm, ARM_JOINTS, positions, duration_s)

    def gripper_positions(self) -> list[float]:
        """两指当前位置（/joint_states 实测；没数据返回空表）。"""
        pos = self.current_arm_positions()
        return [pos[j] for j in GRIPPER_JOINTS if j in pos]

    def set_gripper(self, finger_pos: float, duration_s: float = config.GRIP_TIME_S,
                    verify_reach: bool = True) -> bool:
        """夹爪到指定指位：topic 发布轨迹（见 __init__ 注释——不用 action），完成判定走
        /joint_states 实测。verify_reach=True（张开用）：等两指真到位，超时如实报 False；
        =False（闭合夹子用）：手指会被棋子挡停、到不了目标位是**预期**，只等轨迹时间走完，
        夹没夹住交给下游的位移核实判断（最终判据是物理状态，不是指令 ack）。"""
        jt = JointTrajectory(joint_names=list(GRIPPER_JOINTS))
        pt = JointTrajectoryPoint(positions=[float(finger_pos)] * 2)
        pt.time_from_start = Duration(sec=int(duration_s), nanosec=int((duration_s % 1) * 1e9))
        jt.points = [pt]
        self._grip_pub.publish(jt)
        if not verify_reach:
            time.sleep(duration_s + config.GRIP_SETTLE_S)
            return True
        deadline = time.time() + duration_s + config.TRAJ_EXTRA_S
        while time.time() < deadline:
            got = self.gripper_positions()
            if len(got) == 2 and all(abs(p - finger_pos) <= config.GRIP_POS_TOL_M for p in got):
                return True
            time.sleep(config.SPIN_STEP_S)
        return False

    def open_gripper(self) -> bool:
        return self.set_gripper(config.GRIP_OPEN_M, verify_reach=True)

    def close_gripper(self) -> bool:
        return self.set_gripper(config.GRIP_CLOSE_M, verify_reach=False)

    # ---------- 抓 / 放 ----------
    def _solve_candidate(self, approach: grasp_pose.Pose, grasp: grasp_pose.Pose):
        ja = self.compute_ik(approach)
        if ja is None:
            return None
        jg = self.compute_ik(grasp)
        if jg is None:
            return None
        return ja, jg

    def pick_at(self, px: float, py: float, pz: float, progress=None,
                inject_miss: bool = False, avoid_xy=None,
                rotate_candidates: int = 0) -> tuple[bool, str]:
        """在世界点 (px,py,pz) 抓一个子：选一个 IK 可达候选 → 开爪→到接近点→下到抓取点→闭爪→抬回接近点。

        progress: 可选的进度上报回调 `progress(message: str)`——每个候选/关键子步各报一句人话，
        让上层（MCP progress → 大脑/仪表盘）看到"臂正在干什么"而不是黑等。不传则静默（行为不变）。
        inject_miss: 失败注入（测补救链路）：动作全走、但**不闭爪**——物理后果=夹空，子留在原地。
        rotate_candidates: 重试多样性（v0.7）——同一目标连续物理失败时由 world 递增传入，
        把候选清单整轮轮换 N 位（首选换人、总集不减）。背景：个别位姿的解带系统性偏差、
        「同场景同候选同解」= 每抓必空，2026-07-06 三次实锤（a2/g1），确定性失手重试一万次也没用，
        重试必须**换姿态**才独立（v1.1「重试独立性」原则的物理层落地）。"""
        note = progress or (lambda m: None)
        no_reply0 = self.ik_no_reply
        cands = grasp_pose.candidates_for_point(px, py, pz, avoid_xy=avoid_xy)
        if rotate_candidates:   # 重试多样性：整轮轮换起点（不减候选，只换首选）——见 world 的重试计数
            r = rotate_candidates % len(cands)
            cands = cands[r:] + cands[:r]
        for label, approach, grasp in cands:
            note(f"IK 求解抓取姿态（候选 {label}）")
            sol = self._solve_candidate(approach, grasp)
            if sol is None:
                continue
            ja, jg = sol
            note(f"抓取姿态可达（{label}），移向接近点")
            if not self.open_gripper():   # 状态核实没到位=控制器/通信问题，如实定性（≠抓取失败）
                return False, "夹爪没张开（张开指令未生效——控制器/通信问题，不是够不着）"
            if not self.goto_arm(ja, config.MOVE_TIME_APPROACH_S):
                return False, f"到接近点失败({label})"
            note("下探到抓取点")
            if not self.goto_arm(jg, config.MOVE_TIME_SHORT_S):
                return False, f"下到抓取点失败({label})"
            if inject_miss:
                note("（失败注入：不闭爪，模拟夹空）")
            else:
                note("闭合夹爪")
                self.close_gripper()
            time.sleep(config.GRIP_SETTLE_S)
            if not self.goto_arm(ja, config.MOVE_TIME_SHORT_S):
                return False, f"抬起失败({label})"
            return True, f"抓取动作完成({label})"
        return False, self._all_ik_failed_msg(no_reply0)

    def _all_ik_failed_msg(self, no_reply_before: int) -> str:
        """全候选失败时定性：全是「服务超时无应答」= 基础设施问题（MoveIt/DDS），
        不是「够不着」——两种失败混为一谈会把补救方向带偏（实锤：进程快速重启后 DDS 应答丢失，
        30 个候选全超时，被误报成不可达）。"""
        n = self.ik_no_reply - no_reply_before
        if n > 0:
            return (f"IK 服务无应答（{n} 次请求超时）——这是 MoveIt/通信问题，不是够不着；"
                    f"查 move_group 是否活着、或重启世界服务再试")
        return "所有候选姿态都 IK 不可达"

    def place_at(self, px: float, py: float, pz: float, progress=None,
                 avoid_xy=None, rotate_candidates: int = 0) -> tuple[bool, str]:
        """在世界点放下：到接近点→下到放置点→开爪→抬回接近点。参数语义同 pick_at。"""
        note = progress or (lambda m: None)
        no_reply0 = self.ik_no_reply
        cands = grasp_pose.candidates_for_point(px, py, pz, avoid_xy=avoid_xy)
        if rotate_candidates:
            r = rotate_candidates % len(cands)
            cands = cands[r:] + cands[:r]
        for label, approach, grasp in cands:
            note(f"IK 求解放置姿态（候选 {label}）")
            sol = self._solve_candidate(approach, grasp)
            if sol is None:
                continue
            ja, jg = sol
            note(f"放置姿态可达（{label}），移向放置接近点")
            if not self.goto_arm(ja, config.MOVE_TIME_APPROACH_S):
                return False, f"到放置接近点失败({label})"
            note("下探到放置点")
            if not self.goto_arm(jg, config.MOVE_TIME_SHORT_S):
                return False, f"下到放置点失败({label})"
            note("张开夹爪放子")
            if not self.open_gripper():   # 张不开=子可能还夹在手里，如实定性为基础设施问题
                return False, "放置时夹爪没张开（指令未生效——控制器/通信问题；子可能还夹着）"
            time.sleep(config.GRIP_SETTLE_S)
            if not self.goto_arm(ja, config.MOVE_TIME_SHORT_S):
                return False, f"放后抬起失败({label})"
            return True, f"放置动作完成({label})"
        return False, "放置点" + self._all_ik_failed_msg(no_reply0)

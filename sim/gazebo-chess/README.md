# gazebo-chess 世界（ANIMA Zero v0.7）

sim-chess 那张棋桌的 **Gazebo 3D 物理版**：真实建模的 episode 六轴机械臂，用**真实夹爪**把棋子从一格夹起、挪到另一格。对大脑（ANIMA）只露标准 MCP 接口（和 sim-chess 同款），世界内部把 ROS2 + MoveIt + Gazebo 这一摊全包起来。

> v0.4 跑通最小 infra（单子 + 手动遥控）；v0.5 长成完整形态（斜视相机、多子摆盘、长动作
> MCP progress、失败注入/自检/补救）；**v0.7 能下完整盘棋**：几何对齐真实摆位（4.5cm 格、
> 底座轴心到板边 10cm）、径向倾斜抓取 64 格全可达、世界内置裁判 + 瞬移电脑对手、吃子落袋、
> 终局落档给 eval 评分、网页「开新局」、真实斯汤顿棋子外观（CC-BY 4.0 网格，碰撞体不变）。

## 它和大脑怎么对话（标准 MCP，挂在 `/mcp`）

- `tools/list` + `tools/call` —— 三个物理原语：`move`(裸搬)/`remove`(夹走拿出棋盘)/`place`(备用区取子摆盘)；
  长动作边执行边发 `notifications/progress`（人话阶段：定位→抓→搬→放→核对）。
- **裁判（v0.7，`referee.py`，对局模式=摆了 FEN 才开）**：三原语动臂之前先过前置合法闸——非法直接
  拒绝并给人话原因（臂不动，省 26s/次）；一手棋按标准拆解表（吃子=remove→move、过路兵=remove(被吃
  兵格)→move、易位=王先车后两次 move、升变=move→remove→place）逐原语核对，**全部物理核实后真值
  才推进**；物理失败只登记修复上下文（修复须目标格一致**且是同一颗子**——源格=世界报的实际落格）；
  子永久掉出棋盘时支持**备用子恢复**（place 同款子补回真值所在格，盘面与棋局记录重新对齐，
  该走的那手照旧要走——下真棋掉了子也是这么办）。终局判定 + 棋谱落档
  （`logs/games-*.jsonl`，含 `world/white/black/physical_fails` 字段）都在世界内。
- **内置电脑对手（`gz_bot.py`，第三份独立引擎副本，禁与顾问/sim-chess 副本合并）**：大脑每凑完
  一手，对手立刻「瞬移」应手（set_pose/purge/spawn，不用机械臂）——**不播报走了哪步**，大脑下次
  感知看画面自己认（`GZCHESS_BOT_SIDE=white/black/off`）。
- `resources/read anima://observation` —— 给画面（相机帧）+ 空 state，**绝不给棋盘真值**。
- `prompts/get "guidance"` —— 世界说明书（注入大脑系统提示；含拆解规则与失败补救指引）。
- 带外普通 HTTP：`/health`（探活）/ `/status`（人类调试台真值+裁判局面，不给大脑）/
  `POST /reset`（人类侧开新局；**不进 MCP**——大脑不许重置现实）/ `/stream`（人看的视频）/ `/`（人类页，双相机+开新局按钮）。

## 它内部怎么跟仿真说话（ROS2 + MoveIt）

- 机械臂运动：MoveIt `/compute_ik`（+ FK 复核防 IKFast 假解）→ `FollowJointTrajectory` 执行；
  抓取候选按「指尖离邻子净空」排序（多子防撞）。
- **径向倾斜抓取（v0.7，`grasp_pose.py`）**：远格竖直够不着时，工具朝「远离基座」的方向倒
  15°–75°（方位角 = 基座→目标；link6 = 抓取点 − TCP_OFFSET·工具轴，接近/退出都沿工具轴）——
  臂的抓取半径从 0.44m（纯竖直）扩到 ≈0.53m，h 列因此可达。`scripts/reach_map.py` 一条命令
  出 64 格可达性地图（IK+FK 复核、不动臂；`--execute` 抽查真抓）。**重试多样性**：同一目标连续物理失败时自动轮换候选姿态起点（确定性失手的解重试多少次都一样，换姿态才是独立重试）。
- 夹爪：`gripper_controller`（真实闭合夹住子；张开度收窄到指尖不出本格）。
- ROS spin 收敛到**唯一专职线程**（请求线程只对 future 挂事件等待，绝不自己 spin——从请求线程
  spin 会和 DDS 撞线程卡死，v0.5 实测教训）。
- 往 Gazebo 塞棋盘/棋子/相机：`ros_gz_sim create`；读真值：pose 话题（只用于 /status 与执行自检）。
- 相机：Gazebo 相机（默认双路 oblique+overhead）→ `ros_gz_image image_bridge` → 订阅 → /perceive + /stream。
- 棋子外观：`models/meshes/` 六子真实斯汤顿网格（CC-BY 4.0，来源/许可/实测尺寸见
  `models/meshes/SOURCE.md`），运行时按 config 身高梯度派生缩放；**碰撞体仍是三段圆柱**
  （v0.4 验证的抓取物理零重调）；`GZCHESS_PIECE_MESH_DIR=""` 回退几何剪影。
- 离线工具：`scripts/gen_dataset.py`（合成训练数据，世界真值自动打标签）+ `scripts/train_cnn.py`
  （离线 torch 训练导出 ONNX，不进任何运行时）。

## 怎么起（前提：episode 仿真栈由用户亲手起）

```bash
# 终端1（用户亲手起 ROS 仿真栈）
ros2 launch episode1_gz_sim sim.launch.py headless:=true rviz:=false
# 终端1b（相机图桥，见项目 运行命令.md 二·4）
ros2 run ros_gz_image image_bridge /gazebo_chess/oblique/image /gazebo_chess/overhead/image
# 终端2（gazebo-chess 世界服务，:8106；下整盘棋摆标准开局，裁判+对手自动开）
cd .../anima-zero/world/gazebo-chess && source .venv/bin/activate && \
GZCHESS_SETUP_FEN="rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1" uvicorn server:app --port 8106
```

> venv 用 `python3 -m venv --system-site-packages .venv` 建，好 import 系统 ROS2。
> `GZCHESS_SETUP_FEN` 接受完整 FEN 或仅摆放字段；不摆 FEN = 单演示子模式（裁判/对手自动关，
> v0.4-0.6 裸物理行为原样保留）。

## 全部可调项

见 `config.py`（`GZCHESS_*` 环境变量，默认值集中在那里，禁硬编码）。v0.7 新增主要项：
`GZCHESS_CELL_M`（格宽，默认 0.045=Jeff 实测）、`GZCHESS_ARM_BOARD_GAP_M`（底座轴心→板边，默认
0.10）、`GZCHESS_REFEREE`（auto/on/off）、`GZCHESS_BOT_SIDE/DEPTH/TIME`、`GZCHESS_DISCARD_MODE`
（bin 弃子袋/slots 旧槽位）、`GZCHESS_PIECE_MESH_DIR`、`GZCHESS_AUTO_RESET`。

## 当前进度（v0.7）

- [x] `config.py`、`geometry.py`（坐标换算，已离线自测通过）
- [x] 棋子/棋盘/相机模型 + 往 Gazebo spawn（`spawn.py` / `models.py`）
- [x] 俯视相机出图（Gazebo 相机 → `ros_gz_image image_bridge` → JPEG）
- [x] `arm_controller.py`（MoveIt `/compute_ik` + FK 复核 + `FollowJointTrajectory`）、`grasp_pose.py`
- [x] `server.py` / `world.py` 接 MCP（`awi_mcp.py`，接口和 sim-chess 同款）
- [x] **teleop 手动遥控（`:8110`）**：人可顺畅点动这条臂，物理底座已验通（见 `~/episode-robot-dev-framework/episode-ros-ws` 的 `episode_teleop` + 项目 `运行命令.md`「三 · teleop」；套件 2026-07-07 已作为 infra 迁至 home 下）
- [x] **ANIMA 自主走子**（大脑发 `move` → 世界真跑一趟夹取搬运）——v0.5 修通（MCP progress，~26s/原语）
- [x] **多子摆盘**（`GZCHESS_SETUP_FEN`）+ 失败注入/执行自检分类/大脑补救——v0.5 活体验收通过
- [x] **几何对齐真实摆位 + 径向倾斜抓取 + 64 格全可达**（v0.7 wave0：reach_map 64/64 硬闸 +
      a1/h1/h8/e4 实抓 PASS，误差 0.1–0.2cm；h 列用 15–30° 倾斜真抓）
- [x] **裁判 + 真值推进 + 落档**（v0.7 wave1：离线 18 测试全绿）
- [x] **内置对手瞬移应手 + 弃子袋 + 复位**（v0.7 wave2：开局/吃子(过路兵)/易位/升变四场景活体全过）
- [x] **真实斯汤顿棋子外观**（v0.7 wave3：CC-BY 4.0 网格，仅视觉；满盘双相机截图人工核可辨六型）
- [x] **eval 记分**（v0.7 wave4：gazebo 对局进记分卡，分世界指标）
- [x] **整盘对局活体验收**（v0.7 wave5-7）：**两盘完整对局下到终局**（38 步 / 44 步，回合制——
      一句「我们来下棋，你用白的，你先走。」开局 + 每轮只说「我走完了。」；配合大脑侧的
      会话核心任务寄存器）；三类物理失败全部自然出现并被 LLM 自主补救；备用子恢复与
      重试多样性均有实战触发记录；4 盘长段落档 + 记分卡数据在 `logs/`。
- [ ] 扶正倒子、边际格根治、真机（C920 域适配）、VLA 策略换芯——后续版本

## 已知限制（实测数据，如实记录）

- **边际格的确定性失手**（两日 4 盘实测）：个别位姿（如 a2、g1 在满盘邻子下的首选候选）的 IK 解带
  系统性执行偏差 → 同姿态重试必然再失手。已用**重试多样性**兜住（换姿态一般 1-2 次即过，
  对局 8 实战验证）；根治（执行期跟踪误差诊断）留 v0.8。掉子/放偏世界如实报错并给自检分类。
- **角上夹取偶发掉子**（v0.5 实测 ~1/4 概率）：物理仿真的真实抖动；世界如实报错，不隐瞒、不兜底。
- **网格与碰撞体的视觉空隙**：夹取瞬间指尖与棋子表面有几毫米可见空隙（碰撞腰 35mm > 网格腰
  18–24mm，物理接触发生在不可见碰撞体上）——0.85m 相机距离不可辨，明细见 `models/meshes/SOURCE.md`。
- **DDS 长会话腐化（运维教训，2026-07-06 实锤）**：仿真栈被大量短命客户端进程 + 强杀反复折腾后，
  会出现「IK 服务全超时」或「臂轨迹 goal 石沉大海但 CLI 正常」——这是 DDS 图腐化不是代码问题，
  **重启仿真栈即愈**。世界侧已做两层防护：ArmController 节点名带 PID（防同名幽灵）、
  IK「服务无应答」与「运动学无解」分开报错（别把基础设施问题误读成够不着）。
- ~~h 列整列 IK 不可达~~（v0.5 遗留）——**v0.7 已修**：径向倾斜 + 方案 A 几何（轴心量 10cm），
  reach_map 实测 64/64 可达（h 列 15°、四角 30°）。

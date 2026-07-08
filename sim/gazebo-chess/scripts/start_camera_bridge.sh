#!/bin/bash
# 起相机桥（ros_gz_image image_bridge）+ 启动自检。
#
# 为什么要包这一层：image_bridge 是"哑巴"进程——起成功后一行日志都不打，人没法判断成没成。
# 本脚本替它报成活：起桥后轮询 ROS 节点表，看到 /ros_gz_image 上线才打 SUCCESSFULLY START；
# 桥进程中途退出/超时未上线则明确报错。之后前台等着桥（Ctrl+C 即停桥，行为和裸跑一致）。
#
# 前提：本终端已 source ROS2 + episode-ros-ws（见 运行命令.md 二·4 终端 A2）。
# 📌 2026-07-07 目录迁移 note（给其他 agent）：Episode 套件已作为机器人 infra 迁到 home 下
#（`~/episode-robot-dev-framework/`，与 ~/IsaacLab 平级）。本脚本不含其路径、逻辑不受影响；
#  source 路径见更新后的 运行命令.md（已外置成环境变量 $EPISODE_WS）。
# 用法：
#   ./start_camera_bridge.sh                 # 默认桥双相机两路话题（GZCHESS_CAM_TOPIC_BASE 可覆盖话题前缀）
#   ./start_camera_bridge.sh /某/话题 ...     # 显式给话题则只桥给定的（单相机模式用）
set -u

BASE="${GZCHESS_CAM_TOPIC_BASE:-/gazebo_chess}"   # 与 world/gazebo-chess/config.py 的默认一致
if [ "$#" -gt 0 ]; then
  TOPICS=("$@")
else
  TOPICS=("$BASE/oblique/image" "$BASE/overhead/image")
fi

WAIT_TRIES="${BRIDGE_CHECK_TRIES:-20}"            # 自检轮询次数（×0.5s；env 可覆盖）

if ! command -v ros2 >/dev/null 2>&1; then
  echo "❌ 找不到 ros2 —— 先 conda deactivate + source /opt/ros/jazzy/setup.bash（见 运行命令.md）"
  exit 1
fi

echo "起相机桥：${TOPICS[*]}"
ros2 run ros_gz_image image_bridge "${TOPICS[@]}" &
BRIDGE_PID=$!
trap 'kill "$BRIDGE_PID" 2>/dev/null' INT TERM

ok=0
for _ in $(seq 1 "$WAIT_TRIES"); do
  sleep 0.5
  if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
    echo "❌ 桥进程已退出（看上面的报错；常见原因：没 source episode-ros-ws、话题名拼错）"
    exit 1
  fi
  if ros2 node list 2>/dev/null | grep -q ros_gz_image; then
    ok=1
    break
  fi
done

if [ "$ok" = "1" ]; then
  echo "✅ SUCCESSFULLY START —— 相机桥已就绪（/ros_gz_image 在线，PID $BRIDGE_PID）"
  echo "   提示：有没有画面流过要等世界服务(:8106)起来后看：ros2 topic hz ${TOPICS[0]}  （应 ~15Hz）"
else
  echo "⚠️ 桥进程活着但 $((WAIT_TRIES / 2))s 内没在 ROS 节点表看到 /ros_gz_image ——"
  echo "   可能只是发现慢，用 ros2 node list | grep ros_gz_image 再核一次；一直没有就 Ctrl+C 重起。"
fi

wait "$BRIDGE_PID"

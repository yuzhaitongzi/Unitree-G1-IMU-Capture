#!/usr/bin/env bash
# 在 G1 机载电脑上录制用于 FAST-LIO2 的 bag
#
# 默认录制：LiDAR 点云 + G1 本体 IMU（经 g1_imu_bridge.py 桥接）
# 可选录制：机器人 LowState 原始话题、FAST-LIO2 实时输出 (/Odometry /tf)
#
# 用法示例：
#   LIDAR_TOPIC=/livox/lidar G1_IMU_TOPIC=/g1/imu ./record_g1_bag.sh
#   RECORD_FASTLIO_OUTPUT=1 ./record_g1_bag.sh        # 实时跑 FAST-LIO2 时
#   DURATION=120 ./record_g1_bag.sh                   # 录 120 秒后自动停
#
set -euo pipefail

# ----------------- 可调参数 -----------------
LIDAR_TOPIC="${LIDAR_TOPIC:-/livox/lidar}"
G1_IMU_TOPIC="${G1_IMU_TOPIC:-/g1/imu}"
ROBOT_STATE_TOPIC="${ROBOT_STATE_TOPIC:-}"        # 可选，例如 /lowstate 或 /lf/lowstate
RECORD_FASTLIO_OUTPUT="${RECORD_FASTLIO_OUTPUT:-0}"  # 1=同时录 /Odometry /tf
OUT_DIR="${OUT_DIR:-$HOME/g1_bags}"
TAG="${TAG:-g1}"
STORAGE="${STORAGE:-mcap}"                         # mcap | sqlite3
DURATION="${DURATION:-0}"                          # 0=手动 Ctrl-C 停止

# ----------------- 环境 -----------------
# 自动 source 一个 ROS2 发行版（按需修改）
for d in humble jazzy iron rolling foxy; do
  if [ -f "/opt/ros/$d/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "/opt/ros/$d/setup.bash"
    echo "[env] sourced /opt/ros/$d"
    break
  fi
done

command -v ros2 >/dev/null 2>&1 || {
  echo "[error] ros2 not found; 请先 source ROS2 环境" >&2
  exit 1
}

mkdir -p "$OUT_DIR"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
BAG="$OUT_DIR/${TAG}_${RUN_ID}"

# ----------------- 组装话题列表 -----------------
TOPICS=("$LIDAR_TOPIC" "$G1_IMU_TOPIC")
[ -n "$ROBOT_STATE_TOPIC" ] && TOPICS+=("$ROBOT_STATE_TOPIC")
if [ "$RECORD_FASTLIO_OUTPUT" = "1" ]; then
  TOPICS+=("/Odometry" "/tf" "/tf_static")
fi

echo "[bag] output : $BAG"
echo "[bag] storage: $STORAGE"
echo "[bag] topics : ${TOPICS[*]}"

# ----------------- 录前快检 -----------------
echo "[check] 话题是否存在 (若刚启动，看到 empty/报错可忽略) ..."
if command -v ros2 >/dev/null 2>&1; then
  ros2 topic list 2>/dev/null | grep -E "$LIDAR_TOPIC|$G1_IMU_TOPIC" || \
    echo "[warn] 未在 topic list 中看到目标话题，先确认驱动/桥接已启动"
fi

# ----------------- 录制 -----------------
BAG_CMD=(ros2 bag record -o "$BAG" --storage "$STORAGE" "${TOPICS[@]}")
if [ "$DURATION" != "0" ]; then
  # ros2 bag record 没有内置定时；用 timeout 到点发 SIGINT 让它正常收尾索引
  echo "[bag] 将录制 $DURATION 秒"
  timeout --signal=SIGINT "$DURATION" "${BAG_CMD[@]}" || true
else
  echo "[bag] 按 Ctrl-C 停止录制（会等待索引写完再退出）"
  set +e
  "${BAG_CMD[@]}"
  set -e
fi

# ----------------- 录后校验 -----------------
echo "[done] 基本信息："
ros2 bag info "$BAG" || true
echo
echo "[done] bag 路径：$BAG"
echo "[tip] 出现 '.active' 说明没正常收尾；先确认录制进程已退出，再对副本 ros2 bag reindex。"

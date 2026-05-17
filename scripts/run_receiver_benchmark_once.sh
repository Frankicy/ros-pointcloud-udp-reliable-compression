#!/usr/bin/env bash

# 不要在 source ROS setup 前使用 set -u，否则 COLCON_TRACE 未定义会报错
set -eo pipefail

MODE="${1:-unknown}"
PORT="${2:-9999}"

WS="/home/aiseon/udp_reliable_ros2_ws"
TS="$(date +%Y%m%d_%H%M%S)"

LOG_DIR="${WS}/benchmark_results/logs"
CSV_DIR="${WS}/benchmark_results/raw_csv"

mkdir -p "${LOG_DIR}" "${CSV_DIR}"

LOG_FILE="${LOG_DIR}/receiver_${MODE}_${TS}.log"
CSV_PREFIX="${CSV_DIR}/udp_codec_${MODE}.csv"

echo "=========================================="
echo "Receiver benchmark"
echo "MODE      : ${MODE}"
echo "PORT      : ${PORT}"
echo "LOG_FILE  : ${LOG_FILE}"
echo "CSV_PREFIX: ${CSV_PREFIX}"
echo "=========================================="

# 避免 conda 的 libtinfo / python / library 影响 ROS2
conda deactivate 2>/dev/null || true
conda deactivate 2>/dev/null || true

cd "${WS}"

# 先 source 系统 ROS2，再 source 当前 workspace
if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
else
  echo "ERROR: Cannot find /opt/ros/jazzy/setup.bash or /opt/ros/humble/setup.bash"
  exit 1
fi

# source ROS setup 时不要启用 set -u
set +u
source "${WS}/install/setup.bash"
set -u

ros2 run udp_reliable_receiver udp_reliable_receiver_node --ros-args \
  -p listen_port:="${PORT}" \
  -p output_topic:=/dense_pointcloud \
  -p output_csv:="${CSV_PREFIX}" 2>&1 | tee "${LOG_FILE}"

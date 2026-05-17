#!/usr/bin/env bash

set -eo pipefail

MODE="${1:-projection_2d}"
FRAMES="${2:-20}"
SERVER_IP="${3:-192.168.4.217}"
PORT="${4:-9999}"
TOPIC="${5:-/dense_pointcloud}"

WS="/root/udp_reliable_ros1_ws"
TS="$(date +%Y%m%d_%H%M%S)"

LOG_DIR="${WS}/benchmark_results/logs"
mkdir -p "${LOG_DIR}"

LOG_FILE="${LOG_DIR}/sender_${MODE}_${TS}.log"

cd "${WS}"
source /opt/ros/noetic/setup.bash
source "${WS}/devel/setup.bash"

echo "=========================================="
echo "Sender benchmark"
echo "MODE     : ${MODE}"
echo "FRAMES   : ${FRAMES}"
echo "SERVER_IP: ${SERVER_IP}"
echo "PORT     : ${PORT}"
echo "TOPIC    : ${TOPIC}"
echo "LOG_FILE : ${LOG_FILE}"
echo "=========================================="

rosrun udp_reliable_sender udp_reliable_sender_node \
  _server_ip:="${SERVER_IP}" \
  _server_port:="${PORT}" \
  _input_topic:="${TOPIC}" \
  _max_frames:="${FRAMES}" \
  _compression_mode:="${MODE}" 2>&1 | tee "${LOG_FILE}"

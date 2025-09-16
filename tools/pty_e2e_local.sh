#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${ROOT_DIR}/local_logs"
mkdir -p "${LOG_DIR}"

# 檢查工具
if ! command -v socat >/dev/null 2>&1; then
  echo "socat not found. Try: sudo apt-get install -y socat" >&2
  exit 1
fi

# 偵測可用的 Agent 啟動方式（ROS 版優先）
agent_cmd=""
if command -v ros2 >/dev/null 2>&1 && ros2 pkg list | grep -q '^micro_ros_agent$'; then
  agent_cmd="ros2 run micro_ros_agent micro_ros_agent"
elif command -v MicroXRCEAgent >/dev/null 2>&1; then
  agent_cmd="MicroXRCEAgent"
else
  echo "No micro-ROS agent found. Install ros-humble-micro-ros-agent or MicroXRCEAgent." >&2
  exit 1
fi

cleanup() {
  [[ -n "${AGENT_PID:-}"  ]] && kill "${AGENT_PID}"  >/dev/null 2>&1 || true
  [[ -n "${FEEDER_PID:-}" ]] && kill "${FEEDER_PID}" >/dev/null 2>&1 || true
  [[ -n "${SOCAT_PID:-}"  ]] && kill "${SOCAT_PID}"  >/dev/null 2>&1 || true
}
trap cleanup EXIT

SOCAT_LOG="${LOG_DIR}/socat.log"
AGENT_LOG="${LOG_DIR}/agent.log"
UART_LOG="${LOG_DIR}/uart.log"

# 1) 建一對 PTY
socat -d -d pty,raw,echo=0 pty,raw,echo=0 2>"${SOCAT_LOG}" &
SOCAT_PID=$!

# 解析 PTY 路徑
PTY_A=""; PTY_B=""
for _ in {1..30}; do
  sleep 0.1
  if grep -q "PTY is" "${SOCAT_LOG}"; then
    PTY_A=$(grep -m1 "PTY is" "${SOCAT_LOG}" | awk '{print $NF}')
    PTY_B=$(grep "PTY is" "${SOCAT_LOG}" | tail -n1 | awk '{print $NF}')
    [[ -n "${PTY_A}" && -n "${PTY_B}" && "${PTY_A}" != "${PTY_B}" ]] && break
  fi
done
if [[ -z "${PTY_A}" || -z "${PTY_B}" || "${PTY_A}" == "${PTY_B}" ]]; then
  echo "Failed to allocate PTYs. Check ${SOCAT_LOG}" >&2
  exit 1
fi
echo "[INFO] PTY_A=${PTY_A}, PTY_B=${PTY_B}"

# 2) 啟動 Agent 綁定 PTY_B
echo "[INFO] Starting Agent on ${PTY_B} (115200). Logs: ${AGENT_LOG}"
if [[ "${agent_cmd}" == "MicroXRCEAgent" ]]; then
  stdbuf -oL -eL MicroXRCEAgent serial --dev "${PTY_B}" -b 115200 >"${AGENT_LOG}" 2>&1 &
else
  stdbuf -oL -eL ${agent_cmd} serial --dev "${PTY_B}" -b 115200 >"${AGENT_LOG}" 2>&1 &
fi
AGENT_PID=$!

# 3) 啟動 feeder（若你之後有自己的 host 測試程式，改成在這裡呼叫它）
echo "[WARN] No real XRCE client yet. Using dummy bytes feeder for 10s."
( exec bash -c "exec 3>\"${PTY_A}\"; \
  printf '\xAA\x55\x01\x00' >&3; sleep 1; \
  printf '\xAA\x55\x02\x00' >&3; sleep 8" ) >"${UART_LOG}" 2>&1 &
FEEDER_PID=$!

echo "[INFO] Running for 10 seconds..."
sleep 10

echo "[INFO] Done. Logs at ${LOG_DIR}"

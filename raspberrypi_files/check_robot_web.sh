#!/usr/bin/env bash
set -u

SERVICE_NAME="${SERVICE_NAME:-brt7k-robot.service}"
HTTP_PORT="${HTTP_PORT:-8080}"
ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_WS="${ROS_WS:-${REPO_ROOT}/ros2_ws}"

section() {
  printf "\n==> %s\n" "$1"
}

run() {
  printf "+ %s\n" "$*"
  "$@" || true
}

section "Host"
run hostname -I
run ip -br addr

section "Systemd service"
run systemctl status "${SERVICE_NAME}" --no-pager

section "Listening ports"
run ss -ltnp

section "Local web checks"
run curl -I --max-time 3 "http://127.0.0.1:${HTTP_PORT}/"
run curl --max-time 3 "http://127.0.0.1:${HTTP_PORT}/"

section "ROS environment"
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
fi

if [[ -f "${ROS_WS}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${ROS_WS}/install/setup.bash"
fi

run ros2 service list
run ros2 service call /task_manager/status std_srvs/srv/Trigger "{}"

section "Recent logs"
run journalctl -u "${SERVICE_NAME}" -n 120 --no-pager

section "Expected URLs"
for ip in $(hostname -I); do
  printf "http://%s:%s/\n" "${ip}" "${HTTP_PORT}"
done
printf "rosbridge websocket: ws://<raspberry-pi-ip>:%s\n" "${ROSBRIDGE_PORT}"

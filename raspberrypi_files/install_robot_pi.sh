#!/usr/bin/env bash
set -euo pipefail

# Installs the BRT7K ROS 2 stack on Ubuntu for Raspberry Pi and configures
# a systemd autostart that starts the robot through task_manager.
#
# Usage:
#   sudo ./install_robot_pi.sh
#   sudo ./install_robot_pi.sh --user ubuntu --ros-domain-id 0
#   sudo ./install_robot_pi.sh --install-dir /home/ubuntu/BRT7K

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-0}"
TARGET_USER="${SUDO_USER:-${USER:-ubuntu}}"
REPO_URL="${REPO_URL:-https://github.com/Oderlesspanic/BRT7K.git}"
INSTALL_DIR=""
BUILD_WORKSPACE=1
ENABLE_SERVICE=1

while [[ $# -gt 0 ]]; do
  case "$1" in
    --user)
      TARGET_USER="$2"
      shift 2
      ;;
    --ros-domain-id)
      ROS_DOMAIN_ID_VALUE="$2"
      shift 2
      ;;
    --repo-url)
      REPO_URL="$2"
      shift 2
      ;;
    --install-dir)
      INSTALL_DIR="$2"
      shift 2
      ;;
    --no-build)
      BUILD_WORKSPACE=0
      shift
      ;;
    --no-enable)
      ENABLE_SERVICE=0
      shift
      ;;
    -h|--help)
      sed -n '1,28p' "$0"
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

if [[ "${EUID}" -ne 0 ]]; then
  echo "Please run this script with sudo/root." >&2
  exit 1
fi

if ! id "${TARGET_USER}" >/dev/null 2>&1; then
  echo "Target user '${TARGET_USER}' does not exist." >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT=""
ROS_WS=""

run_as_user() {
  sudo -H -u "${TARGET_USER}" bash -lc "$*"
}

echo "==> Installing Ubuntu and ROS package prerequisites"
apt-get update
apt-get install -y \
  curl \
  gnupg \
  lsb-release \
  software-properties-common \
  sudo

add-apt-repository universe -y
add-apt-repository "deb http://ports.ubuntu.com/ubuntu-ports $(. /etc/os-release && echo "${UBUNTU_CODENAME}")-updates main universe restricted multiverse" -y
add-apt-repository "deb http://ports.ubuntu.com/ubuntu-ports $(. /etc/os-release && echo "${UBUNTU_CODENAME}")-backports main universe restricted multiverse" -y
add-apt-repository "deb http://ports.ubuntu.com/ubuntu-ports $(. /etc/os-release && echo "${UBUNTU_CODENAME}")-security main universe restricted multiverse" -y

ROS2_DEB822_SOURCE=""
if [[ -d /etc/apt/sources.list.d ]]; then
  ROS2_DEB822_SOURCE="$(
    grep -Rls "packages.ros.org/ros2/ubuntu" /etc/apt/sources.list.d/*.sources 2>/dev/null || true
  )"
fi

if [[ -n "${ROS2_DEB822_SOURCE}" ]]; then
  rm -f /etc/apt/sources.list.d/ros2.list
elif ! grep -Rqs "packages.ros.org/ros2/ubuntu" /etc/apt/sources.list /etc/apt/sources.list.d 2>/dev/null; then
  install -d -m 0755 /etc/apt/keyrings
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /etc/apt/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "${UBUNTU_CODENAME}") main" \
    > /etc/apt/sources.list.d/ros2.list
fi

apt-get update
apt-get install -f -y
apt-get full-upgrade -y
apt-get install -y \
  build-essential \
  cmake \
  git \
  nano \
  vim \
  python3-pip \
  python3-rosdep \
  python3-vcstool \
  iputils-ping \
  curl \
  tmux \
  pkg-config \
  i2c-tools \
  v4l-utils \
  udev \
  libcamera-dev \
  libgmock-dev \
  nlohmann-json3-dev \
  python3-colcon-common-extensions \
  python3-opencv \
  python3-numpy \
  ros-dev-tools \
  "ros-${ROS_DISTRO}-desktop" \
  "ros-${ROS_DISTRO}-xacro" \
  "ros-${ROS_DISTRO}-imu-filter-madgwick" \
  "ros-${ROS_DISTRO}-robot-localization" \
  "ros-${ROS_DISTRO}-navigation2" \
  "ros-${ROS_DISTRO}-nav2-bringup" \
  "ros-${ROS_DISTRO}-slam-toolbox" \
  "ros-${ROS_DISTRO}-tf2" \
  "ros-${ROS_DISTRO}-tf2-ros" \
  "ros-${ROS_DISTRO}-camera-ros" \
  "ros-${ROS_DISTRO}-cv-bridge" \
  "ros-${ROS_DISTRO}-vision-opencv" \
  "ros-${ROS_DISTRO}-image-transport" \
  "ros-${ROS_DISTRO}-tf-transformations" \
  "ros-${ROS_DISTRO}-laser-filters" \
  "ros-${ROS_DISTRO}-message-filters" \
  "ros-${ROS_DISTRO}-vision-msgs" \
  "ros-${ROS_DISTRO}-image-pipeline" \
  "ros-${ROS_DISTRO}-rosbridge-suite"

echo "==> Installing micro-ROS Agent if available via apt"
if apt-cache show "ros-${ROS_DISTRO}-micro-ros-agent" >/dev/null 2>&1; then
  apt-get install -y "ros-${ROS_DISTRO}-micro-ros-agent"
else
  echo "WARNING: ros-${ROS_DISTRO}-micro-ros-agent was not found in apt."
  echo "         The workspace contains micro_ros_setup, but the serial agents in hardware.launch.py need micro_ros_agent."
fi

USER_HOME="$(getent passwd "${TARGET_USER}" | cut -d: -f6)"
if [[ -z "${INSTALL_DIR}" ]]; then
  INSTALL_DIR="${USER_HOME}/BRT7K"
fi

echo "==> Preparing Git repository and submodules"
if [[ -d "${SCRIPT_REPO_ROOT}/.git" && -d "${SCRIPT_REPO_ROOT}/ros2_ws/src" ]]; then
  REPO_ROOT="${SCRIPT_REPO_ROOT}"
  echo "Using existing repository at ${REPO_ROOT}"
elif [[ -d "${INSTALL_DIR}/.git" ]]; then
  REPO_ROOT="${INSTALL_DIR}"
  echo "Using existing repository at ${REPO_ROOT}"
else
  if [[ -e "${INSTALL_DIR}" ]]; then
    echo "Install directory exists but is not a Git repository: ${INSTALL_DIR}" >&2
    echo "Remove it, choose --install-dir, or clone the repository there first." >&2
    exit 1
  fi
  run_as_user "git clone --recurse-submodules '${REPO_URL}' '${INSTALL_DIR}'"
  REPO_ROOT="${INSTALL_DIR}"
fi

run_as_user "git -C '${REPO_ROOT}' submodule update --init --recursive"
ROS_WS="${REPO_ROOT}/ros2_ws"
MAP_DIR="${ROS_WS}/maps/map"

if [[ ! -d "${ROS_WS}/src" ]]; then
  echo "Could not find ROS workspace at ${ROS_WS}" >&2
  exit 1
fi

install -d -o "${TARGET_USER}" -g "${TARGET_USER}" "${MAP_DIR}"

echo "==> Preparing user permissions"
for group_name in dialout video i2c gpio render; do
  if getent group "${group_name}" >/dev/null 2>&1; then
    usermod -aG "${group_name}" "${TARGET_USER}"
  fi
done

echo "==> Adding ROS setup to ${TARGET_USER}'s shell profile"
touch "${USER_HOME}/.bashrc"
if ! grep -q "/opt/ros/${ROS_DISTRO}/setup.bash" "${USER_HOME}/.bashrc"; then
  echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> "${USER_HOME}/.bashrc"
fi
if ! grep -q "${ROS_WS}/install/setup.bash" "${USER_HOME}/.bashrc"; then
  echo "[[ -f ${ROS_WS}/install/setup.bash ]] && source ${ROS_WS}/install/setup.bash" >> "${USER_HOME}/.bashrc"
fi
if ! grep -q "BRT7K_MAP_DIR=" "${USER_HOME}/.bashrc"; then
  echo "export BRT7K_MAP_DIR=${MAP_DIR}" >> "${USER_HOME}/.bashrc"
fi
chown "${TARGET_USER}:${TARGET_USER}" "${USER_HOME}/.bashrc"

echo "==> Initializing rosdep"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  rosdep init
fi
run_as_user "rosdep update"

if [[ "${BUILD_WORKSPACE}" -eq 1 ]]; then
  echo "==> Installing ROS dependencies from workspace"
  run_as_user "cd '${ROS_WS}' && source /opt/ros/${ROS_DISTRO}/setup.bash && rosdep install --from-paths src --ignore-src -r -y"

  echo "==> Removing stale generated interface artifacts"
  run_as_user "rm -rf '${ROS_WS}/build/interfaces' '${ROS_WS}/install/interfaces'"

  echo "==> Building workspace"
  run_as_user "cd '${ROS_WS}' && source /opt/ros/${ROS_DISTRO}/setup.bash && colcon --log-base /tmp/brt7k-colcon-log build"
fi

echo "==> Installing robot autostart launcher"
install -m 0755 "${REPO_ROOT}/raspberrypi_files/brt7k_assign_serial.py" \
  /usr/local/bin/brt7k_assign_serial.py
rm -f /etc/udev/rules.d/brt7k_serial.rules
install -m 0644 "${REPO_ROOT}/raspberrypi_files/rplidar.rules" \
  /etc/udev/rules.d/rplidar.rules
udevadm control --reload-rules
udevadm trigger

cat > /usr/local/bin/brt7k_robot_autostart.sh <<EOF
#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/${ROS_DISTRO}/setup.bash
if [[ ! -f "${ROS_WS}/install/setup.bash" ]]; then
  echo "Missing ROS workspace overlay: ${ROS_WS}/install/setup.bash" >&2
  echo "Build the workspace before starting brt7k-robot.service." >&2
  exit 1
fi
source "${ROS_WS}/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VALUE}"
export BRT7K_MAP_DIR="${MAP_DIR}"

cd "${ROS_WS}"

ros2 launch task_manager task_manager.launch.py &
TASK_MANAGER_PID=\$!

stop_task_manager() {
  if kill -0 "\${TASK_MANAGER_PID}" >/dev/null 2>&1; then
    kill -INT "\${TASK_MANAGER_PID}" >/dev/null 2>&1 || true
    wait "\${TASK_MANAGER_PID}" || true
  fi
}
trap stop_task_manager INT TERM EXIT

for _ in {1..60}; do
  if ros2 service list | grep -q "/task_manager/start_description"; then
    break
  fi
  sleep 1
done

# Start only the dashboard stack. Robot subsystems are controlled explicitly
# through the web GUI or task_manager services.
for target in web; do
  ros2 service call "/task_manager/start_\${target}" std_srvs/srv/Trigger "{}" || true
  sleep 1
done

wait "\${TASK_MANAGER_PID}"
EOF
chmod 0755 /usr/local/bin/brt7k_robot_autostart.sh

echo "==> Installing systemd service"
cat > /etc/systemd/system/brt7k-assign-serial.service <<EOF
[Unit]
Description=BRT7K ESP32 serial role assignment
After=systemd-udev-settle.service
Before=brt7k-robot.service

[Service]
Type=oneshot
ExecStartPre=/usr/bin/udevadm settle --timeout=10
ExecStart=/usr/local/bin/brt7k_assign_serial.py --require drive
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

cat > /etc/systemd/system/brt7k-robot.service <<EOF
[Unit]
Description=BRT7K robot autostart
Wants=network-online.target
Wants=brt7k-assign-serial.service
After=network-online.target brt7k-assign-serial.service

[Service]
Type=simple
User=${TARGET_USER}
WorkingDirectory=${ROS_WS}
Environment=ROS_DOMAIN_ID=${ROS_DOMAIN_ID_VALUE}
Environment=BRT7K_MAP_DIR=${MAP_DIR}
ExecStart=/usr/local/bin/brt7k_robot_autostart.sh
Restart=on-failure
RestartSec=5
KillSignal=SIGINT
TimeoutStopSec=30

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
if [[ "${ENABLE_SERVICE}" -eq 1 ]]; then
  systemctl enable brt7k-assign-serial.service
  systemctl enable brt7k-robot.service
fi

echo
echo "Done."
echo "Serial assignment: brt7k-assign-serial.service"
echo "Autostart service: brt7k-robot.service"
echo "Assign now:        sudo systemctl start brt7k-assign-serial.service"
echo "Start now:        sudo systemctl start brt7k-robot.service"
echo "Show serial logs:  journalctl -u brt7k-assign-serial.service -n 80 --no-pager"
echo "Show logs:        journalctl -u brt7k-robot.service -f"
echo "Mapping remains disabled on boot and can be started through the web GUI/task_manager."
echo
echo "Important:"
echo "  Reboot or log out/in once so group permissions for serial, camera and I2C devices apply."
echo "  Check that the ESP32 serial devices match the paths used by hardware.launch.py."

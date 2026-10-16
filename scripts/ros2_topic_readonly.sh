#!/usr/bin/env bash
set -euo pipefail
# Read-only server diagnostics. No launch, controller calls or command publishers.
if [[ "${1:-}" == --help ]]; then
  echo 'Usage: bash scripts/ros2_topic_readonly.sh info|echo TOPIC [ros2 options] [--raw-warnings]'
  echo 'Only info/echo; no daemon; 12-second timeout. Filters only missing-type-hash warnings.'
  exit 0
fi
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd -- "${script_dir}/.." && pwd)"
# Use system Python and clean ROS paths; do not change the caller or ~/.bashrc.
exec env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u PYTHONPATH -u LD_LIBRARY_PATH \
  -u CMAKE_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION \
  PATH=/usr/bin:/bin:/usr/sbin:/sbin PYTHONDONTWRITEBYTECODE=1 \
  bash --noprofile --norc -s -- "${repository_root}" "$@" <<'INNER'
set -eo pipefail
repository_root="$1"; shift
ros_distro="${DATA_SERVER_ROS_DISTRO:-jazzy}"
for setup in "/opt/ros/${ros_distro}/setup.bash" \
  "${HOME}/franka_ros2_ws/install/local_setup.bash" \
  "${repository_root}/gello_software/ros2/install/local_setup.bash"; do
  [[ -r "$setup" ]] || { echo "Missing ROS setup: $setup" >&2; exit 1; }
  source "$setup"
done
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
if [[ "$ros_distro" == humble ]]; then
  export ROS_LOCALHOST_ONLY=0
else
  unset ROS_LOCALHOST_ONLY
fi
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
if [[ -z "${CYCLONEDDS_URI:-}" && -r "${HOME}/.config/cyclonedds.xml" ]]; then
  export CYCLONEDDS_URI="file://${HOME}/.config/cyclonedds.xml"
fi
# Keep any diagnostic files inside the server's srg directory.
export ROS_LOG_DIR="${repository_root}/../ros_diagnostics/log"
exec /usr/bin/python3 "${repository_root}/scripts/ros2_topic_readonly.py" "$@"
INNER

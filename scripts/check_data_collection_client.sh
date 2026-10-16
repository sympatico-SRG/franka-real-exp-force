#!/usr/bin/env bash
set -euo pipefail

# Read-only preflight for the robot-control host. This script never launches a
# controller, opens an FCI session, or publishes a command.

usage() {
  cat <<'EOF'
Usage: ./scripts/check_data_collection_client.sh [--left|--right|--duo]

With no arm option, validate only the installed ROS/Franka message interface.
With an arm option, additionally require one real RobotState payload from each
selected arm. Start the normal client control stack first when using this check.
EOF
}

die() {
  echo "Error: $*" >&2
  exit 1
}

arm_mode=""
while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --left|--right|--duo)
      [[ -z "${arm_mode}" ]] || die "choose only one of --left, --right, or --duo"
      arm_mode="${1#--}"
      shift
      ;;
    --help)
      usage
      exit 0
      ;;
    *)
      usage >&2
      die "unknown argument: $1"
      ;;
  esac
done

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd -- "${script_dir}/.." && pwd)"

setup_files=(
  "/opt/ros/humble/setup.bash"
  "${HOME}/franka_ros2_ws/install/local_setup.bash"
  "${repository_root}/gello_software/ros2/install/local_setup.bash"
)

unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
set +u
for setup_file in "${setup_files[@]}"; do
  [[ -r "${setup_file}" ]] || die "ROS setup file is missing or unreadable: ${setup_file}"
  # shellcheck disable=SC1090
  source "${setup_file}"
done
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

python3 - <<'PY'
from franka_msgs.msg import FrankaRobotState

fields = FrankaRobotState.get_fields_and_field_types()
required = {"measured_joint_state", "tau_ext_hat_filtered"}
missing = sorted(required - set(fields))
if missing:
    raise SystemExit(f"FrankaRobotState is missing required fields: {missing}")

message = FrankaRobotState()
for field_name in sorted(required):
    joint_state = getattr(message, field_name)
    nested_fields = joint_state.get_fields_and_field_types()
    if "effort" not in nested_fields:
        raise SystemExit(f"{field_name} has no effort field")

print("Static Franka torque interface check: PASS")
print(f"measured_joint_state: {fields['measured_joint_state']}")
print(f"tau_ext_hat_filtered: {fields['tau_ext_hat_filtered']}")
PY

[[ -n "${arm_mode}" ]] || exit 0
command -v timeout >/dev/null 2>&1 || die "required command not found: timeout"

case "${arm_mode}" in
  left) arm_names=(left) ;;
  right) arm_names=(right) ;;
  duo) arm_names=(left right) ;;
esac

for arm_name in "${arm_names[@]}"; do
  topic="/${arm_name}/franka_robot_state_broadcaster/robot_state"
  topic_type="$(timeout 10s ros2 topic type "${topic}" --no-daemon 2>/dev/null || true)"
  [[ "${topic_type}" == "franka_msgs/msg/FrankaRobotState" ]] || die \
    "${topic} is unavailable or has unexpected type ${topic_type:-<none>}"
  if ! timeout 10s ros2 topic echo \
    "${topic}" franka_msgs/msg/FrankaRobotState --once --no-daemon \
    --qos-reliability best_effort >/dev/null 2>&1; then
    die "${topic} was discovered but no RobotState payload was received"
  fi
  echo "Live RobotState payload check: PASS (${topic})"
done

"""Read-only Franka state diagnostics; never creates robot command publishers."""
import argparse
import math
import os
import time

import rclpy
from franka_msgs.msg import FrankaRobotState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message, serialize_message
from rclpy.utilities import get_rmw_implementation_identifier


def validate(message):
    for field in ("measured_joint_state", "tau_ext_hat_filtered"):
        effort = getattr(message, field).effort
        if len(effort) != 7 or not all(math.isfinite(x) for x in effort):
            return f"{field}.effort must contain 7 finite values"
    stamp = message.header.stamp
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000 or (stamp.sec == 0 and stamp.nanosec == 0):
        return "header timestamp is invalid or zero"
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    arms = parser.add_mutually_exclusive_group()
    for arm in ("left", "right", "duo"):
        arms.add_argument(f"--{arm}", action="store_true")
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--verbose", action="store_true", help="retain Cyclone DDS discovery warnings")
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be finite and positive")
    message = deserialize_message(serialize_message(FrankaRobotState()), FrankaRobotState)
    for field in ("measured_joint_state", "tau_ext_hat_filtered"):
        if not hasattr(getattr(message, field), "effort"):
            raise RuntimeError(f"Missing required torque field: {field}.effort")
    print(f"ROS={os.environ.get('ROS_DISTRO')} RMW={get_rmw_implementation_identifier()} DOMAIN={os.environ.get('ROS_DOMAIN_ID')}", flush=True)
    print("PASS: local Franka message type support and serialization (not a cross-host compatibility test).", flush=True)
    selected = ["left", "right"] if args.duo else ["left"] if args.left else ["right"] if args.right else []
    if not selected:
        print("Offline check only. Use --left/--right/--duo to subscribe to existing publishers.")
        return 0
    print("Humble peers may omit type hashes in Jazzy discovery. Missing hashes alone do not prove data failure.", flush=True)
    if not args.verbose:
        print("Cyclone DDS warnings hidden for this check only; errors retained. Use --verbose for raw warnings.", flush=True)
    ros_args = [] if args.verbose else ["--ros-args", "--log-level", "rmw_cyclonedds_cpp:=error"]
    rclpy.init(args=ros_args)
    node = Node("srg_robot_state_check", enable_rosout=False, start_parameter_services=False)
    received, invalid, topics = {}, {}, {}
    try:
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        def on_message(arm, sample):
            error = validate(sample)
            if error:
                invalid[arm] = error
            else:
                received[arm] = sample
        for arm in selected:
            topics[arm] = f"/{arm}/franka_robot_state_broadcaster/robot_state"
            node.create_subscription(FrankaRobotState, topics[arm], lambda sample, arm=arm: on_message(arm, sample), qos)
        deadline = time.monotonic() + args.timeout
        while rclpy.ok() and len(received) < len(selected) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.2, max(0.0, deadline-time.monotonic())))
        for arm, topic in topics.items():
            publishers = node.count_publishers(topic)
            if arm in received:
                sample = received[arm]
                print(f"PASS {arm}: received valid state; publishers={publishers}; tau_J={list(sample.measured_joint_state.effort)}; tau_ext={list(sample.tau_ext_hat_filtered.effort)}")
            elif arm in invalid:
                print(f"FAIL {arm}: received invalid state: {invalid[arm]}")
            elif publishers == 0:
                print(f"FAIL {arm}: no publisher discovered within {args.timeout:g}s. Check the existing robot stack/domain/network; this tool does not start it.")
            else:
                print(f"FAIL {arm}: {publishers} publisher(s) discovered but no valid sample. Check both hosts' franka_msgs definitions, DDS/network and broadcaster publication.")
        return 0 if len(received) == len(selected) else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

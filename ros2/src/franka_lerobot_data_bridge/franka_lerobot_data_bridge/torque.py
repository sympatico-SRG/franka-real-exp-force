from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import Mapping, Sequence


FRANKA_JOINT_COUNT = 7
TORQUE_UNIT = "N.m"


@dataclass(frozen=True)
class TorqueSample:
    """Measured and estimated external joint torques from one RobotState message."""

    joint_torque: tuple[float, ...]
    external_joint_torque: tuple[float, ...]
    stamp_s: float


def ordered_franka_joint_values(
    joint_names: Sequence[str], values: Sequence[float], *, field_name: str
) -> tuple[float, ...]:
    """Return seven finite values ordered joint1..joint7.

    Franka's RobotState normally includes names. A raw-order fallback is allowed only
    when no recognizable Franka joint names are present, matching the existing bridge
    behavior for joint positions. Partially recognizable names are rejected because
    falling back could silently permute joints.
    """

    if len(values) != FRANKA_JOINT_COUNT:
        raise ValueError(
            f"{field_name} must contain exactly {FRANKA_JOINT_COUNT} values; got {len(values)}."
        )

    numeric_values = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in numeric_values):
        raise ValueError(f"{field_name} contains a non-finite value.")

    ordered: list[float | None] = [None] * FRANKA_JOINT_COUNT
    matched = 0
    for joint_name, value in zip(joint_names, numeric_values, strict=False):
        match = re.search(r"(?:^|_)(?:fr3|panda)_joint([1-7])$", str(joint_name))
        if match is None:
            continue
        index = int(match.group(1)) - 1
        if ordered[index] is not None:
            raise ValueError(f"{field_name} contains duplicate joint {index + 1}.")
        ordered[index] = value
        matched += 1

    if matched == FRANKA_JOINT_COUNT:
        return tuple(float(value) for value in ordered if value is not None)
    if matched:
        raise ValueError(
            f"{field_name} contains only {matched} recognizable Franka joint names; expected 7."
        )
    return numeric_values


def make_torque_sample(
    measured_names: Sequence[str],
    measured_effort: Sequence[float],
    external_names: Sequence[str],
    external_effort: Sequence[float],
    stamp_s: float,
) -> TorqueSample:
    stamp_s = float(stamp_s)
    if not math.isfinite(stamp_s) or stamp_s < 0.0:
        raise ValueError(f"RobotState timestamp must be finite and non-negative; got {stamp_s}.")
    return TorqueSample(
        joint_torque=ordered_franka_joint_values(
            measured_names, measured_effort, field_name="tau_J"
        ),
        external_joint_torque=ordered_franka_joint_values(
            external_names,
            external_effort,
            field_name="tau_ext_hat_filtered",
        ),
        stamp_s=stamp_s,
    )


def torque_sample_wait_reason(
    sample: TorqueSample | None,
    *,
    arm_name: str,
    robot_joint_stamp_s: float,
    reference_time_s: float,
    max_age_sec: float,
    max_state_skew_sec: float,
    future_tolerance_sec: float = 0.01,
) -> str | None:
    """Explain why a torque sample cannot be paired with the current observation."""

    if sample is None:
        return f"{arm_name} Franka torque state"

    age_s = reference_time_s - sample.stamp_s
    if age_s > max_age_sec:
        return f"fresh {arm_name} Franka torque state"
    if age_s < -future_tolerance_sec:
        return f"non-future {arm_name} Franka torque state"
    if abs(sample.stamp_s - robot_joint_stamp_s) > max_state_skew_sec:
        return f"synchronized {arm_name} Franka torque/joint state"
    return None


def torque_sample_skew_wait_reason(
    sample: TorqueSample,
    *,
    arm_name: str,
    source_stamps_s: Mapping[str, float],
    max_skew_sec: float,
) -> str | None:
    """Reject a torque sample that is not aligned with any current frame source."""

    for source_name, source_stamp_s in source_stamps_s.items():
        if abs(sample.stamp_s - float(source_stamp_s)) > max_skew_sec:
            return f"synchronized {arm_name} Franka torque/{source_name}"
    return None

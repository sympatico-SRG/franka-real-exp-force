from __future__ import annotations

import sys
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from franka_lerobot_data_bridge.torque import (  # noqa: E402
    TorqueSample,
    make_torque_sample,
    torque_sample_skew_wait_reason,
    torque_sample_wait_reason,
)


def _joint_names(prefix: str = "fr3") -> list[str]:
    return [f"{prefix}_joint{index}" for index in range(1, 8)]


def test_parses_and_orders_seven_dimensional_torques() -> None:
    reverse_names = list(reversed(_joint_names()))
    sample = make_torque_sample(
        reverse_names,
        list(reversed([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0])),
        reverse_names,
        list(reversed([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])),
        12.5,
    )

    assert sample.joint_torque == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0)
    assert sample.external_joint_torque == pytest.approx((0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7))


@pytest.mark.parametrize(
    ("measured", "external"),
    [([1.0] * 6, [0.0] * 7), ([1.0] * 7, [0.0] * 8), ([1.0] * 6 + [float("nan")], [0.0] * 7)],
)
def test_rejects_bad_torque_shape_or_value(measured: list[float], external: list[float]) -> None:
    with pytest.raises(ValueError):
        make_torque_sample(_joint_names(), measured, _joint_names(), external, 1.0)


def test_rejects_partial_joint_names_instead_of_guessing_order() -> None:
    names = _joint_names()
    names[-1] = "unknown_joint"
    with pytest.raises(ValueError, match="only 6 recognizable"):
        make_torque_sample(names, [0.0] * 7, _joint_names(), [0.0] * 7, 1.0)


def test_timestamp_freshness_alignment_missing_and_future_checks() -> None:
    sample = TorqueSample((1.0,) * 7, (0.1,) * 7, stamp_s=10.0)
    common = {
        "arm_name": "left",
        "max_age_sec": 0.5,
        "max_state_skew_sec": 0.05,
    }

    assert torque_sample_wait_reason(
        sample, robot_joint_stamp_s=10.01, reference_time_s=10.1, **common
    ) is None
    assert "fresh" in torque_sample_wait_reason(
        sample, robot_joint_stamp_s=10.0, reference_time_s=10.6, **common
    )
    assert "synchronized" in torque_sample_wait_reason(
        sample, robot_joint_stamp_s=10.2, reference_time_s=10.1, **common
    )
    assert "non-future" in torque_sample_wait_reason(
        TorqueSample((1.0,) * 7, (0.1,) * 7, 10.2),
        robot_joint_stamp_s=10.2,
        reference_time_s=10.1,
        **common,
    )
    assert "torque state" in torque_sample_wait_reason(
        None, robot_joint_stamp_s=10.0, reference_time_s=10.1, **common
    )


def test_torque_is_aligned_with_action_and_camera_timestamps() -> None:
    sample = TorqueSample((1.0,) * 7, (0.1,) * 7, stamp_s=10.0)
    assert torque_sample_skew_wait_reason(
        sample,
        arm_name="left",
        source_stamps_s={"action joint state": 10.02, "cam_front": 9.97},
        max_skew_sec=0.05,
    ) is None
    assert "cam_front" in torque_sample_skew_wait_reason(
        sample,
        arm_name="left",
        source_stamps_s={"action joint state": 10.02, "cam_front": 9.9},
        max_skew_sec=0.05,
    )

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from data_collection.lerobot_collection import build_features, packet_to_frame
from data_collection.torque_dataset import (
    torque_arrays_from_packet,
    torque_config_from_packet,
)
from lerobot.datasets.lerobot_dataset import LeRobotDataset


def _packet(*, record_torque: bool, arm_mode: str = "left") -> dict:
    arms = ["left", "right"] if arm_mode == "duo" else [arm_mode]
    dim = 7 * len(arms)
    packet = {
        "state": [0.0] * dim,
        "action": [0.0] * dim,
        "joint_state": [0.0] * dim,
        "target_joint": [0.0] * dim,
        "joint_state_dim": dim,
        "target_joint_dim": dim,
        "robot_state_dim": dim,
        "action_dim": dim,
        "ee_pose": [0.0] * (9 * len(arms)),
        "target_ee_pose": [0.0] * (9 * len(arms)),
        "delta_ee_pose": [0.0] * (6 * len(arms)),
        "camera_names": [],
        "cameras": {},
        "arm_mode": arm_mode,
        "include_right_arm": arm_mode == "duo",
        "include_gripper": False,
        "include_hand": False,
        "state_action_mode": "joint",
        "arm_action_representation": "delta_joint_position",
        "gripper_action_representation": "absolute_width",
    }
    if record_torque:
        packet.update(
            {
                "record_torque": True,
                "joint_torque": np.arange(dim, dtype=np.float64).tolist(),
                "external_joint_torque": (
                    np.arange(dim, dtype=np.float64) * 0.1
                ).tolist(),
                "torque_dim": dim,
                "torque_unit": "N.m",
                "torque_arm_order": arms,
            }
        )
    return packet


def test_record_torque_off_preserves_legacy_features_and_frame() -> None:
    packet = _packet(record_torque=False)
    features, camera_names = build_features(packet)
    frame = packet_to_frame(packet, camera_names, "legacy")

    assert "observation.joint_torque" not in features
    assert "observation.external_joint_torque" not in features
    assert "observation.joint_torque" not in frame
    assert "observation.external_joint_torque" not in frame


def test_record_torque_on_registers_float32_features_and_frame() -> None:
    packet = _packet(record_torque=True)
    features, camera_names = build_features(packet)
    frame = packet_to_frame(packet, camera_names, "torque")

    assert features["observation.joint_torque"]["shape"] == (7,)
    assert features["observation.external_joint_torque"]["shape"] == (7,)
    assert features["observation.joint_torque"]["dtype"] == "float32"
    assert frame["observation.joint_torque"].shape == (7,)
    assert frame["observation.joint_torque"].dtype == np.float32
    assert frame["observation.external_joint_torque"].dtype == np.float32


@pytest.mark.parametrize(
    ("arm_mode", "expected_order"),
    [("left", ["left"]), ("right", ["right"]), ("duo", ["left", "right"])],
)
def test_torque_metadata_tracks_selected_arm(
    arm_mode: str, expected_order: list[str]
) -> None:
    config = torque_config_from_packet(_packet(record_torque=True, arm_mode=arm_mode))
    assert config is not None
    assert config["arm_order"] == expected_order
    assert config["unit"] == "N.m"


def test_dual_arm_torque_uses_left_then_right_order() -> None:
    packet = _packet(record_torque=True, arm_mode="duo")
    arrays = torque_arrays_from_packet(packet)
    assert arrays is not None
    joint_torque, external_torque = arrays

    assert joint_torque.shape == (14,)
    assert external_torque.shape == (14,)
    np.testing.assert_array_equal(joint_torque[:7], np.arange(7, dtype=np.float32))
    np.testing.assert_array_equal(joint_torque[7:], np.arange(7, 14, dtype=np.float32))


def test_missing_or_non_finite_torque_is_rejected() -> None:
    incomplete = _packet(record_torque=True)
    incomplete.pop("external_joint_torque")
    with pytest.raises(ValueError, match="incomplete"):
        torque_arrays_from_packet(incomplete)

    non_finite = _packet(record_torque=True)
    non_finite["joint_torque"][-1] = float("nan")
    with pytest.raises(ValueError, match="non-finite"):
        torque_arrays_from_packet(non_finite)


def test_lerobot_torque_dataset_save_and_reload(tmp_path: Path) -> None:
    packet = _packet(record_torque=True)
    features, camera_names = build_features(packet)
    dataset = LeRobotDataset.create(
        repo_id="local/torque_test",
        fps=15,
        features=features,
        use_videos=False,
        root=tmp_path / "torque_dataset",
    )
    dataset.add_frame(packet_to_frame(packet, camera_names, "test"))
    dataset.save_episode()
    dataset.finalize()

    loaded = LeRobotDataset(repo_id="local/torque_test", root=dataset.root)
    frame = loaded[0]
    np.testing.assert_allclose(frame["observation.joint_torque"].numpy(), np.arange(7))
    np.testing.assert_allclose(
        frame["observation.external_joint_torque"].numpy(), np.arange(7) * 0.1
    )

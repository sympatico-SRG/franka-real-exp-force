from __future__ import annotations

from typing import Any

import numpy as np


TORQUE_FIELDS = ("joint_torque", "external_joint_torque")


def torque_arm_order(packet: dict[str, Any]) -> list[str]:
    arm_mode = str(packet.get("arm_mode", "")).strip().lower()
    if arm_mode == "single":
        arm_mode = "left"
    if arm_mode == "left":
        return ["left"]
    if arm_mode == "right":
        return ["right"]
    if arm_mode == "duo" or (not arm_mode and bool(packet.get("include_right_arm", True))):
        return ["left", "right"]
    if not arm_mode:
        return ["left"]
    raise ValueError(f"Unsupported torque arm_mode={arm_mode!r}.")


def torque_arrays_from_packet(packet: dict[str, Any]) -> tuple[np.ndarray, np.ndarray] | None:
    present = [field in packet for field in TORQUE_FIELDS]
    requested = bool(packet.get("record_torque", False))
    if not any(present):
        if requested:
            raise ValueError("Torque recording is enabled but the packet has no torque fields.")
        return None
    if not all(present):
        missing = [field for field in TORQUE_FIELDS if field not in packet]
        raise ValueError(f"Torque packet is incomplete; missing {', '.join(missing)}.")
    if not requested:
        raise ValueError("Torque fields are present but record_torque is not enabled.")
    if packet.get("torque_unit") != "N.m":
        raise ValueError(f"Torque packet unit must be 'N.m'; got {packet.get('torque_unit')!r}.")

    arm_order = torque_arm_order(packet)
    packet_arm_order = list(packet.get("torque_arm_order", arm_order))
    if packet_arm_order != arm_order:
        raise ValueError(
            f"Torque arm order must be {arm_order}; got {packet_arm_order}."
        )
    expected_dim = 7 * len(arm_order)
    packet_dim = int(packet.get("torque_dim", expected_dim))
    if packet_dim != expected_dim:
        raise ValueError(f"Torque packet dimension must be {expected_dim}; got {packet_dim}.")

    arrays = tuple(np.asarray(packet[field], dtype=np.float32) for field in TORQUE_FIELDS)
    for field, values in zip(TORQUE_FIELDS, arrays, strict=True):
        if values.shape != (expected_dim,):
            raise ValueError(f"{field} must have shape ({expected_dim},); got {values.shape}.")
        if not np.isfinite(values).all():
            raise ValueError(f"{field} contains a non-finite value.")
    return arrays


def torque_config_from_packet(packet: dict[str, Any]) -> dict[str, Any] | None:
    arrays = torque_arrays_from_packet(packet)
    if arrays is None:
        return None
    return {
        "unit": "N.m",
        "arm_order": torque_arm_order(packet),
        "joints_per_arm": 7,
        "joint_torque_source": "libfranka RobotState.tau_J",
        "external_joint_torque_source": "libfranka RobotState.tau_ext_hat_filtered",
    }


def torque_feature_specs(packet: dict[str, Any]) -> dict[str, dict[str, Any]]:
    arrays = torque_arrays_from_packet(packet)
    if arrays is None:
        return {}
    torque_dim = int(arrays[0].shape[0])
    return {
        "observation.joint_torque": {
            "dtype": "float32",
            "shape": (torque_dim,),
            "names": ["joint_torque_Nm"],
        },
        "observation.external_joint_torque": {
            "dtype": "float32",
            "shape": (torque_dim,),
            "names": ["external_joint_torque_Nm"],
        },
    }


def add_torque_to_frame(frame: dict[str, Any], packet: dict[str, Any]) -> None:
    arrays = torque_arrays_from_packet(packet)
    if arrays is None:
        return
    frame["observation.joint_torque"] = arrays[0]
    frame["observation.external_joint_torque"] = arrays[1]

#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Python 2/3-compatible validation for auto-capture safe joint poses."""

import math


SAFE_EMPTY_KEY = "safe_joints_empty"
SAFE_GRIPPED_KEY = "safe_joints_gripped"


def validate_joint_vector(value, label):
    if not isinstance(value, (list, tuple)) or len(value) != 6:
        raise ValueError("{} must be an explicit 6-element joint vector".format(label))
    out = []
    for idx, raw in enumerate(value):
        try:
            val = float(raw)
        except (TypeError, ValueError):
            raise ValueError("{}[{}] is not numeric".format(label, idx))
        if math.isnan(val) or math.isinf(val):
            raise ValueError("{}[{}] must be finite".format(label, idx))
        out.append(val)
    return out


SAFE_MODE_KEY = "safe_pose_mode"
SAFE_MODE_Z_LIFT = "z_lift_only"

# Which A/B layout the payload claims. The validator enforces the claimed shape
# exactly, so a lost or misrouted block still aborts before the first motion.
CAPTURE_PROTOCOL_KEY = "capture_protocol"
PROTOCOL_PER_SET_AB = "per_set_AB"          # every set: B sweep + A placement
PROTOCOL_A_SETS_B_STATION = "A_sets_plus_B_station"  # A-only sets, one terminal B station
CAPTURE_PROTOCOLS = (PROTOCOL_PER_SET_AB, PROTOCOL_A_SETS_B_STATION)


def validate_safe_joint_config(data):
    """Fail closed: empty and gripped payload safe poses are both mandatory.

    The single exception is an explicit ``safe_pose_mode: "z_lift_only"`` in the
    payload, which returns None so the executor retracts along +Z instead of
    routing through a taught safe pose. That opt-out has to be written into the
    waypoint file on purpose — a missing key still aborts — so a session shot
    without safe poses is identifiable from its own artifacts afterwards.
    """
    if not isinstance(data, dict):
        raise ValueError("waypoint payload must be an object")
    if data.get(SAFE_MODE_KEY) == SAFE_MODE_Z_LIFT:
        return None
    empty = validate_joint_vector(data.get(SAFE_EMPTY_KEY), SAFE_EMPTY_KEY)
    gripped = validate_joint_vector(data.get(SAFE_GRIPPED_KEY), SAFE_GRIPPED_KEY)
    return {SAFE_EMPTY_KEY: empty, SAFE_GRIPPED_KEY: gripped}


def shortest_joint_error_deg(actual, target):
    """Per-axis absolute angular error, accounting for equivalent +/-360 deg."""
    a = validate_joint_vector(actual, "actual_joints")
    b = validate_joint_vector(target, "target_joints")
    errors = []
    for av, bv in zip(a, b):
        delta = abs(av - bv) % 360.0
        errors.append(min(delta, 360.0 - delta))
    return errors


def validate_waypoint_semantics(data):
    """Reject mislabeled A/B records before the robot performs any motion.

    Two layouts are legal, and the payload must say which one it is via
    ``capture_protocol``.  A missing key means the legacy ``per_set_AB`` shape,
    so an old file keeps its old contract.  Neither branch accepts a set whose
    blocks merely "look plausible": the claimed shape is checked exactly, so a
    dropped B station or a half-written set aborts before the robot moves.
    """
    if not isinstance(data, dict):
        raise ValueError("waypoint payload must be an object")
    protocol = data.get(CAPTURE_PROTOCOL_KEY, PROTOCOL_PER_SET_AB)
    if protocol not in CAPTURE_PROTOCOLS:
        raise ValueError(
            "unknown {} {!r} (expected one of {})".format(
                CAPTURE_PROTOCOL_KEY, protocol, ", ".join(CAPTURE_PROTOCOLS))
        )
    waypoints = data.get("waypoints")
    if not isinstance(waypoints, list) or not waypoints:
        raise ValueError("waypoints must be a non-empty list")
    seen_capture_indices = set()
    blocks_by_set = {}
    set_order = []
    for idx, wp in enumerate(waypoints):
        if not isinstance(wp, dict):
            raise ValueError("waypoints[{}] must be an object".format(idx))
        block = wp.get("capture_block")
        if block not in ("A_placement", "B_eyetohand"):
            raise ValueError("waypoints[{}] has unknown capture_block {!r}".format(idx, block))
        expected = block == "B_eyetohand"
        if wp.get("cube_gripped") is not expected:
            raise ValueError(
                "waypoints[{}] block {} requires cube_gripped={}".format(idx, block, expected)
            )
        capture_index = wp.get("capture_index")
        if capture_index is None or capture_index in seen_capture_indices:
            raise ValueError("waypoints[{}] capture_index is missing or duplicated".format(idx))
        seen_capture_indices.add(capture_index)
        set_index = wp.get("set_index")
        if set_index is None:
            raise ValueError("waypoints[{}].set_index is required".format(idx))


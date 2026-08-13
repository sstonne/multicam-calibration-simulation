"""Copy this file when connecting an author's SOTA repository."""

from __future__ import annotations

from .sota_simulation import CalibrationInput, CalibrationResult


class ExternalMethodAdapter:
    name = "replace_with_method_name"

    def calibrate(self, data: CalibrationInput) -> CalibrationResult:
        """Convert common input -> author format, run code, convert output back.

        Required output convention:
          * T_base_camera[name]: camera frame -> robot base frame
          * T_gripper_board: board frame -> gripper frame, or None if unavailable
          * translation unit: metre

        Ground truth is intentionally not present in ``data``.
        """
        # 1. Convert data.board_points, data.cameras, data.T_base_gripper,
        #    and data.observations to the external repository's input format.
        # 2. Call the original method without changing its objective.
        # 3. Convert its transforms and units to the convention above.
        raise NotImplementedError("Implement the external repository adapter")


def create_method() -> ExternalMethodAdapter:
    return ExternalMethodAdapter()

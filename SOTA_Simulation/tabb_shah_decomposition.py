#!/usr/bin/env python3
"""Shah(2013) closed-form과 Tabb(2017) separable이 어디서 갈라지는지 분해한다.

opencv_multicam_evaluation.py의 통합 지표만으로는 "Tabb가 Shah보다 좋다"까지만
보이고 **왜** 좋은지가 보이지 않는다. 이 스크립트는 같은 trajectory/noise
sample/split에서 두 방법을 카메라 종류별로 나눠 비교해 그 원인을 분리한다.

확인하는 것:

1. Rotation 단계가 같은가
   Tabb separable의 1단계(Eq. 7 / Eq. 11)는
       argmin sum ||R_A,i R_X - R_Z R_B,i||_F^2
   인데, Shah의 Kronecker product + SVD는 **같은 목적함수의 closed-form 해**다.
   게다가 이 목적함수는 A/B의 역할 교환이나 동시 역변환에 대해 불변이다
   (Frobenius norm이 orthogonal 곱과 transpose에 불변이므로).
   따라서 두 방법의 rotation 추정은 수치오차 범위에서 같아야 한다.

2. Translation 단계는 왜 다른가
   Eq. 8 / Eq. 12의 linear least squares는 residual이 **어느 frame에서
   측정되는가**에 따라 값이 달라진다. Shah wrapper와 Tabb wrapper는 논문
   정의가 서로 달라 A_i / B_i 배정이 다르고, 그 결과:
     - eye-to-hand(fixed camera): 두 배정이 우연히 일치 -> 결과도 같다
     - eye-in-hand(wrist camera): 두 배정이 서로 역 -> 결과가 갈린다
   즉 Tabb separable이 Shah보다 나은 부분은 "iterative라서"가 아니라
   "translation residual을 camera frame에서 재기 때문"이다.

3. iterative가 실제로 보태는 것
   separable -> simultaneous로 갈 때만 rotation 추정까지 바뀐다. closed-form
   으로는 만들 수 없는 부분이 바로 이것이다.

실행:
    python SOTA_Simulation/tabb_shah_decomposition.py --trials 30
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
os.environ.setdefault("QT_API", "PyQt5")

import numpy as np

from SOTA_Simulation.opencv_multicam_evaluation import (
    DEFAULT_HELDOUT,
    DEFAULT_NOISE_MM,
    provenance,
)
from SOTA_Simulation.shah_solver import (
    solve_shah_eye_in_hand,
    solve_shah_eye_to_hand,
)
from SOTA_Simulation.sota_simulation import generate_case, load_config, pose_error
from SOTA_Simulation.tabb_solver import (
    TabbConfig,
    solve_tabb_eye_in_hand,
    solve_tabb_eye_to_hand,
)
from SOTA_Simulation.tsai_combined_demo import build_eye_in_hand_session
from SOTA_Simulation.tsai_noise_sweep import (
    make_corner_observations,
    prepare_fixed_inputs,
)

VARIANTS = {
    "shah": None,
    "tabb_c1_sep": TabbConfig(cost="c1", separable=True),
    "tabb_c2_sep": TabbConfig(cost="c2", separable=True),
    "tabb_c1_sim": TabbConfig(cost="c1", separable=False),
    "tabb_c2_sim": TabbConfig(cost="c2", separable=False),
}


def run(trials: int, seed: int, noise_levels, heldout_events):
    config, config_path = load_config(
        Path(__file__).with_name("config.example.json"))
    config["simulation"]["pixel_noise_sigma"] = 0.0
    config["simulation"]["corner_dropout_probability"] = 0.0
    config["simulation"]["camera_event_dropout_probability"] = 0.0
    case = generate_case(config, config_path)
    wrist = build_eye_in_hand_session(
        int(config["simulation"]["number_of_events"]))
    fixed = prepare_fixed_inputs(case)
    board_points = case.data.board_points
    event_count = len(wrist["T_base_gripper"])
    heldout = set(heldout_events)
    train = [event for event in range(event_count) if event not in heldout]

    samples = {
        name: {group: {noise: [] for noise in noise_levels}
               for group in ("wrist", "fixed")}
        for name in VARIANTS
    }
    for name in VARIANTS:
        for group in ("wrist", "fixed"):
            for noise in noise_levels:
                samples[name][group][noise] = {"translation": [], "rotation": []}

    for trial in range(trials):
        # opencv_multicam_evaluation.run_evaluation과 동일한 소비 순서를 지켜
        # 같은 noise sample을 얻는다.
        rng = np.random.default_rng(seed + trial)
        wrist_unit = rng.standard_normal((event_count, len(board_points), 3))
        fixed_unit = {
            name: rng.standard_normal((len(item["visual"]), len(board_points), 3))
            for name, item in fixed.items()
        }
        for noise in noise_levels:
            wrist_poses, _ = make_corner_observations(
                wrist["T_wrist_board"], board_points, wrist_unit, noise)
            fixed_poses = {
                name: make_corner_observations(
                    item["visual"], board_points, fixed_unit[name], noise)[0]
                for name, item in fixed.items()
            }
            robot_wrist = [wrist["T_base_gripper"][event] for event in train]
            visual_wrist = [wrist_poses[event] for event in train]

            for name, config_variant in VARIANTS.items():
                if config_variant is None:
                    estimate = solve_shah_eye_in_hand(
                        robot_wrist, visual_wrist).T_gripper_wrist
                else:
                    estimate = solve_tabb_eye_in_hand(
                        robot_wrist, visual_wrist, config_variant).T_gripper_wrist
                translation, rotation = pose_error(
                    estimate, wrist["T_gripper_wrist_truth"])
                record = samples[name]["wrist"][noise]
                record["translation"].append(translation)
                record["rotation"].append(rotation)

            for camera_name, item in fixed.items():
                robot = [item["robot"][event] for event in train]
                visual = [fixed_poses[camera_name][event] for event in train]
                truth = case.truth.T_base_camera[camera_name]
                for name, config_variant in VARIANTS.items():
                    if config_variant is None:
                        estimate = solve_shah_eye_to_hand(
                            robot, visual).T_base_fixed_i
                    else:
                        estimate = solve_tabb_eye_to_hand(
                            [camera_name], [robot], [visual], config_variant,
                        ).T_base_fixed[camera_name]
                    translation, rotation = pose_error(estimate, truth)
                    record = samples[name]["fixed"][noise]
                    record["translation"].append(translation)
                    record["rotation"].append(rotation)

    return samples, train, sorted(heldout)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=30)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--noise-mm", nargs="+", type=float,
                        default=list(DEFAULT_NOISE_MM))
    parser.add_argument("--heldout-events", nargs="+", type=int,
                        default=list(DEFAULT_HELDOUT))
    parser.add_argument("--output", default=None)
    args = parser.parse_args()

    samples, train, heldout = run(
        args.trials, args.seed, args.noise_mm, args.heldout_events)

    report = {
        "trials": args.trials,
        "seed": args.seed,
        "noise_levels_mm": args.noise_mm,
        "calibration_events": train,
        "heldout_events": heldout,
        "provenance": provenance(),
        "groups": {
            "wrist": "eye-in-hand, T_gripper_wrist vs GT",
            "fixed": "eye-to-hand, T_base_camera vs GT (3 cameras pooled)",
        },
        "summary": {},
    }
    for name, groups in samples.items():
        report["summary"][name] = {
            group: {
                str(float(noise)): {
                    "translation_error_mm_mean": float(np.mean(values["translation"])),
                    "translation_error_mm_std": float(np.std(values["translation"], ddof=1)),
                    "rotation_error_deg_mean": float(np.mean(values["rotation"])),
                    "rotation_error_deg_std": float(np.std(values["rotation"], ddof=1)),
                }
                for noise, values in noises.items()
            }
            for group, noises in groups.items()
        }

    # 두 방법 사이의 최대 편차 — rotation 단계가 같은 문제인지 확인하는 핵심 숫자
    def max_gap(first: str, second: str) -> dict:
        entry = {}
        for group in ("wrist", "fixed"):
            rotation_gap, translation_gap = 0.0, 0.0
            for noise in args.noise_mm:
                a = samples[first][group][noise]
                b = samples[second][group][noise]
                rotation_gap = max(rotation_gap, float(np.max(np.abs(
                    np.asarray(a["rotation"]) - np.asarray(b["rotation"])))))
                translation_gap = max(translation_gap, float(np.max(np.abs(
                    np.asarray(a["translation"]) - np.asarray(b["translation"])))))
            entry[group] = {
                "max_abs_rotation_gap_deg": rotation_gap,
                "max_abs_translation_gap_mm": translation_gap,
            }
        return entry

    deviation = {name: max_gap("shah", name)
                 for name in VARIANTS if name != "shah"}
    report["max_deviation_from_shah"] = deviation
    # c1 separable과 c2 separable의 회전 단계도 같은 문제인지 (문서 5.4절 주장)
    report["max_deviation_between_variants"] = {
        "tabb_c1_sep_vs_tabb_c2_sep": max_gap("tabb_c1_sep", "tabb_c2_sep"),
        "tabb_c1_sim_vs_tabb_c2_sim": max_gap("tabb_c1_sim", "tabb_c2_sim"),
    }

    header = f"{'method':14s}" + "".join(
        f"{f'{n:g}mm':>26s}" for n in args.noise_mm)
    for group in ("wrist", "fixed"):
        print(f"\n=== {group} ({report['groups'][group]}) ===")
        print(header)
        for name in VARIANTS:
            row = f"{name:14s}"
            for noise in args.noise_mm:
                values = report["summary"][name][group][str(float(noise))]
                row += (f"{values['translation_error_mm_mean']:14.4f} mm"
                        f"{values['rotation_error_deg_mean']:9.5f}°")
            print(row)

    print("\n=== Shah 대비 최대 편차 (전체 trial/noise) ===")
    for name, entry in deviation.items():
        for group, values in entry.items():
            print(f"{name:14s} {group:6s} rotation {values['max_abs_rotation_gap_deg']:.3e}°"
                  f"  translation {values['max_abs_translation_gap_mm']:.3e} mm")

    print("\n=== c1 / c2 사이 최대 편차 (전체 trial/noise) ===")
    for label, entry in report["max_deviation_between_variants"].items():
        for group, values in entry.items():
            print(f"{label:30s} {group:6s} "
                  f"rotation {values['max_abs_rotation_gap_deg']:.3e}°"
                  f"  translation {values['max_abs_translation_gap_mm']:.3e} mm")

    if args.output:
        output = Path(args.output).resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("w", encoding="utf-8") as handle:
            json.dump(report, handle, indent=2, ensure_ascii=False)
        print(f"\nwrote {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

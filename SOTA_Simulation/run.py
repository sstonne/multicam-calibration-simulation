#!/usr/bin/env python3
"""Generate one shared dataset, run a method adapter, evaluate, and visualize."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from SOTA_Simulation.evaluate import evaluate_result, noiseless_pass
from SOTA_Simulation.methods import load_method
from SOTA_Simulation.sota_simulation import (
    export_input_npz,
    generate_case,
    load_config,
)
from SOTA_Simulation.visualize import create_visualizations


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        default=str(Path(__file__).with_name("config.example.json")),
    )
    parser.add_argument(
        "--method",
        default="joint_reference",
        help="joint_reference, independent_reference, or module.path:factory",
    )
    parser.add_argument(
        "--output",
        default=str(Path(__file__).with_name("outputs") / "smoke"),
    )
    parser.add_argument("--no-viz", action="store_true")
    parser.add_argument(
        "--show",
        action="store_true",
        help="Open interactive visualization windows in addition to saving PNG files",
    )
    args = parser.parse_args()

    config, config_path = load_config(args.config)
    case = generate_case(config, config_path)
    method = load_method(args.method)
    result = method.calibrate(case.data)
    report = evaluate_result(case, result)

    output_directory = Path(args.output).resolve()
    output_directory.mkdir(parents=True, exist_ok=True)
    export_input_npz(case.data, output_directory / "calibration_input_no_gt.npz")
    tolerance = config["noiseless_tolerance"]
    is_noiseless = (
        float(config["simulation"].get("pixel_noise_sigma", 0.0)) == 0.0
        and float(config["simulation"].get("corner_dropout_probability", 0.0)) == 0.0
        and float(config["simulation"].get(
            "camera_event_dropout_probability", 0.0)) == 0.0
    )
    report["noiseless_gate"] = {
        "applicable": is_noiseless,
        "passed": noiseless_pass(report, tolerance) if is_noiseless else None,
        "tolerance": tolerance,
    }
    with (output_directory / "report.json").open("w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, ensure_ascii=False)
    if not args.no_viz:
        create_visualizations(
            case, result, report, output_directory, show=bool(args.show)
        )

    print(json.dumps(report, indent=2, ensure_ascii=False))
    print(f"\nOutput: {output_directory}")
    if is_noiseless and not report["noiseless_gate"]["passed"]:
        print("Noiseless recovery gate: FAIL")
        return 2
    print("Noiseless recovery gate: PASS" if is_noiseless else "Noise run complete")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

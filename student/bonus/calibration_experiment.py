"""Reproducible short-run camera extrinsic translation sensitivity experiment.

Run from the repository root with the project environment active:
    python student/bonus/calibration_experiment.py

Each trial uses an isolated temporary student root so the official grading
artifacts are never overwritten. The camera translation is perturbed along the
camera-left axis while all other inputs and the RNG seed stay fixed.
"""

from __future__ import annotations

import csv
import json
import os
import shutil
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
PLATFORM = ROOT / "platform"
STUDENT = ROOT / "student"
DATA = ROOT / "data"
os.environ["FUSION_LAB_PLATFORM"] = str(PLATFORM)
sys.path.insert(0, str(PLATFORM))

from fusion_lab.scripts import run_lab


SEGMENT = "training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord"
OFFSETS_METRES = (0.0, 0.05, 0.10)
FRAME_COUNT = 20


def run_trial(offset_metres: float, trial_root: Path) -> dict[str, object]:
    """Run fused tracking with a left-axis camera extrinsic offset."""
    student_root = trial_root / "student"
    shutil.copytree(STUDENT / "workspace", student_root / "workspace")
    (student_root / "artifacts").mkdir(parents=True)
    config_path = trial_root / "paths.yaml"
    config_path.write_text(
        "\n".join(
            (
                f"waymo_dir: {DATA / 'Waymo'}",
                f"weights_dir: {DATA / 'weights'}",
                f"segment: {SEGMENT}",
                "frame_start: 0",
                f"frame_end: {FRAME_COUNT - 1}",
            )
        ),
        encoding="utf-8",
    )

    os.environ["DAY23_STUDENT_ROOT"] = str(student_root)
    accepted_camera_innovations: list[float] = []
    camera_gate_counts = {"passed": 0, "rejected": 0}
    base_load_workspace = run_lab.load_workspace

    def load_instrumented_workspace():
        modules = base_load_workspace()
        kalman = modules["kalman"]
        association = modules["association"]
        original_gate = association.chi2_gate
        original_associate = association.associate_and_update

        def count_camera_gate(distance, sensor):
            accepted = original_gate(distance, sensor)
            if sensor.name == "camera":
                camera_gate_counts["passed" if accepted else "rejected"] += 1
            return accepted

        def capture_camera_updates(manager, measurements, filter_obj, sensor):
            if sensor.name != "camera":
                return original_associate(manager, measurements, filter_obj, sensor)
            original_update = filter_obj.update

            def record_update(track, measurement):
                residual = kalman.innovation(track.x, measurement)
                accepted_camera_innovations.append(float(np.linalg.norm(residual)))
                original_update(track, measurement)

            filter_obj.update = record_update
            try:
                return original_associate(manager, measurements, filter_obj, sensor)
            finally:
                filter_obj.update = original_update

        association.chi2_gate = count_camera_gate
        association.associate_and_update = capture_camera_updates
        return modules

    original_loader = run_lab.load_workspace
    original_sensor_init = run_lab.Sensor.__init__

    def perturbed_sensor_init(sensor, name, calibration, camera_fusion):
        original_sensor_init(sensor, name, calibration, camera_fusion)
        if name == "camera" and offset_metres:
            transform = np.asarray(sensor.veh_to_sens, dtype=float).copy()
            transform[1, 3] += offset_metres
            sensor.veh_to_sens = np.asmatrix(transform)
            sensor.sens_to_veh = np.asmatrix(np.linalg.inv(transform))

    run_lab.load_workspace = load_instrumented_workspace
    run_lab.Sensor.__init__ = perturbed_sensor_init
    try:
        metrics = run_lab.run(
            config_path,
            fusion_mode="fused",
            max_frames=FRAME_COUNT,
            artifact_suffix="",
            seed=0,
        )
    finally:
        run_lab.load_workspace = original_loader
        run_lab.Sensor.__init__ = original_sensor_init
        os.environ.pop("DAY23_STUDENT_ROOT", None)

    camera_summary = {
        "accepted_camera_updates": len(accepted_camera_innovations),
        "accepted_innovation_median_px": float(np.median(accepted_camera_innovations))
        if accepted_camera_innovations
        else None,
        "accepted_innovation_p95_px": float(np.percentile(accepted_camera_innovations, 95))
        if accepted_camera_innovations
        else None,
        "camera_gate_passed": camera_gate_counts["passed"],
        "camera_gate_rejected": camera_gate_counts["rejected"],
    }
    tracking = metrics["tracking"]["fused"]
    return {
        "camera_left_translation_offset_m": offset_metres,
        **camera_summary,
        "rmse_m": tracking["rmse"],
        "matches": tracking["matches"],
        "ghost_track_frames": tracking["ghost_track_frames"],
        "missed_gt_frames": tracking["missed_gt_frames"],
        "frames": metrics["frames"],
        "segment": metrics["segment"],
        "seed": metrics["seed"],
    }


def main() -> None:
    rows = []
    temp_root = Path.home() / "AppData/Local/Temp/opencode"
    with tempfile.TemporaryDirectory(prefix="day23_calibration_", dir=temp_root) as temp:
        base = Path(temp)
        for index, offset in enumerate(OFFSETS_METRES):
            rows.append(run_trial(offset, base / f"trial_{index}"))

    output = STUDENT / "bonus"
    output.mkdir(parents=True, exist_ok=True)
    csv_path = output / "calibration_results.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (output / "calibration_results.json").write_text(
        json.dumps(rows, indent=2), encoding="utf-8"
    )
    print(json.dumps(rows, indent=2))
    print(f"Saved calibration results to {csv_path}")


if __name__ == "__main__":
    main()

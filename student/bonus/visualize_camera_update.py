"""Capture two reproducible BEV plots of accepted camera EKF corrections.

Run from repository root using the lab virtual environment. The Waymo run uses
an isolated temporary student root and does not overwrite grading artifacts.
"""

from __future__ import annotations

import json
import os
import shutil
import sys
import tempfile
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
PLATFORM = ROOT / "platform"
STUDENT = ROOT / "student"
DATA = ROOT / "data"
SEGMENT = "training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord"
FRAME_COUNT = 30

os.environ["FUSION_LAB_PLATFORM"] = str(PLATFORM)
sys.path.insert(0, str(PLATFORM))

from fusion_lab import tracking_params
from fusion_lab.scripts import run_lab


def main() -> None:
    updates: list[dict[str, object]] = []
    original_loader = run_lab.load_workspace

    def load_instrumented_workspace():
        modules = original_loader()
        association = modules["association"]
        original_associate = association.associate_and_update

        def capture_camera_updates(manager, measurements, filter_obj, sensor):
            if sensor.name != "camera":
                return original_associate(manager, measurements, filter_obj, sensor)

            current_frame = (
                int(round(measurements[0].t / tracking_params.dt))
                if measurements
                else -1
            )
            original_update = filter_obj.update

            def record_update(track, measurement):
                prior_state = np.asarray(track.x, dtype=float).copy()
                try:
                    prior_projection = np.asarray(
                        measurement.sensor.get_hx(track.x), dtype=float
                    ).reshape(2)
                    measured_pixel = np.asarray(measurement.z, dtype=float).reshape(2)
                    prior_residual = float(np.linalg.norm(measured_pixel - prior_projection))
                except (ValueError, FloatingPointError):
                    prior_residual = None
                    measured_pixel = np.asarray(measurement.z, dtype=float).reshape(2)

                original_update(track, measurement)
                posterior_state = np.asarray(track.x, dtype=float).copy()
                try:
                    posterior_projection = np.asarray(
                        measurement.sensor.get_hx(track.x), dtype=float
                    ).reshape(2)
                    posterior_residual = float(
                        np.linalg.norm(measured_pixel - posterior_projection)
                    )
                except (ValueError, FloatingPointError):
                    posterior_residual = None
                updates.append(
                    {
                        "frame": current_frame,
                        "track_id": int(track.id),
                        "before_xy": prior_state[:2, 0].tolist(),
                        "after_xy": posterior_state[:2, 0].tolist(),
                        "pixel_measurement": measured_pixel.tolist(),
                        "pixel_residual_before": prior_residual,
                        "pixel_residual_after": posterior_residual,
                    }
                )

            filter_obj.update = record_update
            try:
                return original_associate(manager, measurements, filter_obj, sensor)
            finally:
                filter_obj.update = original_update

        association.associate_and_update = capture_camera_updates
        return modules

    temp_root = Path.home() / "AppData/Local/Temp/opencode"
    with tempfile.TemporaryDirectory(prefix="day23_visualization_", dir=temp_root) as temp:
        trial = Path(temp)
        student_root = trial / "student"
        shutil.copytree(STUDENT / "workspace", student_root / "workspace")
        (student_root / "artifacts").mkdir(parents=True)
        config = trial / "paths.yaml"
        config.write_text(
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
        run_lab.load_workspace = load_instrumented_workspace
        try:
            run_lab.run(config, fusion_mode="fused", max_frames=FRAME_COUNT, seed=0)
        finally:
            run_lab.load_workspace = original_loader
            os.environ.pop("DAY23_STUDENT_ROOT", None)

    if not updates:
        raise RuntimeError("No camera-associated EKF updates occurred in the selected frames")

    output = STUDENT / "bonus"
    output.mkdir(parents=True, exist_ok=True)
    (output / "camera_update_examples.json").write_text(
        json.dumps(updates, indent=2), encoding="utf-8"
    )
    ranked = sorted(
        updates,
        key=lambda item: np.linalg.norm(
            np.asarray(item["after_xy"]) - np.asarray(item["before_xy"])
        ),
        reverse=True,
    )
    selected = []
    seen_frames = set()
    for item in ranked:
        frame = item["frame"]
        if frame not in seen_frames:
            selected.append(item)
            seen_frames.add(frame)
        if len(selected) == 2:
            break
    if len(selected) < 2:
        raise RuntimeError("Need camera corrections from at least two distinct frames for evidence")

    for number, item in enumerate(selected, start=1):
        before = np.asarray(item["before_xy"], dtype=float)
        after = np.asarray(item["after_xy"], dtype=float)
        fig, ax = plt.subplots(figsize=(8, 6), constrained_layout=True)
        ax.scatter(*before, marker="x", s=100, label="Track before camera update")
        ax.scatter(*after, marker="o", s=80, label="Track after camera update")
        ax.annotate(
            "",
            xy=after,
            xytext=before,
            arrowprops={"arrowstyle": "->", "linewidth": 2, "color": "tab:red"},
        )
        displacement = float(np.linalg.norm(after - before))
        residual_before = item["pixel_residual_before"]
        residual_after = item["pixel_residual_after"]
        ax.set_title(
            f"Frame {item['frame']} — track {item['track_id']} camera EKF correction\n"
            f"BEV shift={displacement:.3f} m; pixel residual "
            f"{residual_before:.1f} → {residual_after:.1f} px"
        )
        ax.set_xlabel("Vehicle x (m)")
        ax.set_ylabel("Vehicle y (m)")
        ax.grid(True, alpha=0.3)
        ax.axis("equal")
        ax.legend(loc="best")
        fig.savefig(output / f"camera_update_bev_{number}.png", dpi=160)
        plt.close(fig)

    print(json.dumps({"accepted_camera_updates": len(updates), "examples": selected}, indent=2))


if __name__ == "__main__":
    main()

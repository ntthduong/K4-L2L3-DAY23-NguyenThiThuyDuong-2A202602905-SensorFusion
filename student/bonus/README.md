# Bonus evidence

## Calibration sensitivity (+4)

Run `python student/bonus/calibration_experiment.py` from the repository root.
The script runs the official fused pipeline on frames 0–19 with seed 0 at three
camera-left extrinsic translation offsets (0, 0.05, 0.10 m). Each trial uses a
temporary student root so it cannot overwrite the official grading artifacts.
Results, including accepted camera-update innovations and chi-square gate pass /
reject counts, are in `calibration_results.csv` and `.json`.

This is a short, segment-specific sensitivity experiment. The positive offsets
happen to reduce accepted residuals and RMSE in this sample; this does not mean
miscalibration generally improves tracking. The gate rejects incompatible
track/measurement hypotheses using normalized innovation, while some pairs can
remain within the gate after a calibration shift.

## BEV visualization (+3)

Run `python student/bonus/visualize_camera_update.py`. It runs 30 frames in an
isolated temporary student root and records actual accepted camera EKF updates.
`camera_update_bev_1.png` and `camera_update_bev_2.png` show the state before and
after the camera correction; `camera_update_examples.json` contains the track
positions, camera pixels, and pixel residuals used in those figures.

## CVAT export / inspection (+3)

Not completed: no CVAT instance/account was available for importing and capturing
the required visual inspection evidence.

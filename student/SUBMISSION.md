# Báo cáo bài nộp — Day 23 Sensor Fusion Lab

> Điền file này rồi commit. Cách nộp: [hướng dẫn nộp](../SUBMISSION.md).

## Thông tin học viên

- Họ tên: Nguyễn Thị Thùy Dương
- MSSV: 2A202602905
- Email:
- Link repo (fork): https://github.com/ntthduong/K4-L2L3-DAY23-NguyenThiThuyDuong-2A202602905-SensorFusion
- Commit hash nộp (`git rev-parse HEAD`):

## Tóm tắt kết quả

- `fusion_mode=compare`, `frames=[0,198]` (199 frames), `segment=training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord`, `seed=0`.
- Detection: `precision=0.970093`, `recall=0.700405`, `tp=519`, `fp=16`, `fn=222`.
- LiDAR: `rmse=0.150323 m`, `matches=502`, `sum_sq_err=11.343649 m²`, `ghost_track_frames=0`, `missed_gt_frames=239`, `mean_confirmed_tracks=2.522613`.
- Fused: `rmse=0.135867 m`, `matches=502`, `sum_sq_err=9.266812 m²`, `ghost_track_frames=0`, `missed_gt_frames=239`, `mean_confirmed_tracks=2.522613`.
- So sánh: fused giảm RMSE khoảng `0.014456 m` so với LiDAR, trong khi matches, ghost và misses không đổi. Cả hai mode không có ghost; 239 missed GT frames vẫn cần đọc cùng 502 matches, không nên chỉ nhìn RMSE. Theo rubric, `precision_track=502/(502+0)=1.0`, `coverage=502/519≈0.9672`; cả hai vượt ngưỡng và RMSE ≤0.45 m.

Chạy từ root repo:

```bash
fusion-run-lab --config student/config/paths.yaml --fusion compare --seed 0
```

`rmse = sqrt(sum_sq_err/matches)` trên vị trí 3D của confirmed tracks ghép
một-một với GT xe trong cửa sổ BEV, gate XY **2.0 m**; `null` nếu không có cặp.
Camera dùng tâm hộp 2D ground-truth FRONT có nhiễu seeded, **không** dùng camera
detector. Kết quả này không đo hiệu quả một perception system độc lập với GT.

`grade_run.log` là JSONL, mỗi `(mode,frame)` đúng một record với các trường:
`mode`, `frame`, `det_tp`, `det_fp`, `det_fn`, `valid_gt`, `confirmed`, `matches`,
`sum_sq_err`, `ghosts`, `misses`. Đảm bảo `matches+ghosts==confirmed` và
`matches+misses==valid_gt`; tổng/trung bình record phải khớp `metrics.json`.
File per-mode `metrics_lidar.json`, `metrics_fused.json`, `grade_run_lidar.log`,
`grade_run_fused.log` được giữ để đối chiếu.

## Giải thích ngắn (Parts E–H — tự viết)

1. LiDAR cho đo vị trí 3D trong hệ tọa độ cảm biến (`z` có 3 thành phần); EKF đổi sang vehicle frame khi khởi tạo. Camera cho pixel `(u,v)` (`z` có 2 thành phần), với mô hình pinhole phi tuyến `h(x)`. Vì vậy `R` của LiDAR là covariance 3×3 theo mét, còn camera là covariance 2×2 theo pixel; `H` camera là Jacobian của phép chiếu.
2. Euclidean distance không tính uncertainty và độ tin cậy theo hướng của track/measurement. Mahalanobis chuẩn hóa innovation theo `S = HPHᵀ + R`; chi-square gate loại cặp quá xa so với uncertainty ở đúng số chiều đo, giảm ghép nhầm trước greedy assignment.
3. Đây là **track-then-fuse**: `run_lab.py` predict các track một lần, gọi association/update LiDAR, rồi association/update camera trên cùng track khi chạy fused. Camera không tạo track. `grade_run.log` có record theo mode/frame để so sánh kết quả; thứ tự xử lý thể hiện trực tiếp trong `platform/fusion_lab/scripts/run_lab.py`.
4. Calibration lệch làm phép đổi vehicle→camera sai, nên pixel dự đoán `h(x)` lệch có hệ thống khỏi pixel đo; innovation/residual tăng và có thể vượt chi-square gate, khiến cặp camera bị từ chối. Nếu độ lệch còn nằm trong gate, EKF vẫn có thể bị kéo sai.
5. Khi danh sách đo rỗng, không thể suy ra sensor từ measurement nhưng vẫn phải biết đây là lượt LiDAR hay camera để áp dụng lifecycle đúng. `TrackManager.manage_tracks(..., sensor)` chỉ score miss/init/delete ở lượt LiDAR; LiDAR hit/miss quyết định tồn tại. Camera chỉ cập nhật state EKF, không đổi score, không sinh track và không xóa track.
6. Track bắt đầu `initialized` với score `1/window`; hit LiDAR cộng `1/window` (tối đa 1), và score **lớn hơn** `confirmed_threshold` thì thành `confirmed`. Track confirmed giữ trạng thái khi miss. Xóa nếu `Pxx` hoặc `Pyy > max_P`; nếu confirmed thì thêm điều kiện `score < delete_threshold`; nếu chưa confirmed thì xóa khi `score <= 0`.

## Bonus (không bắt buộc)

Liệt kê phần bonus đã làm, file bằng chứng trong `student/bonus/` và kết quả chính
(xem [RUBRIC.md](../RUBRIC.md) mục 2). Không làm thì ghi "Không".

- **Phân tích calibration (+4, thử nghiệm ngắn 20 frame, frames 0–19, seed 0):** chạy `student/bonus/calibration_experiment.py`; offset tịnh tiến extrinsic theo trục camera-left, giữ nguyên các đầu vào khác. `calibration_results.csv/json` ghi innovation trước camera update đã được chấp nhận, số gate pass/reject và RMSE fused.

  | Offset extrinsic | Median innovation accepted (px) | P95 (px) | Gate pass / reject | RMSE (m) | Matches / ghost / miss |
  |---:|---:|---:|---:|---:|---:|
  | 0.00 m | 18.68 | 30.08 | 40 / 363 | 0.154703 | 32 / 0 / 8 |
  | +0.05 m | 15.54 | 26.18 | 40 / 363 | 0.146991 | 32 / 0 / 8 |
  | +0.10 m | 14.10 | 23.26 | 40 / 363 | 0.148794 | 32 / 0 / 8 |

  Trong mẫu ngắn này, lệch dương camera-left tình cờ làm innovation accepted và RMSE giảm; không thể suy ra calibration sai nói chung cải thiện tracking. Hướng lệch có thể bù residual sẵn có trong sample. Gate χ² theo Mahalanobis (2D, mức 0.995) loại 363 cặp ứng viên không tương thích nhưng vẫn nhận 40 cặp; gate xét residual đã chuẩn hóa theo covariance `S`, nên không phải mọi innovation pixel lớn đều bị loại. Kết quả chỉ là phân tích độ nhạy trên segment/frames đã ghi, không thay đổi calibration hoặc artifacts chính thức.
- **Trực quan hóa (+3):** `visualize_camera_update.py` sinh `camera_update_bev_1.png` và `camera_update_bev_2.png`, cùng dữ liệu trong `camera_update_examples.json`. Ví dụ track 0: frame 10 residual pixel giảm `28.45 → 3.87 px`; frame 11 giảm `29.59 → 4.05 px` sau camera EKF update.
- **CVAT (+3):** chưa có ảnh chụp import/kiểm tra trực quan trong CVAT.

## Khai báo sử dụng AI (bắt buộc)

Ghi rõ, kể cả khi không dùng ("Không dùng AI"). Xem [RULES.md](../RULES.md) mục 2.

- Công cụ đã dùng (ChatGPT, Copilot, Claude, …): ChatGPT (OpenCode).
- Dùng cho phần nào (hàm, câu hỏi, debug): Hỗ trợ triển khai và rà soát Parts E–H; hỗ trợ giải thích kỹ thuật, checklist repo, và scripts phân tích calibration/visualization bonus.
- Cách bạn đã kiểm tra lại (pytest, chạy Waymo, đối chiếu công thức): `pytest student/tests -q` (128 passed); chạy Waymo compare seed 0 trên frames 0–198; dùng `fusion_lab.evaluation.validate_metrics_records` xác thực 398 records trong `grade_run.log` khớp `metrics.json`.

## Checklist nộp

- [x] **Part E–H** trong `workspace/` đã implement; `pytest student/tests -q` không còn `failed`/`xfailed`
- [x] Part A–D: không bắt buộc sửa (hoặc ghi chú nếu bạn đã sửa)
- [x] Lần chạy chấm điểm: `--fusion compare --seed 0`, `frame_start: 0`, `frame_end: 198`
- [x] Đã commit `student/artifacts/metrics*.json` và `student/artifacts/grade_run*.log` (không sửa tay)
- [x] Đã điền đủ file này, gồm khai báo AI
- [x] Không commit dữ liệu Waymo, weights, `paths.yaml`, API key
- [ ] `python tools/check_submission.py` báo `KẾT QUẢ: SẴN SÀNG NỘP`
- [ ] Đã push và nộp link repo + commit hash trên LMS ([hướng dẫn nộp](../SUBMISSION.md))

# v0.5.17 — Traffic Model Converted from best.pt

- Replaced `app/src/main/assets/models/traffic_sign.tflite` with the TFLite export of the supplied Ultralytics checkpoint.
- Input tensor: `[1, 3, 640, 640]` (float32).
- Output tensor: `[1, 11, 8400]` (float32).
- Synchronized `TrafficSignDecoder.labels` and `traffic_sign_labels.txt` to the checkpoint's own class order.
- `stop` class ID: 5; `no_entry` class ID: not present in checkpoint; `turn_left` class ID: not present in checkpoint; `turn_right` class ID: 6.
- Export report: `docs/TFLITE_MODEL_REPORT_v0.5.17.json`.

The tensor/class-map checks verify packaging compatibility, not the trained model's recognition accuracy. Test on held-out labeled images and on the actual phone before relying on detections.

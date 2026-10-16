# Model Integration Checklist - Packaged Models

## 1. Free-space model

Asset: `models/free_space.onnx`

Runtime: ONNX Runtime Android.

Verified project metadata:

- YOLO26n semantic segmentation
- 256x256 input
- classes: `free`, `obstacle`, `unknown`
- semantic output includes an ArgMax-derived class map

Android pipeline:

`Bitmap -> 256x256 letterbox -> RGB/255 NCHW -> ONNX -> semantic mask -> crop non-padded region -> FreeSpaceAnalyzer`

The padding crop is important: a 16:9 camera frame letterboxed to 256x256 contains top/bottom padding, which must not be scored as road.

## 2. Traffic-sign model

Asset: `models/traffic_sign.tflite`

Runtime: LiteRT 1.4.2 / TFLite Interpreter API.

This v0.5.17 package replaces the previous traffic model with a TFLite export of the user-supplied `best.pt.zip` Ultralytics checkpoint. Its class order was read from checkpoint metadata and applied to both the Kotlin decoder and the label asset.

- Input: float32 `[1,3,640,640]` NCHW, RGB values normalized to 0..1.
- Output: float32 `[1,33,8400]`, corresponding to 4 box coordinates and 29 class scores for each candidate.
- Labels: `app/src/main/assets/models/traffic_sign_labels.txt`.
- The Kotlin decoder's labels are generated in the exact order reported by the checkpoint.

Android pipeline:

`Bitmap -> 640x640 letterbox -> RGB/255 NCHW -> TFLite -> raw detection tensor -> decoder -> class-aware NMS -> 3-frame filter -> NavigationController`

The runtime validates the supported input and output tensor shapes on startup. For this packaged model, the output is expected to use `[1,33,N]` layout.

## 3. Class-ID mapping

The model names are read from `best.pt` during export; IDs below reflect the converted checkpoint class order.

| ID | Class label | ID | Class label | ID | Class label |
|---:|---|---:|---|---:|---|
| 0 | `compulsory_right` |  |  |  |  |
| 1 | `no_stopping_or_parking` |  |  |  |  |
| 2 | `right_turn_prohibited` |  |  |  |  |
| 3 | `right_reverse_bend` |  |  |  |  |
| 4 | `side_road_right` |  |  |  |  |
| 5 | `stop` |  |  |  |  |
| 6 | `turn_right` |  |  |  |  |
|  |  |  |  |  |  |
|  |  |  |  |  |  |
|  |  |  |  |  |  |

## 4. Traffic-sign behavior mapping

| Detected class | Behavior |
|---|---|
| `stop` (class 5) | STOP with a 2-second phone-side latch, then cooldown |
| `no_entry` (class not present in checkpoint) | STOP if centered; veto a path toward the detected side |
| Other 27 classes (including `turn_left`, `turn_right`, `compulsory_left/right`, prohibited turns, speed limits, give-way, crossings, roundabout, etc.) | Detection/status only; no direct motor command |

The label `stop` is not, by itself, sufficient to validate safe road behavior. The robot's distance and obstacle-safety interlocks remain active. This model is intended for controlled robotics experiments, not road-vehicle control.

## 5. Temporal filtering

Candidate sign confidence threshold: 0.50.

Behavior confidence threshold: 0.75 for `stop` and `no_entry`.

Consecutive matching frames required: 3.

Live telemetry shows the raw class candidate separate from the temporally confirmed sign. A confirmed sign remains usable through brief missed detections but expires after 1.2 seconds. The virtual obstacle video deliberately does not run traffic-sign inference because it has no sign ground-truth labels; `SIGN --` in virtual mode does not test the sign model. Synthetic decoder tests validate the 29 class-ID mapping, not learned model accuracy.

## 6. Export/deployment rule

The traffic-sign model is TFLite/LiteRT and is integrated as TFLite/LiteRT. The free-space model is ONNX and is integrated through ONNX Runtime.

Any future model replacement must include a fresh input/output signature check, class-index confirmation, and validation on held-out images before it is allowed to issue behavior events.

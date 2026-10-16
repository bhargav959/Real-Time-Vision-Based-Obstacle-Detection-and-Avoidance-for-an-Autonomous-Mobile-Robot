# v0.5.18 - model integration fixes

Audit of the v0.5.17 TFLite model integration (flatbuffer, ONNX signature and the checkpoint metadata embedded in the .tflite were inspected directly):

- Verified OK: tflite SHA, input [1,3,640,640], output [1,11,8400], built-in ops only, normalised xywh + sigmoid scores, class order == checkpoint metadata (stop = 5), end2end = false, free-space ONNX [1,3,256,256] -> uint8 [1,256,256].
- Fixed `TrafficSignDecoderTest` (asserted the old 29-class map): now 7 classes on the real tensor shape, normalised boxes, label asset sync, old-shape rejection.
- Fixed stale 29-class text in models/README.txt, TrafficSignDecoder, TfliteModelRunner and the v0.5.17 profile doc.
- NO_ENTRY: the model has no such class. `TrafficSignDecoder.unsupportedBehaviorLabels` now exposes this and MainActivity logs a MODEL warning at start-up instead of failing silently. The controller logic is kept for a future retrain.
- Fixed three stale tests unrelated to the model (backoff distance at the 48 PWM cap, scan spin-up needing realism, scan-PWM escalation constants).

Not verified: detection accuracy on real images, on-device speed, and the Android build (no SDK here).

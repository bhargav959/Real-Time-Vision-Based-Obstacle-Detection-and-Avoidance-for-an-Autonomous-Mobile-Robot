PACKAGED MODEL ASSETS

free_space.onnx
  YOLO26n semantic segmentation.
  Classes: 0=free, 1=obstacle, 2=unknown.
  Runtime: ONNX Runtime Android.
  Input: 256x256 NCHW float RGB.

traffic_sign.tflite
  Custom Ultralytics YOLO detection model converted from the supplied best.pt checkpoint to TFLite. The class order is generated from checkpoint metadata and synchronized with TrafficSignDecoder.labels.
  Classes: 7 (checkpoint metadata names 0..6); see traffic_sign_labels.txt for the exact class-ID order.
  Not in this model: no_entry, turn_left, speed limits, traffic lights.
  Runtime: LiteRT 1.4.2 / TFLite Interpreter API.
  Expected input: float32 [1, 3, 640, 640]; camera pixels are RGB normalized to 0..1.
  Expected output: float32 [1, 11, 8400] = 4 box values (normalised xywh, /640) + 7 sigmoid class scores; decoded by TrafficSignDecoder.
  The Android runner validates the runtime input/output shapes during load.

Safety behavior mapping:
  stop (class 5) -> STOP with a 2-second phone-side latch.
  no_entry -> NOT AVAILABLE: the current model has no such class, so the NO_ENTRY veto cannot trigger
              (the app logs a MODEL warning at start-up). Retrain with the class to enable it.
  Other signs are detected and surfaced in status, but do not directly command turns.

Do not replace a model without re-checking its input/output signature and class-index mapping.

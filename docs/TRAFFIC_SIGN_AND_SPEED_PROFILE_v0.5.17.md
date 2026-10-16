# v0.5.17 — Traffic Model Export and Class Map

## Current speed profile

These are commanded PWM values, not measured road speeds. DC-motor response is nonlinear and depends on battery voltage, load, friction, wheel alignment and the motor driver's losses.

| Behavior | Current setting | Notes |
|---|---:|---|
| Base speed-controller ceiling | 40 PWM | Applied before command-type scaling and obstacle/geometry reductions |
| Straight `F` / `REV` | 80% of base, hard cap 32 PWM | Firmware repeats the 32 PWM cap |
| Normal steering `SL` / `SR` / `L` / `R` | 120% of base, hard cap 48 PWM | A 40 PWM base target becomes 48 PWM before ramping/safety caps |
| Normal slight-turn outer wheel | Minimum 40 PWM | Used with valid sonar so a differential remains after the inner wheel floor |
| Slight-turn outer wheel when sonar is `NO_ECHO` | Maximum 30 PWM | NO_ECHO safety cap takes precedence over the normal 40 PWM outer-wheel minimum; inner wheel floors at 25 PWM |
| Active scan | Start 31; max 41 PWM | The 41 PWM ceiling is used only with a valid sonar echo and adequate turn evidence |
| Active scan with `NO_ECHO` | Maximum 30 PWM | Same cap in Android, Arduino and virtual motor model |
| Motor floor | 25 PWM | Placeholder until tested with the chassis under load |
| Startup kick | 30 PWM for 40 ms | Only when requested PWM is below 28; active scan bypasses it |
| Bounded reverse backoff | 30 PWM for 250 ms | Distance is estimated, not measured; no rear sensor is fitted |

The 20% settings apply to the app's requested PWM target. They do not promise a 20% change in centimetres/second. The motor floor, the normal 40 PWM outer-wheel floor for slight turns, ramp-up, curvature reductions and safety caps may dominate a particular command.

## Traffic-sign class map

The class IDs below reflect the order stored in the converted `best.pt` checkpoint.

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
### What detection means in this release

- Input expected by the packaged model: float32 NCHW `[1,3,640,640]`; output expected: float32 `[1,11,8400]` (4 box values + 7 class scores).
- The decoder considers candidates at confidence >= 0.50 and uses class-aware NMS. The temporal filter requires 3 consecutive matching detections before a candidate becomes the confirmed sign.
- `SIGN CANDIDATE` in LIVE CAMERA telemetry is a raw current model result. `SIGN` is the three-frame confirmed result and is retained through brief misses for up to 1.2 seconds.
- Only `stop` changes navigation behavior with this model. The NO_ENTRY veto logic remains in the code but the model has no `no_entry` class, so it cannot trigger (a MODEL warning is logged at start-up). `turn_right`, `compulsory_right`, `right_turn_prohibited` and the other classes are detection/status only; they do not tell the robot to choose or execute that turn.
- The virtual obstacle video deliberately does not run sign inference because its synthetic scenes do not have labelled traffic-sign ground truth. `SIGN --` in virtual mode is not a test of the sign model.

### What has and has not been verified

A synthetic-tensor unit test passes for each of the 7 decoder class IDs on the real `[1,11,8400]` shape (and for normalised boxes), and a test checks that the label asset matches the code list and that `stop` is class 5. The class order was also checked against the metadata embedded in the .tflite. This verifies output decoding/class-index wiring, **not** that the trained neural network reliably recognizes real signs. Real model accuracy has not been tested against a labelled image set or confirmed on the target phone in this session.

### Manual phone test

1. Open the v0.5.16 project in Android Studio, sync/build it, install it on the phone, and open **LIVE CAMERA**. Do not use the virtual scenario to validate sign detection.
2. Hold one sign at a time in a well-lit view, facing the camera and large enough to be clearly visible; avoid other signs or text in the background for the first test.
3. Watch `SIGN CANDIDATE` for the raw label and confidence. If it stays on the same label through 3 inference updates, check `SIGN` for the confirmed label. A momentary candidate does not mean it is stable enough to be used as behavior.
4. Test each of the 7 classes separately: `compulsory_right`, `no_stopping_or_parking`, `right_turn_prohibited`, `right_reverse_bend`, `side_road_right`, `stop`, `turn_right`. Record misses and wrong labels rather than counting only successful detections.
5. Never test by allowing the robot to approach real traffic. The setup is a controlled robotics experiment, not a road-vehicle system.

## Release validation status

- Navigation package compile and speed-profile smoke check: passed.
- Virtual motion source compiled against local Android-graphics stubs; virtual NO_ECHO turn test confirmed lower yaw than a valid-sonar turn at the same requested PWM.
- Arduino sketch syntax check and speed-cap runtime smoke test with Arduino API stubs: passed.
- Decoder mapping tests: all 7 labels pass on the real tensor shape; a 29-class tensor is refused.
- Full Android Gradle build was not completed in this environment (the required Gradle 9.6+ distribution / Android SDK are not available here).
- Physical movement and real-camera model accuracy still require testing on the target chassis and phone.

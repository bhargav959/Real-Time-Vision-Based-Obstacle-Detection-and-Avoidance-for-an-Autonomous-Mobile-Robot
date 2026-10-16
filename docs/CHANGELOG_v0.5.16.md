# v0.5.16 - Turn +20%, straight -20%, and traffic-sign diagnostics

## Command speed profiles
- Base cruise controller remains 40 PWM.
- Forward/reverse translation is scaled to 80% of the base target and capped at PWM 32.
- Steering commands (SL/SR/L/R) are scaled to 120% of the base target, capped at PWM 48.
- Active scan starts at PWM 31 and can rise to PWM 41 when gyro feedback shows the turn is ineffective.
- Firmware enforces the PWM 48 hard cap, straight F/REV cap 32, and slight-turn outer-wheel floor 40 with valid sonar (clamped to 30 on NO_ECHO). The NO_ECHO cap 30 and close-obstacle stops remain in force.
- The motor floor is 25 PWM, so speed scaling cannot go below the uncalibrated floor. Measure the floor under chassis load.

## Traffic-sign checks
- The bundled model's configured label map has 29 classes; `turn_left` is class 22, `turn_right` class 23, `compulsory_left/right` classes 0/1, `left_turn_prohibited` class 7, and `right_turn_prohibited` class 13.
- Added decoder tests with synthetic output tensors to verify all 29 class IDs map to expected labels. This verifies decoder indexing, not learned model accuracy.
- Live telemetry exposes the raw current candidate as `SIGN CANDIDATE`, separately from the confirmed sign. A confirmed sign persists through brief misses but expires after 1.2 s.
- `stop` and `no_entry` remain the only classes that directly influence autonomous behavior. Other classes are informational detections only. Virtual video does not run the traffic-sign detector because the scenes lack traffic-sign ground-truth annotations.

## Validation limits
- Validate model accuracy on held-out real traffic-sign images and the target phone before relying on detections. No ground-truth sign photos were provided for a real model inference test.
- Physical PWM-to-speed/turning response needs chassis calibration; PWM percentages do not guarantee the same percentage change in ground speed.

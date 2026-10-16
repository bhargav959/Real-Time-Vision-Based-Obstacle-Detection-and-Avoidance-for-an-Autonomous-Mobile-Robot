# v0.5.21 — camera-first obstacle stop and side-check profile

- Updated the hardware default for the supplied chassis photo: Arduino UNO + L298N, with no clearly visible HC-SR04. Sonar is now an explicit opt-in firmware setting (`USE_HC_SR04 = true` only after wiring the sensor).
- Live Android snapshots no longer require sonar telemetry; fresh camera/free-space geometry remains mandatory. Side-pivot checks use a conservative camera-estimated clearance threshold.
- Disabled blind BACKOFF by default. If the forward path is blocked and safe side clearance cannot be established from fresh frames, the expected behavior is STOP.
- Preserved the existing project structure, paths, and filenames.

## Hardware safety

Camera-only obstacle avoidance is not a substitute for a physical emergency stop or a range sensor. Test with wheels lifted first, then at very low speed in an open area with a person ready to cut motor power. Camera geometry can fail on transparent, dark, thin, low-profile, or out-of-frame obstacles. If an HC-SR04 is installed and wired to D6/D7, set `USE_HC_SR04 = true` in both firmware sketches and restore the matching Android sonar-required policy before relying on sonar interlocks.

---

# v0.5.21 — bounded blind backoff with verified avoidance

- Re-enabled the short `BACKOFF` command at 25 PWM for a single 250 ms firmware-timed pulse.
- Kept continuous `REV` disabled; the front-facing HC-SR04 cannot confirm rear clearance.
- Removed the post-backoff fast path that could immediately hand control to a normal path candidate.
- After backoff, the robot stops and requires a fresh camera/free-space sample captured after reverse ends before scanning.
- Completed bounded left/right scans before resuming normal navigation.
- Added a final route guard: when an obstacle is detected and the centre corridor is blocked, `FORWARD` is rejected; a feasible signed left/right avoidance command must agree with the selected path direction.
- Added regression tests for bounded backoff default behavior, no early resume while backing off, and rejection of straight motion toward a centre-blocking obstacle.
- Android app version is `0.5.21-bounded-backoff-avoidance` (versionCode 24).

## Limitations

The reverse path remains blind because there is no rear-facing sensor. The app and firmware cannot guarantee physical trajectory without wheel encoders or another suitable odometry source. Test only at low speed with the rear area physically clear; if the post-reverse scan cannot verify a path, the robot remains stopped.

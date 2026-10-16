# Implementation Status — v0.5.16 turn/straight speed profile and sign diagnostics

## Implemented in the current code

- v0.5.16: base cruise ceiling 40 PWM; F/REV commands scaled to 80% and firmware-capped at 32 PWM; normal steering targets scaled to 120% and capped at 48 PWM; slight-turn outer-wheel floor 40 with valid sonar (clamped to 30 on NO_ECHO); moving floor 25; startup kick 30 PWM for 40 ms only below request 28; NO_ECHO cap 30; backoff cap 30. Active scan starts at 31 and may rise by 5 to 41 with valid sonar, or 30 on NO_ECHO. The app and Arduino enforce command-specific limits; calibrate the motor floor under chassis load.

- v0.5.13: independently scores left/right avoidance paths; holds the chosen side across brief perception flicker; releases only after a sustained centre-clear corridor and trajectory recentering; forces SL/SR for even small signed avoidance headings instead of accidentally continuing FORWARD; requests L/R in the ultrasonic stop band only when camera clearance and fresh sonar pass.

- Local obstacle navigation, straight-path preference, and avoidance candidates; lane/line-following is not used.
- 15 cm physical body width and 25.5 cm body length.
- 10 cm nominal side boundary and 20 cm temporary extension beyond it.
- A 2 cm lateral clearance margin on each side in both app planner instances, giving a **19 cm effective minimum gap**. The 10 cm preferred boundary is not added to the gap-fit rule.
- Unknown segmentation pixels remain blocked for collision geometry.
- Candidate path evaluation samples the body footprint over projected depth rows.
- Speed management is based on path curvature, effective gap, obstacle distance and ultrasonic readings; Arduino has a 48 PWM hard cap, while straight F/REV is capped at 32 and normal controller cruise target is 40 before profile scaling.
- Android-to-Arduino serial commands and USB fault handling, with STOP prioritized over movement.
- Live/virtual shared `NavigationController`, `LiveDecisionStabilizer`, safety thresholds and `ActivePathDiscovery` state machine. Leased `SCAN_L`/`SCAN_R` commands are refreshed every control tick and have a 250 ms firmware dead-man timeout; an Arduino turn veto aborts the active scan rather than pretending the robot rotated.
- Virtual differential-drive approximation based on discrete motor commands, PWM floor, low-speed startup kick, slight-turn inner/outer ratio, a 100 ms per-step scan-pivot model with no in-sweep stops, and a front ultrasonic beam model.
- Swept-footprint virtual collision checks and progressive timed-backoff simulation. Backoff distance is predicted from calibrated speed × command PWM × firmware pulse duration, rather than hard-coded to 5 cm.
- Green route overlay only for a feasible/authorized path; scan directions are not presented as verified paths.

## Not equivalent to real hardware yet

- Virtual perception uses perfect synthetic ground-truth masks; live perception relies on a fallible segmentation model and calibrated ground-plane projection.
- Default loaded wheel-speed (18 cm/s at PWM 150) and wheel track (13.5 cm) are **placeholders**, not measured chassis data.
- The actual chassis has no wheel encoders or IMU, so wheel slip, motor mismatch, turning angle, speed and stopping distance cannot be observed directly by the controller.
- The front-facing HC-SR04 cannot verify rear/side clearance; the virtual sonar is geometric and cannot model all acoustic misses.
- A 250 ms, PWM-30 backoff distance is not guaranteed; the lower PWM changes its physical displacement. Measure repeated physical travel and calibrate.

## Required before autonomous physical use

- Verify left/right sign convention with wheels lifted; positive image-ground lateral maps to RIGHT and negative maps to LEFT. Confirm actual `SL/SR` curvature matches the phone overlay, and test close-range pivot refusal at ≤24 cm on the ultrasonic reading.

- Verify motor polarity and `SL/SR` steering with wheels lifted first.
- Calibrate PWM-to-speed under load, wheel track, ultrasonic distances, and phone camera height/tilt/FOV.
- Measure backoff distance and stopping distance at low and maximum allowed PWM.
- Run one-obstacle, narrow-gap, turn and dead-end tests at low speed in a clear, enclosed area with emergency STOP reachable.
- Open in Android Studio, sync dependencies and run the full Gradle build/test suite. The current container cannot run the supplied wrapper because it requires Gradle 9.6+ and the Android SDK is unavailable here.

The software model is more physically grounded by the v0.5.16 discrete-command simulation. The navigation Kotlin sources, speed-profile smoke test, 29-class decoder mapping smoke test, and Arduino C++ syntax check with API stubs passed local checks for v0.5.16. The full Android Gradle build was not available and no physical chassis acceptance test is claimed.


## Traffic-sign detection verification boundary

The decoder map has 29 labels and the bundled `traffic_sign_labels.txt` is in exactly the same order. `turn_left` maps to class 22 and `turn_right` to class 23. New synthetic-output unit tests validate decoding for every class index; they do not measure real-model accuracy. Run the actual TFLite model on labelled held-out traffic-sign images on the target phone to measure detection recall/false positives. Live telemetry reports `SIGN CANDIDATE` (raw highest-scoring detection) separately from `SIGN` (three-frame confirmed); the virtual obstacle-video mode intentionally does not run sign inference. Only `stop` and `no_entry` currently alter navigation behavior.

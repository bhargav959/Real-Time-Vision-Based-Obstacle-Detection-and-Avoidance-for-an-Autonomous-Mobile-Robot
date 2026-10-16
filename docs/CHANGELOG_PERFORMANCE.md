## v0.5.11 — sim-to-real hardening
See docs/SIM_REALITY_CALIBRATION_v0.5.11.md. Fresher sonar (ack-piggybacked `D=`), capture-time data ages,
gyro-verified scan pivots, steering-flicker no longer stops the robot, camera/sonar offsets, shadow-mode logging,
`SimRealism` in the virtual test, headless closed-loop Monte-Carlo test.

# Performance Revision Changelog

## 2026-10-09 — v0.5.8 bounded path-scan backoff

- Added a one-shot `BACKOFF,80` serial action before live/virtual active path scanning.
- Arduino reverse is limited to 250 ms; repeated Android writes do not restart or extend the pulse.
- Added a virtual-world rearward swept-footprint check and a navigation smoke test for BACKOFF-to-left-scan transition.
- Reduced the Android `SpeedController` default PWM cap to 120 to match the firmware's configured hard limit.
- Full Android Gradle build not run: local project requires Gradle 9.6+.

## 2026-10-07 — GC / latency optimization

- Removed per-camera-frame Y-plane `ByteArray` allocation.
- Reused 256x256 and 640x640 letterbox bitmaps, canvases, pixel arrays, and model input float arrays.
- Removed the temporary scaled bitmap from model preprocessing.
- Reused traffic-sign output tensor storage.
- Kept camera backpressure at `KEEP_ONLY_LATEST` and model inference serialized on one executor.
- Updated `compileSdk` from 36 to 37 to satisfy `androidx.core:core-ktx:1.19.0`.
- Preserved the existing safety hierarchy and motor protocol.

## 2026-10-07 (rev 2) — remaining GC sources

Logcat still showed `NativeAlloc concurrent mark compact GC` every ~3-4 s (3-10 MB freed). Those are
background collections (pauses 1-24 ms, not a crash), but they pointed at allocation that was still
happening per frame. Fixed:

- `LineFollower`: the per-pixel helper returned `Int?`, boxing an Integer for every luminance value above
  127 (~240k objects/s at 30 fps). Now returns a primitive (-1 = outside frame); loops are plain `while`
  loops (no progression objects).
- `SemanticMaskDecoder` / `OnnxModelRunner`: output was materialised as nested Java arrays and then boxed
  into a `MutableList<Double>` (65k-200k boxed values per inference). The mask is now decoded straight
  from the output tensor buffer into a reusable `ByteArray` (`runSemantic`). Only the first output is
  requested, the input uses one reusable direct `FloatBuffer`, and the session uses 2-4 intra-op threads
  with full graph optimisation.
- `CameraAnalyzer`: no longer creates a second, rotated full-frame Bitmap per model frame. The raw bitmap
  is passed on with its rotation and the rotation is applied while drawing into the reusable letterbox
  (`ModelInputPreprocessor.Reusable.prepare(bitmap, rotationDegrees)`).
- Free-space mask crop reuses a scratch array.
- `MainActivity`: the 10 Hz control loop only posts to the UI thread when the status text or button state
  actually changed (previously every tick), and CameraX analysis resolution is pinned to 640x480.
- Version bumped to 0.2.1 (versionCode 3). Safety hierarchy and motor protocol unchanged.

## 0.2.2 — traffic-sign input layout
- `TfliteModelRunner` now accepts either [1,640,640,3] (NHWC) or [1,3,640,640] (NCHW) input and exposes `inputChannelFirst`; `MainActivity` builds the input in the matching layout. Fixes "Traffic model input must be [1,640,640,3], got [1, 3, 640, 640]".

## 0.2.3 — decision logging
- New `logging/DecisionLog.kt` + `docs/DECISION_LOG.md`: mode changes, command/reason changes, confirmed signs, model/USB failures logged to logcat (`RobotDecision`) and `logs/decisions.log`.


## 2026-10-08 — obstacle-navigation refinement

- Replaced fixed-column obstacle tests with depth-sampled trajectory footprint checks.
- Unknown semantic regions no longer count as passable gap geometry.
- Updated virtual scenarios to use physical centimetre geometry.
- Added smoother, shared trajectory visualization and matching 65 cm lookahead.


## v0.5.3 — virtual command stability and camera-path alignment

- Added a dedicated virtual navigation controller and disabled concurrent live-loop decision calls during VIDEO TEST.
- Debounced temporary missing geometry separately from immediate collision and critical-fault stops.
- Reduced trajectory-profile peak derivative and increased lookahead to 75 cm for less aggressive avoidance curves.
- Latched one world-space trajectory for the virtual POV overlay, collision guard and simulated pose integration. The view moves along the exact route line; STOP cancels the active path and no route is drawn when no feasible candidate exists.


## 2026-10-10 — v0.5.9 live-control stability and clearance fix

- Added a live-only decision stabilizer: immediate STOPs, two-tick direction confirmation, gradual PWM increase, and immediate speed decrease.
- Reworked the near-sonar behavior so translating commands remain inhibited below 30 cm while a verified pivot may be permitted above 24 cm with camera clearance >=22 cm. Arduino retains a 20 cm pivot hard stop.
- Active path discovery can recover from a near-sonar blocked centre when fresh vision shows the centre is blocked, even if a side path candidate exists. It does not treat NO_ECHO as proof of a safe backoff.
- Fixed path visualization: no feasible candidate no longer produces a misleading green straight line; scan probes are shown as amber dashed cues labelled unverified.
- Added pure-Kotlin smoke checks and unit tests for live stabilization, near pivots, and bounded backoff conditions.
- Full Android Gradle build remains unavailable here because Gradle 9.6+ / Android build environment is not installed. No physical chassis test was performed.


## 2026-10-11 — v0.5.10 simulation/physical parity audit

- Unified live and virtual planner configuration at 19 cm effective passage width (15 cm body + 2 cm per side); removed the old 15 cm comment/README contradiction.
- Replaced ideal curve following in virtual mode with discrete motor command differential-drive/PWM simulation, including Arduino wheel PWM floors, startup kick and slight-turn ratio.
- Added a deterministic virtual HC-SR04-like front beam and enabled the same live safety gate values in the virtual pipeline. Synthetic detection remains more optimistic than the live segmentation model.
- Matched active scanning to dedicated firmware-timed 80 ms PWM-80 `SCAN_L`/`SCAN_R` pulses, plus 200 ms settle/sample intervals. This bounds physical motor-on time despite Android control-loop scheduling jitter.
- Fixed the reverse-distance shortcut: virtual displacement now follows configured speed × PWM × 250 ms. The default 18 cm/s-at-PWM-150 calibration placeholder predicts 2.4 cm at PWM 80 for 250 ms; calibrate this against measured chassis travel.
- Added calibration documentation and tests for gap clearance, motor-command turning, sonar geometry and timed backoff distance.
- Fixed an Android compile issue by importing `LiveDecisionStabilizer` into `MainActivity`, and corrected HUD/header gap labels to consistently report the 19 cm effective requirement.
- Validation boundary: full Gradle/Android build unavailable in this environment; no physical chassis test is claimed.

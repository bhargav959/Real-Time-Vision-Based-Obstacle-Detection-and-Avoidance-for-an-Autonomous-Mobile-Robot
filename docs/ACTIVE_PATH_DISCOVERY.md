# Active Path Discovery — current behavior through v0.5.16

> **Current release note (v0.5.21):** `BACKOFF` is enabled as one blind, firmware-timed pulse capped at 25 PWM / 250 ms; continuous `REV` remains disabled. After the pulse, the robot must stop and wait for a fresh camera/free-space sample captured after reverse ends, scan both sides, and resume only on a verified route. A blocked centre corridor cannot resume with straight `F`. HC-SR04 triggers remain rate-limited to at least 60 ms apart, and opposite steering arcs require a STOP/confirmation tick. Blind reverse still cannot verify rear clearance; only test with the rear area physically clear.

> The historical sections below describe earlier releases. The authoritative current speed values are listed first and in the v0.5.16 appendix at the end.

## Purpose

When an obstacle blocks every verified forward candidate and occupies most of the camera view, the robot can briefly pivot to gather a new view. This is an active-perception behavior, not a blind spin routine.

## State flow

1. `IDLE`: ordinary obstacle-aware path following.
2. `SCAN LEFT`: low-speed pivot is held continuously by same-direction `SCAN_L` keepalives every 100 ms, for up to four increments. Each update rechecks the clearance gates.
3. `SAMPLE LEFT`: the motors stop at the end of the leftward sweep so the camera/free-space pipeline can settle and the controller can re-plan.
4. `SCAN RIGHT`: a bounded rightward sweep is held by `SCAN_R` keepalives for up to eight increments. The change of direction occurs only after the prior sweep has stopped and sampled.
5. `SAMPLE RIGHT`: stop and re-evaluate after the full rightward sweep.
6. A newly feasible path returns control to the normal planner; otherwise the scan ends in `SCAN COMPLETE / NO PATH` and remains stopped.
7. `UNSAFE` / `SAFETY STOP`: motion is stopped immediately if turn clearance or sensor evidence becomes untrustworthy. The firmware also stops if the 250 ms command lease expires.

## Trigger

The normal navigation decision must be STOP, the current free-space snapshot must be valid/fresh, explicit obstacles must be present, the obstacle planner must have no feasible candidate, and the condition must persist for three control frames. Traffic-sign stops, stale frames/models, USB/Arduino readiness problems, and ultrasonic safety stops do not trigger scanning.

## Safety gates

- Live mode requires a fresh camera/free-space estimate, a fresh ultrasonic reading, and camera-estimated nearest-obstacle clearance of at least 22 cm.
- The app inhibits translating movement below 30 cm. A bounded in-place pivot is only allowed at 24 cm or more on the front ultrasonic sensor and at least 22 cm camera-estimated clearance; the Arduino independently rejects pivots below 20 cm. A fresh `NO_ECHO` is not proof of a clear road; the camera must show at least 25 cm pivot clearance and scan PWM remains low.
- If the centre corridor is blocked and the near sonar stop prevents a feasible side route from being driven, one bounded backoff may be requested, then the scan continues only after pivot clearance returns. The rear is not sensed; the operator must verify the rear area before testing.
- Live mode only scans while the app is AUTONOMOUS and the Arduino is connected and ready. Leaving AUTONOMOUS resets the scan controller.
- Same-direction scan commands do not stop the motors between 100 ms refreshes. A full stop is used at sweep boundaries and on every safety/communication fault so the camera can settle before the next sweep.
- Virtual mode uses a swept-circle test with radius `sqrt((19/2)^2 + (25.5/2)^2) + 2 cm` around the vehicle centre, matching the 2 cm per-side gap margin and a separate 2 cm scan buffer. Each virtual scan pulse advances the pose using the differential-drive motor model. Live camera geometry remains approximate.
- After four left pulses and up to eight right pulses, no route means STOP. The right sweep crosses back through the starting heading to inspect both sides. No unbounded rotation is performed.

## Current v0.5.16 parameters (authoritative)

| Parameter | Value |
|---|---:|
| Required blocked frames | 3 |
| Ordinary base cruise ceiling | PWM 40 before command-type scaling |
| Straight F/REV cap | PWM 32 (80% of a 40 PWM base) |
| Normal turn SL/SR/L/R cap | PWM 48 (120% of a 40 PWM base, subject to safety caps and curvature reduction) |
| Slight-turn outer wheel floor | PWM 40 with valid sonar; limited to PWM 30 when NO_ECHO to preserve the safety cap |
| Active scan start / maximum | PWM 31 / 41 with valid sonar; maximum 30 on NO_ECHO |
| Scan step | 100 ms keepalive; firmware lease 250 ms |
| Settle/sample wait | 200 ms after each directional sweep |
| Leftward pulses | Up to 4 |
| Rightward pulses | Up to 8 |
| Camera obstacle clearance gate | 22 cm |
| Live forward/translating stop | 30 cm |
| Live pivot ultrasonic gate | 24 cm |
| Moving PWM floor | 25, uncalibrated |
| Robot dimensions | 15.0 cm wide × 25.5 cm long |

### Historical v0.5.14 baseline

Earlier versions started the scan at PWM 26 and could rise to 34. Those are no longer the current values; v0.5.16 starts at 31 and can rise to 41 with valid sonar, while the NO_ECHO cap remains 30.

These are conservative initial settings. Actual pivot angle depends on motor, battery, chassis, wheel traction, and floor; there are no wheel encoders or IMU in this configuration, so the app re-plans from each camera view instead of assuming an exact angle.

## Virtual scenarios

The 54-second MP4 is a timing clock only. The app renders the POV/world itself. Scenario 8 (segment index 7) is a broad wall intended to trigger the bounded scan; scenario 9 is a dead end that should terminate stopped.

## Validation limitation

The active-scan state machine has JVM unit tests and the virtual world has an explicit pivot-clearance gate. The full Android Gradle build and physical motor timing must still be verified in Android Studio and on the chassis. Do not enable on a real robot until turn direction, PWM, ultrasonic behavior, and camera calibration have been checked in a contained test area.


## v0.5.8 bounded backoff update

After three consecutive blocked-path frames with fresh camera/obstacle geometry, the controller starts a scan pivot only when fresh camera/sonar checks verify pivot clearance. If pivot clearance is not verified, v0.5.21 may use one short blind `BACKOFF` pulse. It then stops, waits for a fresh post-reverse map, scans both sides, and rejects direct-forward resumption while the centre corridor remains blocked. The physical robot has no rear-facing sensor, so the bounded pulse is still blind and must only be tested with the rear area physically clear.

## v0.5.9 live-control stability update

- The physical 10 Hz command loop now uses a separate live stabilizer: STOP remains immediate; a new movement direction needs two consistent ticks; PWM increases gradually and decreases immediately. Active scan pulses are still controlled by the bounded scan state machine.
- A near-sonar stop no longer indiscriminately blocks every possible action. Translating motion is still inhibited below 30 cm; an in-place pivot needs sonar at least 24 cm plus a camera-estimated nearest obstacle of at least 22 cm. Firmware retains its stricter 20 cm pivot cutoff.
- If the centre is visually blocked while the sonar keeps forward movement inhibited, active discovery can request one bounded backoff even when the planner has identified a side candidate, then waits for turn clearance.
- The overlay no longer draws a green straight line when no feasible path exists. During active scan it draws an amber dashed direction cue, explicitly labelled as not yet verified.
- These changes cannot make physical turning match the simulation exactly: the chassis remains open-loop without wheel encoders/IMU, and PWM thresholds, camera geometry, and motor direction still need calibration.


## v0.5.10 simulation/physical parity audit

- Virtual and live planner instances share the same effective 19 cm gap policy.
- Virtual mode shares the live command stabilizer and scan state machine, uses 100 ms `SCAN_L`/`SCAN_R` steps starting at PWM 26 (matching the app control tick), models no motor stop within a sweep, and simulates the front ultrasonic beam against the known obstacle world.
- Virtual motion follows discrete motor outputs using the PWM floor, startup kick and 45% inner-wheel command used by the Arduino; it does not snap to the planner's curve. The default speed and 13.5 cm wheel-track values are unmeasured placeholders.
- A simulated backoff advances by the predicted distance for the same PWM-80 / 250 ms pulse. Under the default speed placeholder this is 2.4 cm, not 5 cm.


## Historical v0.5.15 extra-low-speed crawl override

These values are retained as release history; v0.5.16 supersedes them. App and Arduino share a 40 PWM maximum, 25 PWM moving floor, 33 PWM slight-turn outer-wheel minimum, 30 PWM/40 ms startup kick only below request 28, 30 PWM NO_ECHO cap, and 30 PWM backoff cap. Active scan begins at 26 PWM and can rise by 5 only to 34 with valid sonar or 30 on NO_ECHO. The live decision stabilizer limits speed increases to 5 PWM per 100 ms tick. The app sends same-direction `SCAN_L`/`SCAN_R` keepalives every 100 ms; Arduino checks clearance and stops if the 250 ms lease expires. Sweep-end stops and all safety stops remain. The actual minimum reliable loaded PWM must be measured; lower PWM may stall rather than move smoothly.


## v0.5.16 turn-speed update (current)

Active scan begins at PWM 31 (20% above v0.5.15's 26), may rise to PWM 41 (20% above 34) when turn feedback is insufficient, and remains capped at PWM 30 for NO_ECHO. The base controller's ordinary steering speeds are scaled up by 20% after geometry/clearance reductions, but stopped/no-path/close-obstacle gates are unchanged. Straight F/REV motion is separately capped at PWM 32. On NO_ECHO, the Arduino clamps the slight-turn outer wheel to 30 PWM instead of applying its normal 40 PWM outer-wheel floor, keeping the stated safety speed cap consistent. The actual rotation angle remains hardware-dependent; verify pivot direction and clearance before live autonomous use.

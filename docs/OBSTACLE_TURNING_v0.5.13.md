# Robust left/right obstacle avoidance — v0.5.13

## What changed

- The local planner evaluates LEFT and RIGHT corridor candidates independently rather than choosing from one combined search. It favours a meaningfully wider verified corridor, then a lower-cost trajectory, with deterministic tie-breaking.
- Once the robot commits to a feasible avoidance side, that side is retained through brief mask changes. A single clear-centre frame does not release the lock. The centre must be clear for at least four frames and the planned trajectory must have returned near centre. If the selected side becomes infeasible, the planner may select the other verified side; if neither side is feasible, it stops.
- A blocked centre path with a small heading angle still emits `SL` or `SR`; it is no longer rounded to `F` by the generic five-degree dead band.
- If fresh front sonar enters the app's 30 cm stop zone, the controller requests an in-place `L`/`R` pivot toward the selected safe path. The app only allows it when sonar is at least 24 cm, the nearest camera-estimated obstacle is at least 22 cm away, and the path's measured free gap is at least 3 cm wider than the normal configured fit width. Otherwise it sends STOP.
- `STOP`/`NO_ENTRY` controls and stale perception/sonar stops retain priority. When no safe path is verified, the robot does not guess a turn; the existing bounded scan flow remains subject to fresh evidence and clearance checks.

## Verification

The pure Kotlin navigation source compiles. Twenty-five targeted JVM test methods for `NavigationController` and `ObstaclePlanner` pass in local checks, including small-angle left/right commands, valid close-range pivot commands, refusal of too-close/narrow-gap pivots, corridor feasibility, and avoidance-side stability. The full Android Gradle build was not run here because the project wrapper requires an installed Gradle 9.6+ and Android SDK; open the project in Android Studio and sync/build it there.

## Physical test checklist

1. Lift the drive wheels and verify that `SL` curves left, `SR` curves right, `L` pivots left, and `R` pivots right. Reverse logical motor polarity in the Arduino sketch if needed.
2. Keep a hardware emergency stop within reach. Test in an enclosed area at low speed with soft obstacles; do not test near people, stairs, traffic, or breakable objects.
3. Test centre blockage with open space on each side and confirm the selected direction is stable across camera flicker.
4. Test an obstacle closer than 24 cm on the ultrasonic sensor and verify the robot remains stopped rather than pivoting.
5. The HC-SR04 faces forward and cannot establish rear clearance. The existing timed backoff may move backwards only under its software gates, so keep the rear area physically clear; do not treat it as a production safety system.
6. Calibrate camera height/tilt/FOV, ultrasonic offset, motor dead band, turn direction, and stopping distance on the actual chassis. Simulation and source tests do not substitute for a physical safety validation.

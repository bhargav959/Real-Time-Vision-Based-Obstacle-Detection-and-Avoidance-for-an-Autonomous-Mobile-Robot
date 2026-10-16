# v0.5.3 — Virtual command stability, lower-curvature paths, and POV alignment

## Stop / movement flicker

- Virtual TEST has its own `NavigationController` instance. The live scheduled control loop returns before invoking its controller while the virtual test is active. This prevents concurrent calls from mutating the same `lastMotion`, STOP latch, or NO_ENTRY state.
- `obstacle geometry unavailable` is now treated as a geometry-only transient in the virtual stabilizer, not an immediate hard-fault string match. Geometry-only STOP must persist for four frames. Initial motion requires three agreeing frames and direction changes require confirmation.
- Hard safety conditions still stop immediately; a predicted swept-footprint intersection forces STOP and cancels the current trajectory.

## Less aggressive avoidance curve

- The shared trajectory profile uses a symmetric power-easing curve (exponent 1.25) with a peak normalized lateral derivative of 1.25, versus 1.5 for the previous cubic smoothstep. The selected route continues to be checked against the complete robot footprint; a smoother candidate is not accepted if it cannot clear the obstacle.
- Planning lookahead is now 75 cm, giving the planner more distance for the lateral displacement and reducing the required endpoint bearing for equivalent offsets. The physical camera model still requires calibration.

## POV follows the displayed route

- A feasible planner path is latched in world coordinates as an `ActiveTrajectory`. The generated POV draws the remaining path using that trajectory, the collision guard samples the same trajectory, and pose integration advances along the same profile.
- During STOP, the active trajectory is cancelled. When there is no feasible path, the POV does not draw a made-up straight/curved path; it displays the STOP state instead.
- The HUD includes command, PWM, path angle, elapsed virtual time, frame, and pose. The MP4 remains a timing clock; visible obstacles and road markings are generated from the evolving virtual camera pose.

## Watchdog preservation

The v0.5.2 watchdog-only correction is retained unchanged: telemetry requests do not refresh the movement-command watchdog. No other Arduino behavior is intentionally changed in this refinement.

## Validation

- Core navigation Kotlin compile: passed.
- Full 8-scenario virtual simulation: 0 predicted collisions; command changes are stable, with turn-in/counter-steer/straight phases during curves.
- Geometry/manual harness: clear straight, 12 cm gap boundary crossing, exact 15 cm fit, blocked dead end, hard-stop priority and command hysteresis pass.
- Virtual provider Kotlin compile against Android graphics stubs: passed; unused API warnings were removed/handled where practical.
- Full Android Gradle build and device-level rendering/inference were not run here. Open the project in Android Studio, run the unit tests and APK build, then inspect the generated POV on-device before real operation.

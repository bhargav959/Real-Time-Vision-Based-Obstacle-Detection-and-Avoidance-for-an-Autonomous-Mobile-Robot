# Obstacle-navigation audit — v0.5.20

## Changes made

- Removed the three-ping burst from each Arduino motion gate. The old 10 ms spacing was shorter than the HC-SR04 quiet interval and could make cross-talk look like clearance. Each command gate now uses one fresh ping; the in-motion guard continues to sample at its configured interval.
- Disabled `REV` and `BACKOFF` by default in firmware because the installed sensor faces forward and cannot certify the rearward path. Android path discovery now scans in place only when fresh front/pivot clearance is verified; otherwise it remains stopped. A clearly named bench-test override exists for supervised tests only and must not be used as a substitute for a rear sensor.
- Added a stop/confirmation tick before changing directly from a left arc to a right arc (or vice versa), reducing steering flips caused by noisy frame-to-frame replanning.
- Bumped Android app version to `0.5.20-obstacle-path-safety` (versionCode 23).

## Planning versus physical motion

`ObstaclePlanner` evaluates a local ground-projected swept footprint and the controller re-plans repeatedly from new camera frames. The virtual harness additionally checks simulated swept-footprint collisions. The actual robot still has no wheel encoders, and the phone gyro is currently used to validate active scan pivots, not to reconstruct the chassis's full XY trajectory. Therefore this code can confirm that commands follow the selected path direction and can stop on known unsafe conditions, but it cannot prove that the chassis traced the exact planned curve on the floor. Wheel encoders or an external pose/odometry source are required for that guarantee.

## Hardware validation required

1. Raise drive wheels and verify `F`, `SL`, `SR`, `L`, `R` direction signs before any floor test.
2. Confirm the app reports a signed path angle and sends the matching left/right command.
3. Place obstacles in the center, left, and right corridors; verify it chooses only a feasible side or stops. Verify pivot stops below the ultrasonic threshold and when camera gap is insufficient.
4. Confirm `BACKOFF` and `REV` return the disabled error in the default firmware. Do not enable the bench-test override on the floor.
5. Test with low battery and uneven traction; a feasible image-space path is not a guarantee of chassis clearance.

## Validation status

- 75 core navigation/protocol unit tests passed in a local Kotlin runner, including the new steering reversal and default reverse-denial cases.
- Arduino sketch passed a C++ syntax check against a minimal mock Arduino header.
- An Android Gradle test/build could not be run in this environment because the archive does not include the official Gradle wrapper JAR and Gradle 9.6+ is not installed.
- Physical motion cannot be certified from uploaded source alone. Wheel directions, clearances, camera geometry, and motor response still require the hardware checklist above.

# Obstacle-navigation audit — v0.5.21

## Requested behavior

Keep the blind reverse recovery pulse, then ensure the robot follows an avoidance route instead of advancing directly toward an obstacle.

## Changes

- Android `ActivePathDiscovery` defaults to a single bounded backoff pulse only if a persistent obstacle blocks the centre path and the in-place pivot clearance is not already verified.
- Backoff remains 25 PWM for 250 ms, enforced by firmware. Repeated app writes do not restart or lengthen the pulse.
- After the pulse, the app always enters STOP / WAIT_CLEARANCE; a feasible candidate from the pre-reverse view cannot cancel the reverse-and-scan recovery.
- A camera/free-space sample must be captured after the reverse ends and include a stop/settle guard interval before any scan pivot can begin.
- The controller completes a bounded left/right scan, samples a fresh view, and resumes only if the currently selected route is feasible.
- When an obstacle is still present and the centre corridor is blocked, the final guard refuses `FORWARD`, requires a slight/hard left or right command, and checks its side matches the signed path direction.
- Arduino `BACKOFF` is enabled while continuous `REV` stays disabled.
- HC-SR04 pings remain rate-limited to at least 60 ms apart.

## Verification

- Regression tests added for default bounded backoff, no direct resumption immediately after reverse, and rejection of `FORWARD` when an obstacle still blocks the centre corridor.
- **77/77 Kotlin/JVM navigation and serial-protocol tests passed**, including the new regression cases.
- **Arduino C++ syntax check passed** against a mock Arduino header. This is not an upload or hardware test.
- Full Android Gradle build and physical chassis trajectory cannot be certified in this container; wheel slip, camera calibration, motor polarity and actual reverse displacement require a physical test.

## Hardware caution

The reverse command is intentionally blind. The timed pulse limits duration, not risk. Test only when the rear area is physically clear and supervise the robot. If fresh post-reverse geometry does not identify a safe avoidance path, the expected behavior is STOP, not a guess.

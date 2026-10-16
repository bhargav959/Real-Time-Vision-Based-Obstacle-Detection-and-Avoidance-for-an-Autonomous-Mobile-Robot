# Refinement Notes — v0.5.4 Active Path Discovery

## Changes

- Added `ActivePathDiscovery`, a bounded stop/scan/sample/replan state machine for explicit obstacles that block every verified candidate.
- The state machine requires three consecutive blocked observations, emits one low-speed left probe, then at most five rightward pulses with a stop and sampling interval after each pulse.
- Active scanning is denied or aborted if the pivot sweep cannot be verified, camera/free-space data are stale, ultrasonic safety is triggered, a traffic sign imposes a stop, USB/Arduino readiness is lost, or a simulated collision is predicted. A failed scan ends in STOP; the robot does not spin indefinitely or reverse blindly.
- Added a full-view-wall virtual scenario. Its semantic world and display use the same world coordinates, and the POV rotates in place during scan pulses. Physical motor output remains disabled in VIDEO TEST.
- Extended the clock video to 54 seconds at 10 FPS to cover nine six-second scenarios.
- Kept the Arduino sketch unchanged from v0.5.3, including the watchdog-only fix.

## Validation performed

- Compiled the pure Kotlin navigation/controller/active-scan classes with the local Kotlin compiler.
- Ran a bounded-scan harness: it ended in `NO_ROUTE`/STOP when no route appeared and refused to pivot when clearance was not verified.
- Ran the virtual wall geometry with the real planner and the virtual pose provider: the forward view began blocked and the planner found a route after the simulated scan rotated the view.
- Confirmed the active-scan pivot gate checks the swept vehicle footprint before each virtual rotation.
- Compared the Arduino firmware SHA-256 with the v0.5.3 baseline; it is unchanged.
- Checked the bundled clock asset is 54 seconds and checked the source ZIP's archive integrity before repackaging.

## Build limitation

The full Android Gradle test/build could not run in this environment because the project wrapper requires Gradle 9.6+ and reports that it is not installed/available. Open the ZIP in Android Studio, sync, and run unit tests before physical use. The pure Kotlin and virtual geometry checks are not a substitute for an Android build or a guarded real-robot test.

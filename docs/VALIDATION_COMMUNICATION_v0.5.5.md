# Communication validation — v0.5.5

## Checks performed in this environment
- Protocol smoke test passed: all seven Android movement wires match the defined set; PWM is clamped; STOP encodes to `S,0`; unsupported/injected command strings and unsupported telemetry requests are rejected.
- Kotlin protocol classes compiled with `kotlinc`; source parser check reported no Kotlin syntax/parser errors in the changed Android files. Android types were not available to the standalone compiler.
- Arduino sketch passed a C++ syntax check using a stubbed Arduino API. This verifies C++ syntax only, not board-specific compilation or electrical behavior.
- ZIP integrity check passed.
- Static source review confirmed `sendRaw` is synchronized, fatal serial read errors set `connected=false`, old-port telemetry is discarded, and no outbound USB command remains queued on the general I/O executor.

## Not performed
The complete Android Gradle task could not run because the project wrapper requires Gradle 9.6+, which is not installed/available in this build environment. No actual phone, USB-OTG cable, Arduino UNO, HC-SR04, or motor hardware test was performed.

## Physical verification checklist
1. With wheels lifted, connect USB and wait for `ROBOT_READY`/`STATUS=OK`.
2. Confirm the Android UI remains SAFE until ARM.
3. Request `DISTANCE` and verify `DISTANCE=<number>` or `DISTANCE=NO_ECHO`.
4. Trigger/observe `RESULT=STOP_OBSTACLE`, `SAFETY_STOP=WATCHDOG`, or `ERROR=...`; verify Android switches to SAFE and disables ARM until a fresh numeric reading is at least 30 cm.
5. Verify the fresh reading clears the interlock but does not automatically restart motion. Press ARM explicitly.
6. Disconnect USB during motion; verify the UI detects disconnection, Arduino watchdog stops outputs, and reconnect requires fresh readiness.
7. At low PWM, verify the 90 PWM / 60 ms startup boost and adjust only after measuring real chassis response.

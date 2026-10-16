# Communication reliability update — v0.5.5

This revision retains the v0.5.4 navigation and active path discovery behavior while improving Android/Arduino command scheduling and fault recovery.

## Changes
- Serialized all USB serial writes under one synchronization lock.
- Rejected empty or multiline outbound command strings.
- Used the existing connection executor for USB permission completion rather than leaking a new executor per permission result.
- A fatal reader exception now closes only the affected port, sets `connected=false`, and forces the next control tick out of autonomous mode. Telemetry callback exceptions no longer kill the reader thread.
- Removed periodic queuing of STOP commands from the 10 Hz control loop. A STOP is sent on a mode transition and on explicit STOP button presses.
- STATUS and DISTANCE requests are sent synchronously through the serialized serial writer rather than through a competing executor queue. Distance polling is reduced to 400 ms (the freshness limit remains 800 ms).
- Arduino safety rejections (`RESULT=STOP...`), watchdog reports, and protocol errors latch Android in SAFE immediately. ARM stays disabled until a fresh numeric ultrasonic reply is at least 30 cm; that only clears the interlock and the user must still press ARM.
- Arduino DISTANCE telemetry uses one ping to reduce command-parser blocking. Movement commands continue to run the filtered ultrasonic safety check. The host polls at 400 ms, below the existing 800 ms sensor freshness limit.
- Reduced the low-PWM startup kick from 150/120 ms to 90/60 ms to reduce active-scan overshoot. Tune this on the real chassis.
- Added `SerialProtocol` as the Android source of truth for supported motion commands and telemetry requests, plus unit tests for command encoding/rejection.
- Prevented stale-reader telemetry from an old USB port from marking a newly connected Arduino ready.

## Protocol remains unchanged
- 115200 baud, 8-N-1, ASCII, newline-terminated.
- Movement: `F,150`, `SL,90`, `SR,90`, `L,80`, `R,80`, `REV,80`, `S,0`.
- Telemetry requests: `DISTANCE`, `STATUS`.
- Telemetry: `ROBOT_READY`, `STATUS=OK`, `DISTANCE=<cm|NO_ECHO>`, `RESULT=...`, `SAFETY_STOP=WATCHDOG`, `ERROR=...`.

## Safety note
This is source-level hardening, not hardware validation. Confirm USB permission/connect/reconnect, motor stop response, startup PWM, watchdog behavior, and app/Arduino command framing on the actual phone and UNO with wheels lifted before floor tests.

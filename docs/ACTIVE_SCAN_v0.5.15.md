# v0.5.15 — Extra-Low-Speed Crawl and Active Scan

## Active scan behavior

- Android and Arduino share a **40 PWM hard cap**. Normal live motion ramps upward by at most 5 PWM every 100 ms control tick; requested speed reductions and STOP remain immediate.
- The moving PWM floor is 25. The slight-turn outer wheel minimum is 33 so there remains some differential across the 25 PWM inner-wheel floor. These are starting values and must be calibrated under the actual chassis load.
- The normal startup kick is 30 PWM for 40 ms only when the requested wheel PWM is below 28. Active scan pivots bypass this kick.
- Active scan starts at **26 PWM**. After ineffective yaw feedback, or after two open-loop ticks when no usable gyro feedback is available, the scan may increase by 5 to a maximum of **34 PWM with a valid sonar echo** or **30 PWM on NO_ECHO**. If the robot still does not rotate, the bounded scan aborts rather than increasing beyond the cap.
- Normal motion with a NO_ECHO ultrasonic reading is capped at 30 PWM. The one-shot BACKOFF is capped at 30 PWM for 250 ms. Because PWM was reduced, its retreat distance will differ from earlier versions and must be measured.
- Left and right scan pivots use same-direction keepalives every 100 ms. Arduino checks turn clearance on refresh and uses a 250 ms dead-man lease. Motors remain powered between same-direction scan updates, then stop after each full sweep so the camera can obtain a stable frame. User STOP, stale evidence, close clearance, sign veto, USB failure and watchdog still stop the robot immediately.

## Values that must match

| Parameter | Android | Arduino |
|---|---:|---:|
| Global hard PWM cap | 40 | 40 |
| Minimum moving PWM floor | 25 | 25 |
| Slight-turn outer wheel minimum | 33 | 33 |
| Startup kick | 30 for 40 ms only for requests below 28 | same |
| NO_ECHO cap | 30 | 30 |
| Backoff cap | 30 | 30 |
| Active scan starting PWM | 26 | receives command |
| Scan maximum with valid sonar | 34 | global cap remains 40 |
| Scan maximum on NO_ECHO | 30 | no-echo clamp 30 |
| Scan refresh interval | 100 ms | 250 ms dead-man lease |
| Live PWM ramp-up | +5 per 100 ms tick | n/a |

## Calibration and safe testing

PWM is not a speed unit. A PWM of 26 may not rotate your physical chassis reliably. A low limit can produce stalling, asymmetric turns or no motion rather than a smooth slower movement. With wheels lifted, verify direction first; then test under load in a clear, enclosed area with STOP accessible. If PWM 26 cannot turn the chassis, measure the lowest reliable pivot PWM and adjust the app and firmware values together. Do not raise the speed cap blindly.

The HC-SR04 faces forward only. Clear the area behind the robot before testing BACKOFF. No 250 ms reverse duration can guarantee 5 cm without calibration (and wheel encoders would be needed for direct displacement feedback).

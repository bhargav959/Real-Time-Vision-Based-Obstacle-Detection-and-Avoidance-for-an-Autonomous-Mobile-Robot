# v0.5.20 — obstacle path safety review

- Rate-limit HC-SR04 triggers to >=60 ms globally; replace the 3-ping/10 ms burst in motion gates with a single ping. `DISTANCE=TOO_SOON` is ignored as telemetry, and command acknowledgements continue carrying the authorizing reading.
- Disable app-driven blind reverse by default; use a verified scan pivot if safe, otherwise stay stopped. Arduino rejects `REV` and `BACKOFF` unless the explicit supervised bench-test flag is enabled. A rear-facing sensor is needed before automatic reverse can be safely enabled.
- Require a STOP/confirmation tick before opposite arc steering, including a brief intervening forward command, to reduce left/right oscillation.
- Add tests for default blind-reverse refusal and steering reversal stabilization.
- Add an audit report describing code-verifiable findings and the remaining lack of full chassis odometry.

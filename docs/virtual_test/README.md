# Virtual Obstacle Navigation Test

The virtual harness provides nine deterministic obstacle scenarios: open straight travel, a central obstacle, a 12 cm gap, a 15 cm raw geometric-fit gap that must be rejected by the app's 19 cm effective requirement, left/right avoidance, multiple obstacles, a full-view wall that triggers active path discovery, and a dead end.

The bundled MP4 is a 54-second clock source sampled at 10 virtual frames per second. The app renders a first-person POV from the known obstacle world. The vehicle pose advances from discrete motor commands and an approximate differential-drive/PWM model; it no longer assumes that the robot follows a planner-selected ideal curve.

## Stability behavior (v0.5.3)

- Only the virtual frame processor owns a dedicated virtual navigation controller. The live control loop skips decision-making during VIDEO TEST, preventing concurrent updates from resetting hysteresis.
- A geometry-only STOP must persist for four frames; initial motion needs three agreeing frames; command direction changes need confirmation. True hazards and predicted swept-footprint collisions remain immediate stops. STOP at the beginning of each six-second scenario is deliberate while the new synthetic world is initialized; within a scenario, brief geometry dropouts are filtered.
- The virtual trajectory profile has a lower peak lateral derivative, and planning lookahead is 75 cm, reducing peak curvature compared with v0.5.2.
- The current v0.5.10 model sends the selected discrete motor command into the wheel/PWM kinematic model. It does not counter-steer along a latched ideal trajectory. The green overlay is the current candidate path, not a guarantee that the simulated body will track it perfectly.
- Physical robot output remains disabled throughout virtual testing.

Use RESET, then STEP through the initial frames before PLAY. Watch `COMMAND`, `SPEED`, `PATH ANGLE`, and `POSE` along with the green selected path. At a dead end, the planner should settle on STOP; it must not draw a green route through unknown or blocked space. Live and virtual use the same 19 cm effective gap rule, but virtual segmentation is synthetic ground truth and is therefore still more optimistic than the real AI model.


## Active path discovery (v0.5.4)

- If an explicit obstacle leaves no verified path, the controller must see that condition for at least three frames before it scans.
- The scan consists of one low-speed left pivot, a stop/settle/sample interval, then up to five low-speed right pivot pulses with stop/settle/sample intervals.
- The normal planner continues to evaluate each newly captured view. If a feasible path appears, scanning ends and normal navigation resumes from the robot's current heading.
- Scanning is refused/aborted when the pivot sweep cannot be verified as clear, camera/free-space data are stale, ultrasonic data are stale/too close, or a traffic-sign/hard safety stop is active. The virtual provider performs an explicit half-diagonal swept-circle check before in-place rotation.
- If the bounded sweep finds no path, the robot remains stopped. It does not spin indefinitely or reverse blindly.
- Scenario 8 presents a broad near-field wall whose side route becomes visible after the simulated POV turns; scenario 9 remains a no-route dead-end test.
- Virtual scan turns modify only the simulated heading. Physical robot output is disabled in VIDEO TEST mode.

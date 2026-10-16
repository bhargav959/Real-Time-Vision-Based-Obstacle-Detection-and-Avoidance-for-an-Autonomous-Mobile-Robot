> Historical note: the ideal-path-following and virtual-only stabilizer described below are pre-v0.5.10 behavior. Version 0.5.10 replaces them with a discrete motor-command model and shared live/virtual stabilization. See `SIM_REALITY_CALIBRATION_v0.5.10.md` for the current behavior, including firmware-timed scan pivots.

# Historical audit — Physical vs Virtual Navigation in v0.5.9

## Why physical behavior differs

The virtual test has synthetic obstacle masks and pose updates, checks a swept vehicle footprint before simulated movement, and uses a virtual-only decision stabilizer. Live mode instead receives predictions from the segmentation model, estimates centimetres from the mounted phone camera, reads one forward-facing HC-SR04, and sends open-loop L298N motor commands without wheel encoders or an IMU. The same path plan therefore does not imply the same physical pose or turning angle.

## v0.5.9 software corrections

1. **Live command stability.** A live STOP is immediate. New movement directions require two consistent 10 Hz control ticks; uncertain direction changes issue STOP rather than continuing a previously chosen direction. PWM rises gradually and decreases immediately.
2. **Near-obstacle gating.** Forward/translating commands remain inhibited below 30 cm. A live in-place pivot is permitted in the 24–30 cm band only if the camera estimate places the nearest detected obstacle at least 22 cm away. Arduino firmware independently rejects a pivot below 20 cm. If the centre is visually blocked and near sonar prevents a feasible side path from being driven, the bounded one-shot backoff can be used before rescanning.
3. **Truthful overlay.** A green line is shown only for a feasible and currently authorized path candidate. During active left/right scans, an amber dashed direction cue is labelled “PATH NOT VERIFIED”. If the path is blocked or a safety gate prevents movement, the overlay no longer implies that a straight route is safe.
4. **Telemetry clarity.** The live HUD displays the 19 cm minimum gap requirement when the additional 2 cm per-side live clearance margin is active, plus the relevant sonar/pivot gates.

## Physical acceptance test

Perform these in order in a clear, bounded area. Keep the emergency STOP reachable.

1. **Polarity with wheels lifted:** issue STOP, then brief F, REV, L and R commands at low PWM. Confirm the wheels' directions match the labels before setting the chassis down. If directions are wrong, correct wiring or the `INVERT_LEFT_MOTOR` / `INVERT_RIGHT_MOTOR` constants and retest.
2. **Ultrasonic sanity check:** hold a flat object at measured distances (approximately 20, 25, 30 and 40 cm). Compare the HUD reading with a ruler. Check for `NO_ECHO`, stale values or large jumps.
3. **Reverse calibration:** only with the area behind the robot known clear, run one `BACKOFF,80` pulse on the floor and measure the actual distance. Adjust `BACKOFF_DURATION_MS` only after repeated tests; 250 ms is an estimate, not a measured 5 cm guarantee.
4. **Camera geometry:** verify the phone is mounted at the configured 10 cm height and approximately 7.5° downward tilt. Recalibrate height, tilt, and FOV if measured obstacle distances disagree with the ruler.
5. **Low-speed obstacle test:** begin at PWM 60–72, use one broad obstacle, and verify the sequence is stop/backoff as justified, wait for clearance, scan, then move only after a feasible path is recognized. Do not test near people, pets, stairs, traffic, or fragile objects.
6. **Read the reason field:** use `PATH DISCOVERY`, `OBSTACLES`, `NEAREST`, `ULTRASONIC`, `COMMAND`, and `REASON` to separate a sensor disagreement from a blocked planner route or a motor-direction issue.

## Remaining limitations

- The rear is not sensed. The software cannot guarantee a reverse path is clear; the operator must verify it.
- The HC-SR04 is forward-facing and cannot establish side clearance during rotation. The camera clearance estimate is approximate and depends on calibration and semantic segmentation quality.
- A scan pivot changes the camera heading without wheel-angle feedback. Its physical angle varies with battery, wheel traction, floor, gearboxes and PWM startup behavior.
- A detected feasible trajectory is a geometric target, not a closed-loop path-tracking guarantee. Accurate physical tracking would benefit from wheel encoders and/or an IMU, plus motor calibration.
- The Android Gradle build was not run in this environment because the supplied launcher requires Gradle 9.6+, which is unavailable here. Pure Kotlin navigation/stability tests and protocol smoke checks passed; no physical hardware test was performed.

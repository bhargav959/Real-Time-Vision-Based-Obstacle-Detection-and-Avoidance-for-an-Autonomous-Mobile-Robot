# Virtual Obstacle-Navigation Test Harness

The virtual test validates the new obstacle-navigation controller rather than lane following.

## Safety

Virtual mode never sends a movement command to the physical robot. It forces `S,0` when entering the test and displays `ROBOT OUTPUT: DISABLED`.

## Scenes

The bundled `robot_virtual_test.mp4` contains:

- open straight path;
- center obstacle with side avoidance;
- two obstacles with a narrow center gap that forces temporary boundary crossing;
- two obstacles with a passable center gap;
- offset obstacle requiring a curved avoidance path;
- multi-obstacle exit.

A deterministic `VirtualScenarioProvider` supplies matching obstacle masks to the same `ObstaclePlanner` used by the live camera path. This is intentional: a simple synthetic MP4 cannot provide reliable semantic segmentation ground truth by itself.

## What to watch

The telemetry panel shows:

- selected command;
- PWM speed;
- path type (STRAIGHT / AVOIDANCE_CURVE / STOP);
- path angle;
- boundary state (NOMINAL / CROSSING / BLOCKED);
- estimated obstacle count and nearest distance;
- center gap and required 19 cm effective gap in the app configuration;
- selected gap;
- ultrasonic value when connected.

The white overlay indicates the nominal physical boundary. The amber overlay indicates the temporary expanded boundary. The green line indicates the selected path.

## Animated POV behavior

The prerecorded MP4 is used as a 54-second clock only. The app generates the visible virtual POV at runtime from the known obstacle map and current vehicle pose. Pose advances from discrete `F/SL/SR/L/R/REV/BACKOFF/S` motor commands using differential-drive kinematics, PWM floor, startup-kick behavior, wheel ratio and swept-footprint collision checks. The overlay displays a planner candidate; it does not force the body to follow that curve.

The nine scenarios run for six seconds each (60 frames at 10 virtual FPS). At scenario boundaries the pose, virtual motor/kick state, backoff progress and temporal obstacle state reset. Live and virtual modes share the same navigation controller, command stabilizer, gap margin, and active-scan state machine. Virtual mode also simulates a forward ultrasonic beam, but uses perfect synthetic obstacle ground truth rather than neural model predictions. No physical motor command is sent in VIDEO TEST mode.

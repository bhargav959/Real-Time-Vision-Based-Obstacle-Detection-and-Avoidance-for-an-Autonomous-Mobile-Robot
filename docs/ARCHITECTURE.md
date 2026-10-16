# System Architecture — Obstacle Navigation v0.5.10

This describes the current v0.5.10 implementation. Values that depend on wheel speed, wheel track, floor friction, camera pose or motor mismatch remain calibration parameters; no physical acceptance test has yet been claimed. See `SIM_REALITY_CALIBRATION_v0.5.10.md`.

```text
                         PHONE CAMERA
                              |
                    latest CameraX frame
                              |
              +---------------+---------------+
              |                               |
              v                               v
       FREE-SPACE AI                    TRAFFIC-SIGN AI
       YOLO26n-sem / ONNX              Ultralytics YOLO / TFLite (best.pt checkpoint)
          256x256                         640x640
              |                               |
              v                               v
       semantic mask                    sign filter / NMS
              |                               |
              v                               |
       letterbox crop                        |
              |                               |
              +---------------+---------------+
                              |
                              v
                    CAMERA-GROUND PROJECTION
                              |
                              v
                    OBSTACLE / GAP GEOMETRY
                              |
                +-------------+-------------+
                |                           |
                v                           v
          CENTER PATH                 GAP / FOOTPRINT
          explicit FREE               >= 19 cm effective required
                |                           |
                +-------------+-------------+
                              |
                              v
                    BOUNDARY POLICY
                  nominal ±10 cm center
                  expanded ±30 cm center
                              |
                    if nominal blocked
                              |
                              v
                       PATH PLANNER
                    STRAIGHT / CURVE / STOP
                              |
                              v
                       SPEED CONTROL
                              |
                              v
                         USB SERIAL
                    COMMAND,SPEED\n
                              |
                              v
                           ARDUINO
                  ultrasonic + watchdog safety
                              |
                              v
                           L298N
                              |
                            MOTORS
```

## Responsibilities

### Free-space model
Produces semantic classes:

- `0 = free`
- `1 = obstacle`
- `2 = unknown`
- `255 = ignore`

The obstacle planner uses **only class 0 as drivable geometry**. Unknown does not count toward a usable gap.

### Ground projection
Maps semantic image rows and columns into approximate ground-plane centimetres using the configured camera height, tilt, and FOV.

Current configuration:

- camera height = 10 cm;
- downward tilt = 7.5°;
- horizontal FOV = 62°;
- vertical FOV = 48°;
- planning lookahead = 75 cm (v0.5.3 tuning).

### Obstacle planner
The planner evaluates candidate local trajectories over the forward planning band. App live and virtual planner instances use a 2 cm per-side clearance margin, so the 15 cm chassis width is expanded to a **19 cm effective gap requirement**. Unknown pixels are not drivable.

Path selection order:

```text
center FREE path
      ↓
nominal ±10 cm
      ↓ if blocked
expanded ±30 cm
      ↓ if blocked
STOP
```

The robot's 25.5 cm length is handled through the depth sampling; the route is rejected if the footprint becomes non-free at any sampled depth.

### Gap rule
A passage is accepted by the application only when its estimated usable FREE width is at least 19 cm under the current clearance configuration. The 10 cm nominal side-boundary margin is a different policy and is not added to this gap width.

The 0.5 cm rasterization tolerance handles mask quantization and is not a substitute for calibrated physical clearance.

### Path planner
Two navigation modes are exposed:

- `STRAIGHT`: clear center route is preferred.
- `AVOIDANCE_CURVE`: a smooth lateral trajectory is generated toward a safe offset.

A target offset beyond ±10 cm is marked `CROSSING` because it leaves the nominal corridor. This is permitted only when the nominal corridor contains no feasible route.

### Speed controller
PWM is reduced as:

- path angle increases;
- gap becomes narrower;
- obstacle distance decreases;
- temporary boundary crossing occurs;
- ultrasonic distance decreases.

Normal travel is capped at 40 PWM, matching the Arduino hard cap. Active scanning starts at 26 PWM and may rise by 5 to 34 PWM with valid sonar, or 30 PWM when the sensor reports NO_ECHO. The ordinary moving floor is 25 PWM, the slight-turn outer-wheel minimum is 33, and the startup kick is 30 PWM for 40 ms only for requested values below 28. Active scan bypasses the startup kick. Live command increases are limited to 5 PWM per 100 ms tick; STOP and speed reductions remain immediate. These values are placeholders pending loaded-chassis calibration.

### Traffic signs
The current package uses a custom 7-class Ultralytics traffic-sign detector exported from best.pt trained on the merged Road Sign detection and Indian Traffic Sign Recognition datasets. Labels and IDs are listed in `app/src/main/assets/models/traffic_sign_labels.txt` and must match `TrafficSignDecoder.labels` exactly.

The model's packaged runtime contract is float32 NCHW `[1,3,640,640]` RGB/255 input and `[1,33,8400]` detection output. The decoder performs class-aware NMS and temporal filtering.

`stop` (class 5) produces a STOP latch and `no_entry` (class not present in checkpoint) stops/vetoes a route toward the sign. The other 27 classes are currently informational; they do not issue direct motor commands. Confirmed safety-critical signs retain priority over local navigation.

### Arduino
Arduino is the final physical safety authority. It:

- receives `COMMAND,SPEED`;
- drives L298N ENA=D9 / ENB=D10 PWM;
- polls HC-SR04;
- vetoes dangerously close forward motion;
- executes a bounded 250 ms BACKOFF request only when issued by the Android path-scan controller;
- stops on communication timeout;
- reports telemetry back over USB.

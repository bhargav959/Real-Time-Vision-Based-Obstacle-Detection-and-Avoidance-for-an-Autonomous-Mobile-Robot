# Obstacle Navigation — v0.5.1 refined design

## Core behavior

The robot does **not** follow a painted lane. The phone plans locally from explicit free-space and obstacle geometry.

Normal operation:

1. Prefer a straight path through explicit FREE space.
2. If the straight route is blocked, search for a feasible path inside the nominal corridor.
3. Only when no nominal path can fit the 15 cm robot does the planner search the expanded corridor.
4. Crossing the nominal boundary is therefore a deliberate obstacle-avoidance maneuver, not normal wandering.
5. If no explicit FREE 15 cm path exists in the expanded corridor, STOP.

Unknown pixels are treated as **not drivable** for geometry. The free-space score may use them as uncertainty, but the collision planner does not count them as a usable gap.

## Vehicle geometry

- Robot width: 15.0 cm
- Robot length: 25.5 cm
- Nominal boundary: 10 cm outside each robot side
- Nominal center-offset limit: ±10 cm
- Temporary expanded center-offset limit: ±30 cm
- Temporary expanded boundary: 20 cm beyond the nominal boundary
- Gap-fit requirement: exactly 15.0 cm usable FREE width
- Local lookahead: 65 cm
- No 10 cm side margin is added to a passage gap
- Camera height: 10 cm
- Initial camera downward tilt: 7.5°

The nominal boundary is a **preferred operating corridor**. It is not a collision boundary and it can be crossed when necessary.

## Path geometry

A selected avoidance path is modeled as a smooth lateral trajectory:

- near the vehicle: lateral offset ≈ 0 cm;
- at lookahead: lateral target = selected offset;
- intermediate points follow a smoothstep profile.

The planner samples the actual trajectory through multiple ground-depth rows and checks the complete 15 cm vehicle footprint at every sampled row. This is safer than checking one fixed image column.

Two path modes are exposed:

- `STRAIGHT`
- `AVOIDANCE_CURVE`

The Android command layer converts the selected path angle into `F`, `SL`, `SR`, `L`, or `R`.

## Gap logic

For each candidate trajectory the planner measures the contiguous **explicit FREE** region around the trajectory center.

```text
usable gap >= 15 cm  -> physically fits
usable gap < 15 cm   -> reject
UNKNOWN              -> reject for geometry
```

The robot's 25.5 cm length is handled by evaluating several ground-depth rows across the forward planning band; the minimum usable gap/footprint result must remain feasible along the route.

## Boundary crossing

The planner tries, in order:

```text
center path
    ↓
nominal ±10 cm
    ↓ if no nominal path fits
expanded ±30 cm
    ↓ if no expanded path fits
STOP
```

This prevents unnecessary boundary crossing while still allowing obstacle avoidance when the nominal corridor is blocked.

## Speed policy

Speed is not a fixed direction constant. The phone calculates PWM from:

- path curvature / heading angle;
- boundary crossing;
- usable gap width;
- obstacle proximity;
- ultrasonic proximity.

Typical command examples:

```text
F,150
SL,105
SR,105
L,80
R,80
S,0
```

The Android target is capped at 150 PWM. The Arduino remains the final safety layer and applies its own ultrasonic veto and communication watchdog.

## USB protocol

Android → Arduino:

```text
COMMAND,SPEED\n
```

Examples:

```text
F,150
SR,105
SL,105
R,80
L,80
S,0
```

Arduino → Android may return:

```text
DISTANCE=42.5
RESULT=SR,105
RESULT=STOP_OBSTACLE:22.0
STATUS=OK
```

Serial: 115200 baud, 8-N-1.

## Camera calibration

The initial metric projection assumes:

- height = 10 cm;
- downward tilt = 7.5°;
- horizontal FOV = 62°;
- vertical FOV = 48°.

These FOV values are initial engineering values, not a guarantee of exact centimetre measurement for every phone. Before physical autonomous testing, calibrate at least two known distances on the actual floor/camera mount and adjust the centralized configuration if needed.

## Virtual test

The virtual video validates obstacle-navigation geometry rather than lane following. The scenario provider supplies deterministic masks paired with the video so geometry behavior can be tested independently of synthetic-video model generalization.

Expected cases:

- open road → straight;
- center obstacle → side avoidance;
- narrow center gap → reject gap / search side;
- passable ≥15 cm gap → permit passage;
- nominal blockage → temporary boundary crossing;
- no feasible expanded route → STOP.

The bundled virtual video is H.264/AVC, 1280x720, 30 FPS, 24 seconds, YUV420p.

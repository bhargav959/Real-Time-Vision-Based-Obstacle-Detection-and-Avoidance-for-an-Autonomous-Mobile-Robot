# v0.5.1 Refinement Notes

This package is a refinement of the first obstacle-navigation release.

## Navigation geometry fixes

1. Candidate paths now follow a smooth lateral trajectory through the forward depth band instead of testing a fixed image column.
2. The complete 15 cm vehicle footprint is checked at every sampled depth row.
3. The planner requires explicit `FREE` pixels for geometric clearance. `UNKNOWN` is not considered drivable.
4. Even when no obstacle is detected, the planner cannot bypass the geometry checks and blindly drive through an unknown region.
5. A small 0.5 cm rasterization tolerance exists only to prevent a mathematically exact 15 cm gap from being rejected because the 256x256 semantic grid is discrete. It is not added physical clearance.
6. The default local lookahead is 65 cm to provide more time for an avoidance curve with the 10 cm camera mounting height.

## Path policy

```text
CENTER FREE
   ↓
STRAIGHT

CENTER BLOCKED
   ↓
NOMINAL ±10 cm search

NOMINAL BLOCKED
   ↓
EXPANDED ±30 cm search

EXPANDED BLOCKED
   ↓
STOP
```

Boundary crossing is therefore deliberate and obstacle-driven.

## Virtual test refinement

The scenario generator now defines obstacles in centimetres and projects them using the same camera-ground model as the live planner. This prevents synthetic pixel geometry from accidentally describing an impossible physical gap.

The bundled test contains eight scenes:

1. Open straight
2. Center obstacle
3. 12 cm narrow gap
4. Exact 15 cm passable gap
5. Left obstacle / right curve
6. Right obstacle / left curve
7. Multiple obstacles
8. Dead end

The video is 1280x720, 30 FPS, 24 seconds total (3 seconds per scenario).

## Firmware refinement

The matching Arduino firmware:

- uses ENA=D9 and ENB=D10 PWM with both jumpers removed;
- accepts `COMMAND,SPEED`;
- uses a 400 ms communication watchdog;
- uses separate front-motion and pivot safety distances;
- keeps an explicit motor-polarity inversion switch for physical wiring differences;
- uses a proportional 45% inner-wheel speed for `SL`/`SR` instead of a fixed PWM subtraction.

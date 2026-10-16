# Decision Log Policy

The navigation controller records command/reason changes instead of logging every 10 Hz control tick.

Examples:

    CMD: SLIGHT_RIGHT,95 (obstacle blocks nominal corridor; temporary boundary crossing)
    CMD: FORWARD,150 (clear path; straight preferred)
    SAFETY: STOP,0 (ultrasonic < 30 cm)

The important geometric fields are:

- free left/center/right scores;
- obstacle and unknown center scores;
- selected gap width;
- nearest obstacle distance;
- boundary state;
- path type and angle;
- PWM speed.

- Virtual test path/POV refinement: align scenario ground-truth to the actual 10 FPS clock (60 frames per scene), render first-person scene from the same world pose as the semantic mask, integrate pose from selected path angle/PWM, and disable sign inference on the old text-labelled clock video. Apply command/STOP hysteresis only to virtual simulation output; physical safety decisions remain immediate.

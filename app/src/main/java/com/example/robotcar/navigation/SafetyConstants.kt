package com.example.robotcar.navigation

/**
 * Single source of truth for thresholds that must agree between the Android app and the Arduino
 * firmware (docs/arduino/RobotCarObstacleNavigation.ino repeats these numbers; keep them in sync).
 *
 * Ordering rule: the app acts first (smooth, planned stop); the Arduino is the backstop and
 * therefore triggers at a slightly SHORTER distance.
 */
object SafetyConstants {
    /** App inhibits forward/translating motion below this sonar distance; verified pivots may pass above APP_TURN_MIN_DISTANCE_CM. */
    const val ULTRASONIC_STOP_CM = 30f

    /** Firmware thresholds are intentionally slightly below app thresholds. */
    const val ARDUINO_FORWARD_STOP_CM = 28f
    const val ARDUINO_TURN_STOP_CM = 20f

    /** Camera-estimated nearest-obstacle clearance required before an in-place pivot. */
    const val APP_PIVOT_CLEARANCE_CM = 22f

    /** App pivot gate: firmware turn cutoff plus a 4 cm sensor/latency margin. */
    const val APP_TURN_MIN_DISTANCE_CM = ARDUINO_TURN_STOP_CM + 4f

    /** In-place pivot needs extra measured lateral clearance beyond the straight-through fit width. */
    const val APP_PIVOT_EXTRA_GAP_CM = 3f

    /**
     * An ultrasonic reading older than this is not trusted while a robot is attached. v0.5.11: the
     * firmware piggybacks the filtered distance on every motion ack (10 Hz) and the idle poll runs
     * at ~5 Hz, so 400 ms is achievable. The old 800 ms allowed ~11 cm of blind travel at the previous higher PWM limit.
     */
    const val ULTRASONIC_MAX_AGE_MS = 400L

    /**
     * Max age of the camera-derived free-space map (measured from FRAME CAPTURE, not from the end
     * of inference) for authorising a scan sample/pulse. Includes inference latency.
     */
    const val SCAN_EVIDENCE_MAX_AGE_MS = 500L

    /** With NO_ECHO the sensor gives no close-range assurance, so speed is capped. */
    const val NO_ECHO_MAX_PWM = 30

    /**
     * Lowest PWM that reliably moves the chassis. PLACEHOLDER - calibrate on the real robot
     * (lift wheels, raise PWM until both wheels turn on the floor under load). Mirrored by
     * MIN_MOTOR_PWM in the Arduino sketch.
     */
    const val MIN_MOTOR_PWM = MotorCalibration.MIN_MOTOR_PWM
}

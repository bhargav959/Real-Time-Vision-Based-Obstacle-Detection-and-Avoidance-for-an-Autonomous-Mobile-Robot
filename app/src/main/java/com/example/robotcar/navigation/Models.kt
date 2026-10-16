package com.example.robotcar.navigation

/** High-level motion command understood by the Arduino firmware. */
enum class MotorCommand(val wire: String) {
    FORWARD("F"),
    LEFT("L"),
    RIGHT("R"),
    SLIGHT_LEFT("SL"),
    SLIGHT_RIGHT("SR"),
    REVERSE("REV"),
    BACKOFF("BACKOFF"), // one-shot, firmware-timed reverse pulse for obstacle clearance
    PIVOT_LEFT("SCAN_L"), // low-speed scan pivot, held only while keepalive commands arrive
    PIVOT_RIGHT("SCAN_R"), // low-speed scan pivot, held only while keepalive commands arrive
    STOP("S")
}

/**
 * Motion values mirrored by the Arduino sketch and the virtual differential-drive model.
 * The wheel-speed and track-width figures are calibration placeholders, not measured facts.
 * Update them after timing a known distance and measuring the chassis axle track.
 */
object MotorCalibration {
    /** Base cruise controller limit before the requested straight/turn scaling is applied. */
    const val NORMAL_CRUISE_MAX_PWM = 40
    /** Firmware hard cap leaves room for the +20% turning adjustment: round(40 * 1.45) = 58. */
    const val ARDUINO_MAX_PWM = 58
    /** Straight F/REV commands are capped at 80% of the previous 40 PWM cruise ceiling. */
    const val STRAIGHT_MAX_PWM = 32
    const val STRAIGHT_SPEED_FACTOR = 0.80f
    const val TURN_SPEED_FACTOR = 1.45f
    const val MIN_MOTOR_PWM = 25
    const val SLIGHT_TURN_OUTER_MIN_PWM = 48
    const val STARTUP_KICK_PWM = 30
    const val STARTUP_KICK_MS = 40L
    const val STARTUP_KICK_THRESHOLD_PWM = 28
    const val BACKOFF_PWM = 25
    const val BACKOFF_DURATION_MS = 250L
    const val CONTROL_TICK_MS = 100L
    // One scan step equals one control tick; firmware holds the low-speed pivot while commands refresh.
    const val SCAN_PULSE_DURATION_MS = CONTROL_TICK_MS
    const val NOMINAL_SPEED_CM_S_AT_PWM_150 = 18f // Placeholder; calibrate on the loaded chassis
    const val WHEEL_TRACK_WIDTH_CM = 13.5f

    /** Scan limits are +20% over the v0.5.15 crawl values (26 -> 31, 34 -> 41). */
    const val SCAN_MIN_EFFECTIVE_YAW_DEG = 0.5f
    const val SCAN_START_PWM = 37
    const val SCAN_MAX_PWM = 49
    const val SCAN_SETTLED_YAW_RATE_DEG_S = 4f
    const val SCAN_FRAME_AFTER_STOP_GUARD_MS = 100L
    const val SCAN_MAX_SETTLE_MS = 900L
}

/** Applies consistent command-type speed scaling after obstacle/geometry speed reduction. */
object CommandSpeedProfile {
    fun adjust(command: MotorCommand, basePwm: Int, noEcho: Boolean = false): Int {
        if (command == MotorCommand.STOP || basePwm <= 0) return 0
        val scaled = when (command) {
            MotorCommand.FORWARD, MotorCommand.REVERSE ->
                (basePwm * MotorCalibration.STRAIGHT_SPEED_FACTOR).toInt()
                    .coerceIn(MotorCalibration.MIN_MOTOR_PWM, MotorCalibration.STRAIGHT_MAX_PWM)
            MotorCommand.SLIGHT_LEFT, MotorCommand.SLIGHT_RIGHT, MotorCommand.LEFT, MotorCommand.RIGHT ->
                kotlin.math.round(basePwm * MotorCalibration.TURN_SPEED_FACTOR).toInt()
                    .coerceIn(MotorCalibration.MIN_MOTOR_PWM, MotorCalibration.ARDUINO_MAX_PWM)
            // Active scan and timed backoff already have their own explicit, separately capped PWM.
            MotorCommand.PIVOT_LEFT, MotorCommand.PIVOT_RIGHT ->
                basePwm.coerceIn(MotorCalibration.MIN_MOTOR_PWM, MotorCalibration.SCAN_MAX_PWM)
            MotorCommand.BACKOFF -> basePwm.coerceIn(MotorCalibration.MIN_MOTOR_PWM, MotorCalibration.BACKOFF_PWM)
            MotorCommand.STOP -> 0
        }
        return if (noEcho) minOf(scaled, com.example.robotcar.navigation.SafetyConstants.NO_ECHO_MAX_PWM) else scaled
    }
}

enum class RobotMode { SAFE, AUTONOMOUS, STOPPED }

enum class PathType { STRAIGHT, AVOIDANCE_CURVE, SCANNING, STOP }

enum class BoundaryState { NOMINAL, CROSSING, BLOCKED }

data class TrafficSign(
    val label: String,
    val confidence: Float,
    val centerX: Float = 0.5f
)

data class NavigationGeometryConfig(
    val robotWidthCm: Float = 15f,
    val robotLengthCm: Float = 25.5f,
    val nominalBoundaryMarginCm: Float = 10f,
    val temporaryBoundaryExtensionCm: Float = 20f,
    val cameraHeightCm: Float = 10f,
    val cameraTiltDeg: Float = 7.5f,
    val horizontalFovDeg: Float = 62f,
    val verticalFovDeg: Float = 48f,
    val gapFitWidthCm: Float = 15f,
    val targetLookaheadCm: Float = 75f,
    val nearPlanningDistanceCm: Float = 20f,
    val maxPlanningDistanceCm: Float = 90f,
    val planningStepCm: Float = 4f,
    /** Small rasterization tolerance; this is not physical clearance and is only for pixel quantization. */
    val geometryRasterToleranceCm: Float = 0.5f,
    /**
     * Extra lateral clearance required on EACH side of the robot (perception error, body
     * protrusion, motion error). Standalone planner defaults to 0, but the app configures 2 cm
     * in both live and virtual modes, making the effective passage gap 19 cm for a 15 cm body.
     */
    val gapClearanceMarginCm: Float = 0f,
    /** Obstacle pixels needed on a row before it counts as an obstacle for nearest-distance. */
    val minObstaclePixelsPerRow: Int = 2,
    /**
     * How far the phone camera sits AHEAD of the robot's pivot centre (cm; negative = behind it).
     * Planner distances are measured from the pivot centre, so a camera mounted at the nose would
     * otherwise make every obstacle look [cameraForwardOffsetCm] closer/farther than it is.
     * MEASURE THIS on the chassis; the default 0 assumes the camera is over the centre.
     */
    val cameraForwardOffsetCm: Float = 0f,
    /** How far the HC-SR04 face sits ahead of the pivot centre (cm). Default = front edge of a 25.5 cm body. */
    val sonarForwardOffsetCm: Float = 12.75f
) {
    /** Robot width plus clearance on both sides; the width every footprint/gap check uses. */
    val effectiveRobotWidthCm: Float
        get() = robotWidthCm + 2f * gapClearanceMarginCm

    /** Minimum usable free gap: configured fit width plus clearance on both sides. */
    val requiredGapWidthCm: Float
        get() = gapFitWidthCm + 2f * gapClearanceMarginCm

    /** Outer physical boundary lines are robot half-width + the configured side margin. */
    val nominalBoundaryHalfWidthCm: Float
        get() = robotWidthCm * 0.5f + nominalBoundaryMarginCm

    val expandedBoundaryHalfWidthCm: Float
        get() = nominalBoundaryHalfWidthCm + temporaryBoundaryExtensionCm

    /** Allowed centerline displacement before the robot crosses the nominal boundary. */
    val nominalCenterOffsetCm: Float
        get() = nominalBoundaryMarginCm

    /** Allowed centerline displacement in the temporary avoidance region. */
    val expandedCenterOffsetCm: Float
        get() = nominalBoundaryMarginCm + temporaryBoundaryExtensionCm
}

data class PathCandidate(
    val lateralOffsetCm: Float,
    val targetDistanceCm: Float,
    val minimumGapWidthCm: Float,
    val headingAngleDeg: Float,
    val crossesBoundary: Boolean,
    val feasible: Boolean
)

data class ObstaclePlan(
    val obstacleDetected: Boolean,
    val obstacleCountEstimate: Int,
    val nearestObstacleCm: Float?,
    val centerPathFeasible: Boolean,
    val nominalPathFeasible: Boolean,
    val expandedPathFeasible: Boolean,
    val bestPath: PathCandidate?,
    val centerGapWidthCm: Float,
    val requiredGapWidthCm: Float,
    val nominalBoundaryHalfWidthCm: Float,
    val expandedBoundaryHalfWidthCm: Float,
    val nominalCenterOffsetCm: Float,
    val expandedCenterOffsetCm: Float,
    val boundaryState: BoundaryState,
    val pathType: PathType
)

data class PerceptionSnapshot(
    val freeLeft: Float = 0f,
    val freeCenter: Float = 0f,
    val freeRight: Float = 0f,
    val obstacleCenter: Float = 0f,
    val unknownCenter: Float = 0f,
    val freeSpaceValid: Boolean = false,
    val obstaclePlan: ObstaclePlan? = null,
    /** Most recent raw model candidate for diagnostics; never used to authorize motion. */
    val trafficSignCandidate: TrafficSign? = null,
    val trafficSignCandidateAgeMs: Long = Long.MAX_VALUE,
    /** Temporally confirmed sign; only this field may influence NavigationController. */
    val trafficSign: TrafficSign? = null,
    val ultrasonicCm: Float? = null,
    /** HC-SR04 reported NO_ECHO: nothing in range, OR a surface that does not reflect. Not "clear". */
    val ultrasonicNoEcho: Boolean = false,
    val ultrasonicAgeMs: Long = Long.MAX_VALUE,
    /** True when a physical robot is attached (live mode); false in the virtual harness. */
    val ultrasonicRequired: Boolean = false,
    val cameraAgeMs: Long = 0L,
    val freeSpaceAgeMs: Long = Long.MAX_VALUE,
    val trafficSignAgeMs: Long = Long.MAX_VALUE
)

data class NavigationDecision(
    val command: MotorCommand,
    val speedPwm: Int,
    val reason: String,
    val pathType: PathType,
    val boundaryState: BoundaryState,
    val pathAngleDeg: Float = 0f,
    val gapWidthCm: Float? = null,
    val nearestObstacleCm: Float? = null
)

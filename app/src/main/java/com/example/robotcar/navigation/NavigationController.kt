package com.example.robotcar.navigation

import kotlin.math.abs
import kotlin.math.min

/**
 * Local obstacle-navigation controller. There is deliberately no lane-following dependency here.
 * The robot prefers a straight path, but will generate an avoidance curve when an obstacle blocks
 * the nominal corridor. A temporary expanded corridor is permitted only when the nominal region
 * contains no feasible path.
 */
class NavigationController(
    private val maxCameraAgeMs: Long = 350L,
    private val maxFreeSpaceAgeMs: Long = 600L,
    private val maxTrafficSignAgeMs: Long = 1_200L,
    private val stopHoldMs: Long = 2_000L,
    /** After a STOP-sign hold, the same sign is ignored for this long so the robot can leave it. */
    private val stopCooldownMs: Long = 8_000L,
    /** A NO_ENTRY sighting stays in force this long after the sign was last seen. */
    private val noEntryMemoryMs: Long = 1_500L,
    private val speedController: SpeedController = SpeedController()
) {
    private enum class Side { LEFT, CENTER, RIGHT }

    private var stopLatchUntilMs = 0L
    private var stopIgnoreUntilMs = 0L
    private var noEntrySeenAtMs = Long.MIN_VALUE
    private var noEntrySide = Side.CENTER
    private var lastMotion = MotorCommand.STOP

    fun reset() {
        stopLatchUntilMs = 0L
        stopIgnoreUntilMs = 0L
        noEntrySeenAtMs = Long.MIN_VALUE
        lastMotion = MotorCommand.STOP
    }

    fun decide(snapshot: PerceptionSnapshot, nowMs: Long): NavigationDecision {
        if (snapshot.cameraAgeMs > maxCameraAgeMs) return stop("stale camera frame")

        // Ultrasonic: a missing/stale reading is "unknown", never "clear", when a robot is attached.
        if (snapshot.ultrasonicRequired && snapshot.ultrasonicAgeMs > SafetyConstants.ULTRASONIC_MAX_AGE_MS) {
            return stop("ultrasonic stale/unavailable")
        }
        val ultrasonic = snapshot.ultrasonicCm

        if (nowMs < stopLatchUntilMs) return stop("stop sign latch")

        val sign = if (snapshot.trafficSignAgeMs <= maxTrafficSignAgeMs) snapshot.trafficSign else null
        if (sign != null && sign.confidence >= 0.75f) {
            when (sign.label.lowercase()) {
                "stop" -> if (nowMs >= stopIgnoreUntilMs) {
                    stopLatchUntilMs = nowMs + stopHoldMs
                    stopIgnoreUntilMs = stopLatchUntilMs + stopCooldownMs
                    return stop("traffic sign: STOP")
                }
                "no_entry" -> {
                    noEntrySeenAtMs = nowMs
                    noEntrySide = when {
                        sign.centerX < 0.4f -> Side.LEFT
                        sign.centerX > 0.6f -> Side.RIGHT
                        else -> Side.CENTER
                    }
                }
            }
        }
        val noEntryActive = noEntrySeenAtMs != Long.MIN_VALUE && nowMs - noEntrySeenAtMs <= noEntryMemoryMs
        if (noEntryActive && noEntrySide == Side.CENTER) return stop("traffic sign: NO_ENTRY ahead")

        if (!snapshot.freeSpaceValid || snapshot.freeSpaceAgeMs > maxFreeSpaceAgeMs) {
            return stop("free-space model unavailable/stale")
        }

        val plan = snapshot.obstaclePlan ?: return stop("obstacle geometry unavailable")
        val candidate = plan.bestPath ?: return stop("no feasible path within expanded boundary")
        val angle = candidate.headingAngleDeg

        // A NO_ENTRY on one side vetoes steering toward that side.
        if (noEntryActive) {
            val plannedLateral = if (abs(angle) >= 0.25f) angle else candidate.lateralOffsetCm
            if (noEntrySide == Side.LEFT && plannedLateral < -0.5f) return stop("NO_ENTRY on left; path leads left")
            if (noEntrySide == Side.RIGHT && plannedLateral > 0.5f) return stop("NO_ENTRY on right; path leads right")
        }

        // When the straight corridor is blocked, following a mathematically feasible curve is
        // mandatory. The generic heading classifier previously returned FORWARD for angles below
        // 5°, which could ignore a small but necessary avoidance offset. Keep the sign of the
        // planner's path and command an actual left/right arc. At the close-range stop boundary,
        // request an in-place LEFT/RIGHT pivot; the independent sonar + camera gate below decides
        // whether that pivot is safe, otherwise the robot remains stopped.
        val avoidanceRequired = !plan.centerPathFeasible || plan.pathType == PathType.AVOIDANCE_CURVE
        val command = if (avoidanceRequired) {
            classifyAvoidance(angle, candidate.lateralOffsetCm, snapshot)
        } else {
            classify(angle)
        }
        if (command == MotorCommand.STOP) {
            return stop("avoidance direction is ambiguous; refusing to guess a turn")
        }

        // Below the normal forward-stop distance, a translating command is not allowed. A
        // bounded in-place pivot may be allowed only with both a conservative sonar margin and
        // a camera-estimated clearance. This avoids the old deadlock where the global 30 cm stop
        // prevented every avoidance turn even though the firmware permits pivots above 20 cm.
        if (snapshot.ultrasonicRequired && ultrasonic != null &&
            ultrasonic < SafetyConstants.ULTRASONIC_STOP_CM) {
            val cameraPivotClear = plan.nearestObstacleCm?.let {
                it >= SafetyConstants.APP_PIVOT_CLEARANCE_CM
            } == true && candidate.minimumGapWidthCm >=
                plan.requiredGapWidthCm + SafetyConstants.APP_PIVOT_EXTRA_GAP_CM
            val verifiedPivot = command in setOf(MotorCommand.LEFT, MotorCommand.RIGHT) &&
                ultrasonic >= SafetyConstants.APP_TURN_MIN_DISTANCE_CM && cameraPivotClear
            if (!verifiedPivot) {
                return stop(
                    "ultrasonic < ${SafetyConstants.ULTRASONIC_STOP_CM.toInt()} cm: " +
                        "forward motion inhibited; verified pivot/backoff required"
                )
            }
        }

        // Apply the requested speed profile to the command actually sent to the chassis:
        // straight translation is 20% below the old 40 PWM cruise ceiling; steering commands
        // are 20% above the base controller target. The Arduino repeats the hard caps as a backstop.
        val baseSpeed = speedController.speedFor(plan, angle, ultrasonic)
        val noEcho = snapshot.ultrasonicRequired && snapshot.ultrasonicNoEcho
        var speed = CommandSpeedProfile.adjust(command, baseSpeed, noEcho)
        var noEchoNote = ""
        if (noEcho) noEchoNote = " [ultrasonic NO_ECHO: speed capped]"
        lastMotion = command

        val reason = when {
            !plan.obstacleDetected && command == MotorCommand.FORWARD -> "clear explicit free path; straight preferred"
            plan.boundaryState == BoundaryState.CROSSING -> "nominal corridor blocked; temporary boundary crossing permitted"
            plan.centerPathFeasible && command == MotorCommand.FORWARD -> "center passage is ${plan.centerGapWidthCm.formatCm()} cm; ${plan.requiredGapWidthCm.formatCm()} cm required"
            plan.nominalPathFeasible && command != MotorCommand.FORWARD -> "nominal corridor avoidance path"
            plan.expandedPathFeasible && command != MotorCommand.FORWARD -> "expanded corridor avoidance path"
            command == MotorCommand.FORWARD -> "explicit free center path feasible"
            else -> "avoid obstacle via ${angle.formatAngle()}° path; usable gap ${candidate.minimumGapWidthCm.formatCm()} cm"
        } + noEchoNote

        return NavigationDecision(
            command = command,
            speedPwm = speed,
            reason = reason,
            pathType = plan.pathType,
            boundaryState = plan.boundaryState,
            pathAngleDeg = angle,
            gapWidthCm = candidate.minimumGapWidthCm,
            nearestObstacleCm = plan.nearestObstacleCm
        )
    }

    /**
     * Selects the actual avoidance direction from the signed local path. Negative lateral/heading
     * is LEFT, positive is RIGHT. Small avoidance curves still issue SL/SR rather than FORWARD.
     * Within the ultrasonic forward-stop zone, request a true L/R pivot; the caller must pass the
     * separate verified-pivot gate before this command can reach the motors.
     */
    private fun classifyAvoidance(
        angle: Float,
        lateralOffsetCm: Float,
        snapshot: PerceptionSnapshot
    ): MotorCommand {
        val steeringAngle = when {
            abs(angle) >= 0.25f -> angle
            abs(lateralOffsetCm) >= 0.25f -> if (lateralOffsetCm < 0f) -4f else 4f
            else -> return MotorCommand.STOP // ambiguous path direction: never guess
        }
        val directionIsLeft = steeringAngle < 0f
        val sonarInsideForwardStop = snapshot.ultrasonicRequired &&
            snapshot.ultrasonicCm != null &&
            snapshot.ultrasonicCm < SafetyConstants.ULTRASONIC_STOP_CM

        if (sonarInsideForwardStop) {
            return if (directionIsLeft) MotorCommand.LEFT else MotorCommand.RIGHT
        }

        val magnitude = abs(steeringAngle)
        val pivotEnter = 22f
        val pivotHold = 18f
        val wasSamePivot = if (directionIsLeft) lastMotion == MotorCommand.LEFT else lastMotion == MotorCommand.RIGHT
        if (magnitude > pivotEnter || (wasSamePivot && magnitude > pivotHold)) {
            return if (directionIsLeft) MotorCommand.LEFT else MotorCommand.RIGHT
        }
        // There is no dead band while the centre corridor is blocked: even 1–4 degrees must
        // produce a differential-drive command in the selected direction.
        return if (directionIsLeft) MotorCommand.SLIGHT_LEFT else MotorCommand.SLIGHT_RIGHT
    }

    /**
     * Heading -> command with hysteresis: the threshold to LEAVE the current command is a few
     * degrees looser than the threshold to ENTER it, so a path angle hovering on a boundary does
     * not make the robot flip between commands every frame.
     */
    private fun classify(angle: Float): MotorCommand {
        val a = abs(angle)
        val pivotEnter = 22f
        val pivotHold = 18f
        val slightEnter = 5f
        val slightHold = 3f

        val wasLeftPivot = lastMotion == MotorCommand.LEFT
        val wasRightPivot = lastMotion == MotorCommand.RIGHT
        val wasLeftSlight = lastMotion == MotorCommand.SLIGHT_LEFT
        val wasRightSlight = lastMotion == MotorCommand.SLIGHT_RIGHT

        if (angle < 0f) {
            if (a > pivotEnter || (wasLeftPivot && a > pivotHold)) return MotorCommand.LEFT
            if (a > slightEnter || ((wasLeftSlight || wasLeftPivot) && a > slightHold)) return MotorCommand.SLIGHT_LEFT
        } else {
            if (a > pivotEnter || (wasRightPivot && a > pivotHold)) return MotorCommand.RIGHT
            if (a > slightEnter || ((wasRightSlight || wasRightPivot) && a > slightHold)) return MotorCommand.SLIGHT_RIGHT
        }
        return MotorCommand.FORWARD
    }

    private fun stop(reason: String): NavigationDecision {
        lastMotion = MotorCommand.STOP
        return NavigationDecision(
            command = MotorCommand.STOP,
            speedPwm = 0,
            reason = reason,
            pathType = PathType.STOP,
            boundaryState = BoundaryState.BLOCKED,
            pathAngleDeg = 0f
        )
    }
}

private fun Float.formatCm(): String = String.format(java.util.Locale.US, "%.1f", this)
private fun Float.formatAngle(): String = String.format(java.util.Locale.US, "%+.1f", this)

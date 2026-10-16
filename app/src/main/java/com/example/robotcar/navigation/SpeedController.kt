package com.example.robotcar.navigation

import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

/** Converts path geometry and obstacle clearance into a conservative PWM target. */
class SpeedController(
    private val maxPwm: Int = MotorCalibration.NORMAL_CRUISE_MAX_PWM,
    private val minMovingPwm: Int = SafetyConstants.MIN_MOTOR_PWM,
    private val slowestPwm: Int = SafetyConstants.MIN_MOTOR_PWM,
    private val obstacleSlowDistanceCm: Float = 80f,
    private val stopDistanceCm: Float = SafetyConstants.ULTRASONIC_STOP_CM
) {
    fun speedFor(
        plan: ObstaclePlan?,
        pathAngleDeg: Float,
        ultrasonicCm: Float?
    ): Int {
        if (plan?.pathType == PathType.STOP) return 0

        val curveFactor = (abs(pathAngleDeg) / 35f).coerceIn(0f, 1f)
        val boundaryReduction = if (plan?.boundaryState == BoundaryState.CROSSING) 0.42f else 0f

        val narrowReduction = plan?.bestPath?.let {
            // The base chassis width is 15 cm, but plan.requiredGapWidthCm already includes the
            // configured per-side safety margin. Use that effective requirement rather than a
            // hard-coded raw-width assumption so virtual and live planners remain consistent.
            val normalized = ((it.minimumGapWidthCm - plan.requiredGapWidthCm) / 30f).coerceIn(0f, 1f)
            0.40f * (1f - normalized)
        } ?: 0f

        val cameraObstacleReduction = plan?.nearestObstacleCm?.let { distance ->
            if (distance <= stopDistanceCm) 1f
            else ((obstacleSlowDistanceCm - distance) /
                (obstacleSlowDistanceCm - stopDistanceCm)).coerceIn(0f, 1f) * 0.55f
        } ?: 0f

        val ultrasonicReduction = ultrasonicCm?.let { distance ->
            if (distance <= stopDistanceCm) 1f
            else ((obstacleSlowDistanceCm - distance) /
                (obstacleSlowDistanceCm - stopDistanceCm)).coerceIn(0f, 1f) * 0.70f
        } ?: 0f

        // Combine the factors so obstacle crossing + curvature + narrow clearance produces a
        // meaningful slow-down instead of letting one factor mask the others.
        val reduction = (
            curveFactor * 0.45f +
                boundaryReduction * 0.60f +
                narrowReduction * 0.75f +
                cameraObstacleReduction +
                ultrasonicReduction
            ).coerceIn(0f, 0.82f)

        val speed = maxPwm - ((maxPwm - minMovingPwm) * reduction).toInt()
        return speed.coerceIn(slowestPwm, maxPwm)
    }
}

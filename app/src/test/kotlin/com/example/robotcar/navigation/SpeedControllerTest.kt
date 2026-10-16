package com.example.robotcar.navigation

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class SpeedControllerTest {
    private fun plan(angle: Float, gap: Float, nearest: Float?, boundary: BoundaryState = BoundaryState.NOMINAL) = ObstaclePlan(
        obstacleDetected = nearest != null, obstacleCountEstimate = 0, nearestObstacleCm = nearest,
        centerPathFeasible = true, nominalPathFeasible = true, expandedPathFeasible = true,
        bestPath = PathCandidate(0f, 65f, gap, angle, boundary == BoundaryState.CROSSING, true),
        centerGapWidthCm = gap, requiredGapWidthCm = 15f,
        nominalBoundaryHalfWidthCm = 17.5f, expandedBoundaryHalfWidthCm = 37.5f,
        nominalCenterOffsetCm = 10f, expandedCenterOffsetCm = 30f,
        boundaryState = boundary, pathType = PathType.STRAIGHT
    )

    @Test
    fun clearStraightRunsAtCruise() {
        assertEquals(40, SpeedController().speedFor(plan(0f, 100f, null), 0f, null))
    }

    @Test
    fun neverBelowMotorStallFloor() {
        val s = SpeedController().speedFor(plan(40f, 15f, 26f, BoundaryState.CROSSING), 40f, 31f)
        assertTrue(s >= SafetyConstants.MIN_MOTOR_PWM)
    }

    @Test
    fun speedFallsAsCurvatureRises() {
        val c = SpeedController()
        assertTrue(c.speedFor(plan(30f, 100f, null), 30f, null) < c.speedFor(plan(5f, 100f, null), 5f, null))
    }

    @Test
    fun stopPlanGivesZero() {
        val p = plan(0f, 100f, null).copy(pathType = PathType.STOP)
        assertEquals(0, SpeedController().speedFor(p, 0f, null))
    }
}

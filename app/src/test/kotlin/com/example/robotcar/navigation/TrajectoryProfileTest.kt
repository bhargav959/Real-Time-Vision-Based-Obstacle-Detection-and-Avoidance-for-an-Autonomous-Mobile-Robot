package com.example.robotcar.navigation

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class TrajectoryProfileTest {
    @Test
    fun startsAndEndsWithZeroLateralSlope() {
        assertEquals(0f, TrajectoryProfile.lateralFraction(0f), 0.0001f)
        assertEquals(1f, TrajectoryProfile.lateralFraction(1f), 0.0001f)
        assertEquals(0f, TrajectoryProfile.derivative(0f), 0.0001f)
        assertEquals(0f, TrajectoryProfile.derivative(1f), 0.0001f)
    }

    @Test
    fun curveHasLowerPeakSlopeThanCubicSmoothstep() {
        assertTrue(TrajectoryProfile.derivative(0.5f) <= 1.251f)
        assertTrue(TrajectoryProfile.derivative(0.5f) < 1.5f)
    }

    @Test
    fun stillAcquiresAvoidanceOffsetBeforeObstacleBand() {
        assertTrue(TrajectoryProfile.lateralFraction(0.67f) > 0.70f)
        assertTrue(TrajectoryProfile.lateralFraction(0.67f) < 0.72f)
        assertTrue(TrajectoryProfile.lateralFraction(0.75f) > 0.79f)
    }
}

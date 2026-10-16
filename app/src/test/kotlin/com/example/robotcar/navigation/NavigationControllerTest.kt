package com.example.robotcar.navigation

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class NavigationControllerTest {
    @Test
    fun ultrasonicBelowPivotMarginOverridesEverything() {
        val controller = NavigationController()
        val plan = ObstaclePlan(
            obstacleDetected = false,
            obstacleCountEstimate = 0,
            nearestObstacleCm = null,
            centerPathFeasible = true,
            nominalPathFeasible = true,
            expandedPathFeasible = true,
            bestPath = PathCandidate(0f, 55f, Float.POSITIVE_INFINITY, 0f, false, true),
            centerGapWidthCm = 50f,
            requiredGapWidthCm = 15f,
            nominalBoundaryHalfWidthCm = 17.5f,
            expandedBoundaryHalfWidthCm = 37.5f,
            nominalCenterOffsetCm = 10f,
            expandedCenterOffsetCm = 30f,
            boundaryState = BoundaryState.NOMINAL,
            pathType = PathType.STRAIGHT
        )
        val snapshot = PerceptionSnapshot(
            freeSpaceValid = true,
            obstaclePlan = plan,
            ultrasonicCm = 24f,
            ultrasonicAgeMs = 0L,
            ultrasonicRequired = true,
            cameraAgeMs = 0L,
            freeSpaceAgeMs = 0L
        )
        assertEquals(MotorCommand.STOP, controller.decide(snapshot, 1_000L).command)
    }


    @Test
    fun nearObstacleStopsTranslationButAllowsVerifiedPivotWithClearance() {
        val controller = NavigationController()
        val plan = clearPlan(angle = 32f).copy(nearestObstacleCm = 25f, obstacleDetected = true, centerPathFeasible = false)
        val snap = PerceptionSnapshot(
            freeSpaceValid = true,
            obstaclePlan = plan,
            ultrasonicCm = 27f,
            ultrasonicAgeMs = 0L,
            ultrasonicRequired = true,
            cameraAgeMs = 0L,
            freeSpaceAgeMs = 0L
        )
        assertEquals(MotorCommand.RIGHT, controller.decide(snap, 1_000L).command)

        val tooClose = snap.copy(ultrasonicCm = 22f)
        assertEquals(MotorCommand.STOP, controller.decide(tooClose, 1_100L).command)
    }

    @Test
    fun blockedCenterAlwaysIssuesTheChosenLeftOrRightAvoidanceCommandEvenForSmallAngles() {
        val leftController = NavigationController()
        val rightController = NavigationController()
        val left = leftController.decide(
            avoidanceSnapshot(angle = -2f, lateral = -2.5f), 1_000L
        )
        val right = rightController.decide(
            avoidanceSnapshot(angle = 2f, lateral = 2.5f), 1_000L
        )

        // The generic heading classifier used to return FORWARD for sub-5-degree angles. When the
        // centre corridor is blocked, that can ignore a valid side path and drive toward the hazard.
        assertEquals(MotorCommand.SLIGHT_LEFT, left.command)
        assertEquals(MotorCommand.SLIGHT_RIGHT, right.command)
    }

    @Test
    fun closeObstacleRequestsDirectionSpecificPivotOnlyWhenClearanceIsVerified() {
        val leftController = NavigationController()
        val rightController = NavigationController()
        val leftSafe = leftController.decide(
            avoidanceSnapshot(angle = -3f, lateral = -3f, nearest = 25f).copy(
                ultrasonicCm = 27f, ultrasonicAgeMs = 0L, ultrasonicRequired = true
            ), 1_000L
        )
        val rightSafe = rightController.decide(
            avoidanceSnapshot(angle = 3f, lateral = 3f, nearest = 25f).copy(
                ultrasonicCm = 27f, ultrasonicAgeMs = 0L, ultrasonicRequired = true
            ), 1_000L
        )
        assertEquals(MotorCommand.LEFT, leftSafe.command)
        assertEquals(MotorCommand.RIGHT, rightSafe.command)

        val tooClose = NavigationController().decide(
            avoidanceSnapshot(angle = 3f, lateral = 3f, nearest = 25f).copy(
                ultrasonicCm = 22f, ultrasonicAgeMs = 0L, ultrasonicRequired = true
            ), 1_100L
        )
        assertEquals(MotorCommand.STOP, tooClose.command)

        val narrowPivot = NavigationController().decide(
            avoidanceSnapshot(angle = 3f, lateral = 3f, nearest = 25f).copy(
                obstaclePlan = avoidanceSnapshot(angle = 3f, lateral = 3f, nearest = 25f)
                    .obstaclePlan!!.copy(
                        bestPath = avoidanceSnapshot(angle = 3f, lateral = 3f, nearest = 25f)
                            .obstaclePlan!!.bestPath!!.copy(minimumGapWidthCm = 16f)
                    ),
                ultrasonicCm = 27f, ultrasonicAgeMs = 0L, ultrasonicRequired = true
            ), 1_200L
        )
        assertEquals(MotorCommand.STOP, narrowPivot.command)
    }

    @Test
    fun staleFreeSpaceStopsInsteadOfActingOnOldGeometry() {
        val controller = NavigationController()
        val snapshot = PerceptionSnapshot(
            freeSpaceValid = true,
            obstaclePlan = null,
            cameraAgeMs = 0L,
            freeSpaceAgeMs = 601L
        )
        assertEquals(MotorCommand.STOP, controller.decide(snapshot, 1_000L).command)
    }

    @Test
    fun speedDropsOnCurvedBoundaryCrossing() {
        val controller = NavigationController()
        val plan = ObstaclePlan(
            obstacleDetected = true,
            obstacleCountEstimate = 2,
            nearestObstacleCm = 45f,
            centerPathFeasible = false,
            nominalPathFeasible = false,
            expandedPathFeasible = true,
            bestPath = PathCandidate(
                lateralOffsetCm = 16f,
                targetDistanceCm = 65f,
                minimumGapWidthCm = 15.5f,
                headingAngleDeg = 16f,
                crossesBoundary = true,
                feasible = true
            ),
            centerGapWidthCm = 9f,
            requiredGapWidthCm = 15f,
            nominalBoundaryHalfWidthCm = 17.5f,
            expandedBoundaryHalfWidthCm = 37.5f,
            nominalCenterOffsetCm = 10f,
            expandedCenterOffsetCm = 30f,
            boundaryState = BoundaryState.CROSSING,
            pathType = PathType.AVOIDANCE_CURVE
        )
        val decision = controller.decide(
            PerceptionSnapshot(freeSpaceValid = true, obstaclePlan = plan, cameraAgeMs = 0L, freeSpaceAgeMs = 0L),
            1_000L
        )
        assertTrue(decision.speedPwm <= MotorCalibration.ARDUINO_MAX_PWM)
        assertEquals(PathType.AVOIDANCE_CURVE, decision.pathType)
        assertEquals(BoundaryState.CROSSING, decision.boundaryState)
    }

    private fun avoidanceSnapshot(
        angle: Float,
        lateral: Float,
        nearest: Float? = null
    ) = PerceptionSnapshot(
        freeSpaceValid = true,
        obstaclePlan = clearPlan(angle).copy(
            obstacleDetected = true,
            nearestObstacleCm = nearest,
            centerPathFeasible = false,
            nominalPathFeasible = true,
            expandedPathFeasible = true,
            bestPath = PathCandidate(
                lateralOffsetCm = lateral,
                targetDistanceCm = 75f,
                minimumGapWidthCm = 25f,
                headingAngleDeg = angle,
                crossesBoundary = false,
                feasible = true
            ),
            pathType = PathType.AVOIDANCE_CURVE
        ),
        cameraAgeMs = 0L,
        freeSpaceAgeMs = 0L
    )

    private fun clearPlan(angle: Float = 0f) = ObstaclePlan(
        obstacleDetected = false,
        obstacleCountEstimate = 0,
        nearestObstacleCm = null,
        centerPathFeasible = true,
        nominalPathFeasible = true,
        expandedPathFeasible = true,
        bestPath = PathCandidate(0f, 65f, Float.POSITIVE_INFINITY, angle, false, true),
        centerGapWidthCm = 50f,
        requiredGapWidthCm = 15f,
        nominalBoundaryHalfWidthCm = 17.5f,
        expandedBoundaryHalfWidthCm = 37.5f,
        nominalCenterOffsetCm = 10f,
        expandedCenterOffsetCm = 30f,
        boundaryState = BoundaryState.NOMINAL,
        pathType = PathType.STRAIGHT
    )

    private fun snap(
        angle: Float = 0f,
        sign: TrafficSign? = null,
        required: Boolean = false,
        usAge: Long = 0L,
        usCm: Float? = null,
        noEcho: Boolean = false
    ) = PerceptionSnapshot(
        freeSpaceValid = true,
        obstaclePlan = clearPlan(angle),
        trafficSign = sign,
        trafficSignAgeMs = if (sign != null) 0L else Long.MAX_VALUE,
        ultrasonicCm = usCm,
        ultrasonicNoEcho = noEcho,
        ultrasonicAgeMs = usAge,
        ultrasonicRequired = required,
        cameraAgeMs = 0L,
        freeSpaceAgeMs = 0L
    )

    @Test
    fun staleUltrasonicStopsWhenRobotAttachedButNotInVirtualMode() {
        val c = NavigationController()
        assertEquals(MotorCommand.STOP, c.decide(snap(required = true, usAge = 801L), 1_000L).command)
        assertEquals(MotorCommand.FORWARD, c.decide(snap(required = true, usAge = 100L), 1_000L).command)
        assertEquals(MotorCommand.FORWARD, c.decide(snap(required = false, usAge = Long.MAX_VALUE), 1_000L).command)
    }

    @Test
    fun missingUltrasonicNeverCountsAsClearWhenRequired() {
        val c = NavigationController()
        // required, never received a reading: age is MAX_VALUE
        assertEquals(MotorCommand.STOP, c.decide(snap(required = true, usAge = Long.MAX_VALUE), 1_000L).command)
    }

    @Test
    fun ultrasonicBetween25And30cmNowStops() {
        val c = NavigationController()
        assertEquals(MotorCommand.STOP, c.decide(snap(required = true, usCm = 29f), 1_000L).command)
        assertEquals(MotorCommand.FORWARD, c.decide(snap(required = true, usCm = 31f), 1_000L).command)
    }

    @Test
    fun noEchoCapsSpeed() {
        val c = NavigationController()
        val d = c.decide(snap(required = true, noEcho = true), 1_000L)
        assertEquals(MotorCommand.FORWARD, d.command)
        assertTrue(d.speedPwm <= SafetyConstants.NO_ECHO_MAX_PWM)
    }

    @Test
    fun stopSignStopsOnceThenLetsRobotLeave() {
        val c = NavigationController(stopHoldMs = 2_000L, stopCooldownMs = 8_000L)
        val sign = TrafficSign("stop", 0.9f)
        assertEquals(MotorCommand.STOP, c.decide(snap(sign = sign), 1_000L).command)   // latch
        assertEquals(MotorCommand.STOP, c.decide(snap(sign = sign), 2_500L).command)   // still held
        assertEquals(MotorCommand.FORWARD, c.decide(snap(sign = sign), 3_100L).command) // cooldown: proceeds
        assertEquals(MotorCommand.FORWARD, c.decide(snap(sign = sign), 9_000L).command)
        assertEquals(MotorCommand.STOP, c.decide(snap(sign = sign), 11_500L).command)  // a new stop sign
    }

    @Test
    fun noEntryAheadStopsThenReleasesWhenSignGone() {
        val c = NavigationController(noEntryMemoryMs = 1_500L)
        val sign = TrafficSign("no_entry", 0.9f, centerX = 0.5f)
        assertEquals(MotorCommand.STOP, c.decide(snap(sign = sign), 1_000L).command)
        assertEquals(MotorCommand.STOP, c.decide(snap(), 2_000L).command)
        assertEquals(MotorCommand.FORWARD, c.decide(snap(), 3_000L).command)
    }

    @Test
    fun noEntryOnLeftOnlyVetoesLeftward() {
        val c = NavigationController()
        val sign = TrafficSign("no_entry", 0.9f, centerX = 0.15f)
        assertEquals(MotorCommand.STOP, c.decide(snap(angle = -15f, sign = sign), 1_000L).command)
        assertEquals(MotorCommand.SLIGHT_RIGHT, c.decide(snap(angle = 15f, sign = sign), 1_100L).command)
        assertEquals(MotorCommand.FORWARD, c.decide(snap(angle = 0f, sign = sign), 1_200L).command)
    }

    @Test
    fun headingHysteresisPreventsFlipFlop() {
        val c = NavigationController()
        assertEquals(MotorCommand.FORWARD, c.decide(snap(angle = 4f), 1_000L).command)
        assertEquals(MotorCommand.SLIGHT_RIGHT, c.decide(snap(angle = 6f), 1_100L).command)
        // 4 deg is inside the hold band once already turning
        assertEquals(MotorCommand.SLIGHT_RIGHT, c.decide(snap(angle = 4f), 1_200L).command)
        assertEquals(MotorCommand.FORWARD, c.decide(snap(angle = 2f), 1_300L).command)
        assertEquals(MotorCommand.RIGHT, c.decide(snap(angle = 25f), 1_400L).command)
        assertEquals(MotorCommand.RIGHT, c.decide(snap(angle = 20f), 1_500L).command)
        assertEquals(MotorCommand.SLIGHT_RIGHT, c.decide(snap(angle = 15f), 1_600L).command)
    }
}

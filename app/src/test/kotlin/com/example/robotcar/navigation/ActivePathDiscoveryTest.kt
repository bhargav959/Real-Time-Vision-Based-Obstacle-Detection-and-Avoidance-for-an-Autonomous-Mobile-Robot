package com.example.robotcar.navigation

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class ActivePathDiscoveryTest {
    private val scanner = ActivePathDiscovery(allowBlindBackoff = true)

    @Test
    fun startsWithBoundedBackoffThenLeftProbeAfterPersistentBlockedFramesAndClearance() {
        assertEquals(MotorCommand.STOP, scanner.update(blocked(), stopped(), 0L, false).decision.command)
        assertEquals(MotorCommand.STOP, scanner.update(blocked(), stopped(), 100L, false).decision.command)
        val start = scanner.update(blocked(), stopped(), 200L, false)
        assertEquals(MotorCommand.BACKOFF, start.decision.command)
        assertFalse(start.isRotationPulse)
        assertEquals(ActivePathDiscovery.State.BACKOFF, start.state)
        val postReverseWait = scanner.update(blocked(), stopped(), 500L, true)
        assertEquals(MotorCommand.STOP, postReverseWait.decision.command)
        assertEquals(ActivePathDiscovery.State.WAIT_CLEARANCE, postReverseWait.state)
        val stillWaiting = scanner.update(blocked(), stopped(), 700L, true)
        assertEquals(MotorCommand.STOP, stillWaiting.decision.command)
        val leftProbe = scanner.update(blocked(), stopped(), 800L, true)
        assertEquals(MotorCommand.PIVOT_LEFT, leftProbe.decision.command)
        assertTrue(leftProbe.isRotationPulse)
        assertEquals(ActivePathDiscovery.State.TURN_LEFT, leftProbe.state)
    }

    @Test
    fun backsOffButWaitsForVerifiedTurnClearance() {
        scanner.update(blocked(), stopped(), 0L, false)
        scanner.update(blocked(), stopped(), 100L, false)
        val backoff = scanner.update(blocked(), stopped(), 200L, false)
        assertEquals(MotorCommand.BACKOFF, backoff.decision.command)
        assertEquals(ActivePathDiscovery.State.BACKOFF, backoff.state)
        val wait = scanner.update(blocked(), stopped(), 500L, false)
        assertEquals(MotorCommand.STOP, wait.decision.command)
        assertEquals(ActivePathDiscovery.State.WAIT_CLEARANCE, wait.state)
        assertFalse(wait.isRotationPulse)
    }

    @Test
    fun nearUltrasonicStopAllowsScanOnlyWhileClearanceRemainsVerified() {
        scanner.update(blocked(), stopped(), 0L, true)
        scanner.update(blocked(), stopped(), 100L, true)
        scanner.update(blocked(), stopped(), 200L, true) // bounded backoff begins
        scanner.update(blocked(), stopped(), 450L, true) // first left pulse
        val safety = stopped("ultrasonic < 30 cm: forward motion inhibited")
        val output = scanner.update(blocked(), safety, 550L, true)
        assertEquals(MotorCommand.PIVOT_LEFT, output.decision.command)
        assertTrue(output.isRotationPulse)

        val noClearance = scanner.update(blocked(), safety, 600L, false)
        assertEquals(MotorCommand.STOP, noClearance.decision.command)
        assertFalse(noClearance.isRotationPulse)
        assertTrue(noClearance.state == ActivePathDiscovery.State.SAFETY_HOLD)
    }

    @Test
    fun closeObstacleMayTriggerBackoffButNeverImmediateTurn() {
        val near = blocked().copy(ultrasonicRequired = true, ultrasonicCm = 25f, ultrasonicAgeMs = 0L)
        val nearStop = stopped("ultrasonic < 30 cm")
        scanner.update(near, nearStop, 0L, false)
        scanner.update(near, nearStop, 100L, false)
        val backoff = scanner.update(near, nearStop, 200L, false)
        assertEquals(MotorCommand.BACKOFF, backoff.decision.command)
        val closerToClear = near.copy(ultrasonicCm = 31f)
        val waiting = scanner.update(closerToClear, stopped(), 500L, false)
        assertEquals(ActivePathDiscovery.State.WAIT_CLEARANCE, waiting.state)
        assertEquals(MotorCommand.STOP, waiting.decision.command)
        val clearEnough = near.copy(ultrasonicCm = 36f)
        val waitForFreshMap = scanner.update(clearEnough, stopped(), 600L, true)
        assertEquals(MotorCommand.STOP, waitForFreshMap.decision.command)
        assertEquals(ActivePathDiscovery.State.WAIT_CLEARANCE, waitForFreshMap.state)
        val left = scanner.update(clearEnough, stopped(), 800L, true)
        assertEquals(MotorCommand.PIVOT_LEFT, left.decision.command)
        assertEquals(ActivePathDiscovery.State.TURN_LEFT, left.state)
    }


    @Test
    fun nearSonarCanTriggerBackoffWhenCenterIsBlockedButSideCandidateExists() {
        val sideCandidatePlan = clearPlan().copy(
            obstacleDetected = true,
            centerPathFeasible = false,
            bestPath = PathCandidate(18f, 75f, 20f, 15f, true, true),
            pathType = PathType.AVOIDANCE_CURVE
        )
        val near = PerceptionSnapshot(
            freeSpaceValid = true,
            freeSpaceAgeMs = 0L,
            cameraAgeMs = 0L,
            obstaclePlan = sideCandidatePlan,
            ultrasonicRequired = true,
            ultrasonicAgeMs = 0L,
            ultrasonicCm = 26f
        )
        val inhibited = stopped("ultrasonic < 30 cm: forward motion inhibited; verified pivot/backoff required")
        scanner.update(near, inhibited, 0L, false)
        scanner.update(near, inhibited, 100L, false)
        val output = scanner.update(near, inhibited, 200L, false)
        assertEquals(MotorCommand.BACKOFF, output.decision.command)
        assertEquals(ActivePathDiscovery.State.BACKOFF, output.state)
    }

    @Test
    fun newlyFeasiblePathDoesNotCancelReverseBeforeFreshPostReverseMap() {
        val discovery = ActivePathDiscovery()
        discovery.update(blocked(), stopped(), 0L, false)
        discovery.update(blocked(), stopped(), 100L, false)
        discovery.update(blocked(), stopped(), 200L, false)
        val plan = clearPlan()
        val snapshot = PerceptionSnapshot(
            freeSpaceValid = true,
            freeSpaceAgeMs = 0L,
            obstaclePlan = plan
        )
        val forward = NavigationDecision(
            MotorCommand.FORWARD, 90, "clear explicit free path; straight preferred",
            PathType.STRAIGHT, BoundaryState.NOMINAL
        )
        val stillBacking = discovery.update(snapshot, forward, 300L, true)
        assertEquals(MotorCommand.BACKOFF, stillBacking.decision.command)
        assertEquals(ActivePathDiscovery.State.BACKOFF, stillBacking.state)
        val waiting = discovery.update(snapshot, forward, 500L, true)
        assertEquals(MotorCommand.STOP, waiting.decision.command)
        assertEquals(ActivePathDiscovery.State.WAIT_CLEARANCE, waiting.state)
        assertTrue(waiting.ownsOutput)
    }

    @Test
    fun scanRunsConsecutiveLowSpeedPulsesAndStopsOnlyAfterTheLeftSweep() {
        scanner.update(blocked(), stopped(), 0L, true)
        scanner.update(blocked(), stopped(), 100L, true)
        scanner.update(blocked(), stopped(), 200L, true)
        val firstTurn = scanner.update(blocked(), stopped(), 500L, true)
        assertEquals(MotorCommand.PIVOT_LEFT, firstTurn.decision.command)
        assertEquals(MotorCalibration.SCAN_START_PWM, firstTurn.decision.speedPwm)
        assertTrue(firstTurn.isRotationPulse)
        assertEquals(100L, MotorCalibration.SCAN_PULSE_DURATION_MS)

        // The next 100 ms tick refreshes the held pivot instead of stopping for a sample wait.
        val secondTurn = scanner.update(blocked(), stopped(), 600L, true)
        assertEquals(MotorCommand.PIVOT_LEFT, secondTurn.decision.command)
        assertEquals(ActivePathDiscovery.State.TURN_LEFT, secondTurn.state)
        // Without explicit gyro feedback, speed only rises after two pulses and remains capped.
        assertTrue(secondTurn.decision.speedPwm <= MotorCalibration.SCAN_MAX_PWM)
        assertTrue(secondTurn.isRotationPulse)
        scanner.update(blocked(), stopped(), 700L, true)
        scanner.update(blocked(), stopped(), 800L, true)
        val endOfSweep = scanner.update(blocked(), stopped(), 900L, true)
        assertEquals(MotorCommand.STOP, endOfSweep.decision.command)
        assertEquals(ActivePathDiscovery.State.SETTLE_LEFT, endOfSweep.state)
        assertFalse(endOfSweep.isRotationPulse)
    }

    @Test
    fun activeScanNeverCommandsAboveNoEchoSpeedCap() {
        scanner.update(blocked(), stopped(), 0L, true)
        scanner.update(blocked(), stopped(), 100L, true)
        scanner.update(blocked(), stopped(), 200L, true)
        val freshSonar = blocked().copy(
            ultrasonicRequired = true,
            ultrasonicNoEcho = false,
            ultrasonicCm = 36f,
            ultrasonicAgeMs = 0L
        )
        val left = scanner.update(freshSonar, stopped(), 500L, true, RotationFeedback(0f, 0f))
        assertEquals(MotorCommand.PIVOT_LEFT, left.decision.command)

        val noEcho = freshSonar.copy(ultrasonicNoEcho = true, ultrasonicCm = null)
        var t = 600L
        var output = scanner.update(noEcho, stopped(), t, true, RotationFeedback(0f, 0f))
        repeat(5) {
            assertTrue("no-echo scan command must respect safety PWM cap", output.decision.speedPwm <= SafetyConstants.NO_ECHO_MAX_PWM)
            t += 100L
            output = scanner.update(noEcho, stopped(), t, true, RotationFeedback(0f, 0f))
        }
    }

    @Test
    fun boundedSweepFinishesStoppedWhenNoPathIsFound() {
        var t = 0L
        var output = scanner.update(blocked(), stopped(), t, true)
        repeat(3) { t += 100L; output = scanner.update(blocked(), stopped(), t, true) }
        repeat(75) { t += 100L; output = scanner.update(blocked(), stopped(), t, true) }
        assertEquals(MotorCommand.STOP, output.decision.command)
        assertEquals(ActivePathDiscovery.State.NO_ROUTE, output.state)
        assertFalse(output.isRotationPulse)
    }


    @Test
    fun boundedBackoffStopsForPostReverseScanBeforeNewlyFeasiblePathTakesOver() {
        val near = blocked().copy(ultrasonicRequired = true, ultrasonicCm = 25f, ultrasonicAgeMs = 0L)
        val nearStop = stopped("ultrasonic < 30 cm: forward motion inhibited")
        scanner.update(near, nearStop, 0L, false)
        scanner.update(near, nearStop, 100L, false)
        assertEquals(MotorCommand.BACKOFF, scanner.update(near, nearStop, 200L, false).decision.command)

        val clear = PerceptionSnapshot(
            freeSpaceValid = true,
            freeSpaceAgeMs = 0L,
            cameraAgeMs = 0L,
            obstaclePlan = clearPlan(),
            ultrasonicRequired = true,
            ultrasonicAgeMs = 0L,
            ultrasonicCm = 32f
        )
        val forward = NavigationDecision(
            MotorCommand.FORWARD, 90, "clear explicit free path; straight preferred",
            PathType.STRAIGHT, BoundaryState.NOMINAL
        )
        val stillBacking = scanner.update(clear, forward, 300L, true)
        assertEquals(ActivePathDiscovery.State.BACKOFF, stillBacking.state)
        assertEquals(MotorCommand.BACKOFF, stillBacking.decision.command)
        val waiting = scanner.update(clear, forward, 500L, true)
        assertEquals(ActivePathDiscovery.State.WAIT_CLEARANCE, waiting.state)
        assertEquals(MotorCommand.STOP, waiting.decision.command)
        val postReverseSampleReady = scanner.update(clear, forward, 800L, true)
        assertEquals(ActivePathDiscovery.State.TURN_LEFT, postReverseSampleReady.state)
        assertEquals(MotorCommand.PIVOT_LEFT, postReverseSampleReady.decision.command)
    }


    @Test
    fun defaultConfigurationAllowsOnlyBoundedBackoffWhenPivotClearanceIsNotVerified() {
        val discovery = ActivePathDiscovery()
        val near = blocked().copy(ultrasonicRequired = true, ultrasonicCm = 25f, ultrasonicAgeMs = 0L)
        val nearStop = stopped("ultrasonic < 30 cm: forward motion inhibited; verified pivot/backoff required")
        discovery.update(near, nearStop, 0L, false)
        discovery.update(near, nearStop, 100L, false)
        val output = discovery.update(near, nearStop, 200L, false)
        assertEquals(MotorCommand.BACKOFF, output.decision.command)
        assertEquals(ActivePathDiscovery.State.BACKOFF, output.state)
    }

    @Test
    fun backoffDoesNotResumeStraightIntoAnObstacleWhenAPathCandidateAppears() {
        val discovery = ActivePathDiscovery()
        val near = blocked().copy(ultrasonicRequired = true, ultrasonicCm = 25f, ultrasonicAgeMs = 0L)
        val nearStop = stopped("ultrasonic < 30 cm: forward motion inhibited; verified pivot/backoff required")
        discovery.update(near, nearStop, 0L, false)
        discovery.update(near, nearStop, 100L, false)
        assertEquals(MotorCommand.BACKOFF, discovery.update(near, nearStop, 200L, false).decision.command)

        val sideCandidate = clearPlan().copy(
            obstacleDetected = true,
            centerPathFeasible = false,
            bestPath = PathCandidate(18f, 75f, 20f, 15f, true, true),
            pathType = PathType.AVOIDANCE_CURVE
        )
        val snapshot = PerceptionSnapshot(
            freeSpaceValid = true,
            freeSpaceAgeMs = 0L,
            cameraAgeMs = 0L,
            obstaclePlan = sideCandidate,
            ultrasonicRequired = true,
            ultrasonicAgeMs = 0L,
            ultrasonicCm = 35f
        )
        // Simulate a planner bug that proposes straight F despite the blocked centre corridor.
        val unsafeStraight = NavigationDecision(
            MotorCommand.FORWARD, 25, "clear explicit free path; straight preferred",
            PathType.STRAIGHT, BoundaryState.BLOCKED
        )
        val afterBackoff = discovery.update(snapshot, unsafeStraight, 500L, false)
        assertEquals(ActivePathDiscovery.State.WAIT_CLEARANCE, afterBackoff.state)
        assertEquals(MotorCommand.STOP, afterBackoff.decision.command)
    }

    @Test
    fun completedScanRejectsForwardWhenObstacleStillBlocksCenterCorridor() {
        val discovery = ActivePathDiscovery()
        val blocked = blocked()
        discovery.update(blocked, stopped(), 0L, true)
        discovery.update(blocked, stopped(), 100L, true)
        discovery.update(blocked, stopped(), 200L, true)

        val sideCandidate = clearPlan().copy(
            obstacleDetected = true,
            centerPathFeasible = false,
            bestPath = PathCandidate(18f, 75f, 20f, 15f, true, true),
            pathType = PathType.AVOIDANCE_CURVE
        )
        val snapshot = PerceptionSnapshot(
            freeSpaceValid = true,
            freeSpaceAgeMs = 0L,
            cameraAgeMs = 0L,
            obstaclePlan = sideCandidate
        )
        val unsafeStraight = NavigationDecision(
            MotorCommand.FORWARD, 25, "clear explicit free path; straight preferred",
            PathType.STRAIGHT, BoundaryState.BLOCKED
        )
        var output = discovery.update(snapshot, unsafeStraight, 300L, true)
        var t = 400L
        repeat(40) {
            output = discovery.update(snapshot, unsafeStraight, t, true)
            t += 100L
        }
        assertTrue(output.state == ActivePathDiscovery.State.NO_ROUTE || output.decision.command != MotorCommand.FORWARD)
        assertEquals(MotorCommand.STOP, output.decision.command)
    }

    private fun blocked() = PerceptionSnapshot(
        freeSpaceValid = true,
        freeSpaceAgeMs = 0L,
        obstaclePlan = ObstaclePlan(
            obstacleDetected = true,
            obstacleCountEstimate = 1,
            nearestObstacleCm = 42f,
            centerPathFeasible = false,
            nominalPathFeasible = false,
            expandedPathFeasible = false,
            bestPath = null,
            centerGapWidthCm = 0f,
            requiredGapWidthCm = 19f,
            nominalBoundaryHalfWidthCm = 17.5f,
            expandedBoundaryHalfWidthCm = 37.5f,
            nominalCenterOffsetCm = 10f,
            expandedCenterOffsetCm = 30f,
            boundaryState = BoundaryState.BLOCKED,
            pathType = PathType.STOP
        )
    )

    private fun stopped(reason: String = "no feasible path within expanded boundary") = NavigationDecision(
        MotorCommand.STOP, 0, reason, PathType.STOP, BoundaryState.BLOCKED
    )

    private fun clearPlan() = ObstaclePlan(
        obstacleDetected = true,
        obstacleCountEstimate = 1,
        nearestObstacleCm = 42f,
        centerPathFeasible = true,
        nominalPathFeasible = true,
        expandedPathFeasible = true,
        bestPath = PathCandidate(0f, 75f, 30f, 0f, false, true),
        centerGapWidthCm = 30f,
        requiredGapWidthCm = 19f,
        nominalBoundaryHalfWidthCm = 17.5f,
        expandedBoundaryHalfWidthCm = 37.5f,
        nominalCenterOffsetCm = 10f,
        expandedCenterOffsetCm = 30f,
        boundaryState = BoundaryState.NOMINAL,
        pathType = PathType.STRAIGHT
    )
}

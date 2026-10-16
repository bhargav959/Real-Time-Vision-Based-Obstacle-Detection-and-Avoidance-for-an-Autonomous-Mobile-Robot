package com.example.robotcar.navigation

import com.example.robotcar.communication.SerialProtocol
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class V0511Test {
    // ---- sonar piggyback / parsing ---------------------------------------------------------
    @Test
    fun parsesSonarFromEveryLineShape() {
        assertEquals(42.5f, SerialProtocol.parseUltrasonic("DISTANCE=42.5")!!.cm)
        assertTrue(SerialProtocol.parseUltrasonic("DISTANCE=NO_ECHO")!!.noEcho)
        assertEquals(41.2f, SerialProtocol.parseUltrasonic("RESULT=F,90,D=41.2")!!.cm)
        assertTrue(SerialProtocol.parseUltrasonic("RESULT=SL,80,D=NO_ECHO")!!.noEcho)
        assertEquals(27.5f, SerialProtocol.parseUltrasonic("RESULT=STOP_OBSTACLE:27.5")!!.cm)
        assertEquals(19.0f, SerialProtocol.parseUltrasonic("RESULT=STOP_TURN_OBSTACLE:19.0")!!.cm)
    }

    @Test
    fun garbageSonarLinesAreNeverTreatedAsClear() {
        assertNull(SerialProtocol.parseUltrasonic("DISTANCE=abc"))
        assertNull(SerialProtocol.parseUltrasonic("DISTANCE=-3"))
        assertNull(SerialProtocol.parseUltrasonic("DISTANCE=0"))
        assertNull(SerialProtocol.parseUltrasonic("RESULT=F,90"))
        assertNull(SerialProtocol.parseUltrasonic("RESULT=F,90,D=nan"))
        assertNull(SerialProtocol.parseUltrasonic("ROBOT_READY"))
        assertNull(SerialProtocol.parseUltrasonic("RESULT=BACKOFF_COMPLETE"))
    }

    // ---- stabilizer ------------------------------------------------------------------------
    @Test
    fun sameSideSteeringFlickerIsSmoothButOppositeArcNeedsConfirmation() {
        val st = LiveDecisionStabilizer()
        st.apply(motion(MotorCommand.FORWARD, 100)); st.apply(motion(MotorCommand.FORWARD, 100))
        val sl = st.apply(motion(MotorCommand.SLIGHT_LEFT, 100))
        assertEquals(MotorCommand.SLIGHT_LEFT, sl.command)
        assertTrue(sl.speedPwm > 0)
        assertEquals(MotorCommand.FORWARD, st.apply(motion(MotorCommand.FORWARD, 100)).command)
        assertEquals(MotorCommand.STOP, st.apply(motion(MotorCommand.SLIGHT_RIGHT, 100)).command)
        assertEquals(MotorCommand.SLIGHT_RIGHT, st.apply(motion(MotorCommand.SLIGHT_RIGHT, 100)).command)
    }

    @Test
    fun pivotsStillRequireStopAndConfirmation() {
        val st = LiveDecisionStabilizer()
        st.apply(motion(MotorCommand.FORWARD, 100)); st.apply(motion(MotorCommand.FORWARD, 100))
        assertEquals(MotorCommand.STOP, st.apply(motion(MotorCommand.LEFT, 80)).command)
    }

    // ---- yaw tracker -----------------------------------------------------------------------
    @Test
    fun yawTrackerIntegratesLeftTurnPositiveForAnyMounting() {
        // Phone lying flat: up = +z. Gyro z = +1 rad/s (CCW seen from above) for 1 s => +57.3 deg.
        val flat = YawTracker()
        flat.onGravity(0f, 0f, 9.81f)
        var t = 0L
        repeat(101) { flat.onGyro(0f, 0f, 1f, t); t += 10_000_000L }
        assertEquals(57.3f, flat.yawDegrees(), 1.0f)

        // Phone mounted upright (portrait): up = +y. Rotation about the vertical = gyro y.
        val upright = YawTracker()
        upright.onGravity(0f, 9.81f, 0f)
        t = 0L
        repeat(101) { upright.onGyro(0f, -0.5f, 0f, t); t += 10_000_000L }
        assertEquals(-28.65f, upright.yawDegrees(), 1.0f) // right turn negative
    }

    @Test
    fun yawTrackerIgnoresRotationAboutHorizontalAxes() {
        val y = YawTracker(); y.onGravity(0f, 0f, 9.81f)
        var t = 0L
        repeat(101) { y.onGyro(2f, 2f, 0f, t); t += 10_000_000L } // pitch/roll shake only
        assertEquals(0f, y.yawDegrees(), 0.5f)
    }

    // ---- scan: gyro verification and post-stop frame gating --------------------------------
    @Test
    fun ineffectivePivotEscalatesPwmThenAbortsInsteadOfPretendingToScan() {
        val s = ActivePathDiscovery()
        driveToFirstLeftPulse(s)
        var t = 500L
        val pwms = mutableSetOf<Int>()
        var aborted = false
        var yaw = 0f // gyro never sees any rotation: chassis is stalled
        repeat(40) {
            t += 100L
            val o = s.update(blocked(), stopped(), t, true, RotationFeedback(yaw, 0f))
            if (o.isRotationPulse) pwms += o.decision.speedPwm
            if (o.state == ActivePathDiscovery.State.UNSAFE) aborted = true
        }
        assertTrue("pwm escalated: $pwms", pwms.size > 1 && pwms.max() == MotorCalibration.SCAN_MAX_PWM)
        assertTrue("never above max", pwms.all { it <= MotorCalibration.SCAN_MAX_PWM })
        assertTrue("scan aborted with a stalled chassis", aborted)
    }

    @Test
    fun effectivePivotKeepsDefaultPwm() {
        val s = ActivePathDiscovery()
        driveToFirstLeftPulse(s)
        var t = 500L
        var yaw = 2f // the first pulse (started above at yaw 0) already turned 2 deg
        val pwms = mutableSetOf<Int>()
        repeat(60) {
            t += 100L
            val o = s.update(blocked(), stopped(), t, true, RotationFeedback(yaw, 0f))
            if (o.isRotationPulse) { pwms += o.decision.speedPwm; yaw += if (o.decision.command == MotorCommand.PIVOT_LEFT) 2f else -2f }
        }
        assertEquals(setOf(MotorCalibration.SCAN_START_PWM), pwms)
    }

    @Test
    fun scanWaitsForAFrameCapturedAfterTheCompletedLeftSweepStop() {
        val s = ActivePathDiscovery()
        driveToFirstLeftPulse(s)
        s.update(blocked(), stopped(), 600L, true) // second left pulse
        s.update(blocked(), stopped(), 700L, true) // third left pulse
        s.update(blocked(), stopped(), 800L, true) // fourth left pulse
        s.update(blocked(), stopped(), 900L, true) // -> SETTLE_LEFT
        // Map captured before sweep stop must not be accepted as the left-side sample.
        val old = s.update(blocked().copy(freeSpaceAgeMs = 300L), stopped(), 1100L, true)
        assertEquals(ActivePathDiscovery.State.SETTLE_LEFT, old.state)
        assertEquals(MotorCommand.STOP, old.decision.command)
        // A recent map captured after the sweep stop is usable and starts the right sweep.
        val fresh = s.update(blocked().copy(freeSpaceAgeMs = 50L), stopped(), 1200L, true)
        assertEquals(MotorCommand.PIVOT_RIGHT, fresh.decision.command)
    }

    @Test
    fun scanAbortsIfNoPostSweepFrameEverArrives() {
        val s = ActivePathDiscovery()
        driveToFirstLeftPulse(s)
        s.update(blocked(), stopped(), 600L, true)
        s.update(blocked(), stopped(), 700L, true)
        s.update(blocked(), stopped(), 800L, true)
        s.update(blocked(), stopped(), 900L, true) // sweep complete, stop for stable view
        var sawUnsafe = false
        var sawPivotAgain = false
        var t = 900L
        while (t < 900L + MotorCalibration.SCAN_MAX_SETTLE_MS + 300L) {
            t += 100L
            val o = s.update(blocked().copy(freeSpaceAgeMs = 450L + (t - 900L)), stopped(), t, true)
            if (o.state == ActivePathDiscovery.State.UNSAFE) sawUnsafe = true
            if (o.isRotationPulse) sawPivotAgain = true
        }
        assertTrue(sawUnsafe)
        assertFalse(sawPivotAgain)
    }

    // ---- camera offset ---------------------------------------------------------------------
    @Test
    fun cameraForwardOffsetShiftsPlannerDistancesToThePivotCentre() {
        fun nearest(offset: Float): Float {
            val cfg = NavigationGeometryConfig(
                robotWidthCm = 15f, robotLengthCm = 25.5f, cameraHeightCm = 10f, cameraTiltDeg = 7.5f,
                targetLookaheadCm = 65f, cameraForwardOffsetCm = offset
            )
            val proj = CameraGroundProjector(cfg)
            // A 20 cm-wide obstacle whose near edge is 40 cm in front of the CAMERA.
            val mask = ByteArray(256 * 256) { i ->
                val y = i / 256; val x = i % 256
                val d = proj.groundDistanceCmAtRow(y, 256)
                val lat = proj.lateralCmAt(y, x + 0.5f, 256, 256) ?: 0f
                if (d != null && d in 40f..60f && lat in -10f..10f) 1 else 0
            }
            return ObstaclePlanner(cfg, proj).analyze(mask, 256, 256).nearestObstacleCm!!
        }
        val atCentre = nearest(0f)
        val ahead = nearest(10f)
        assertEquals(40f, atCentre, 3f)
        assertEquals(10f, ahead - atCentre, 2f)
    }

    private fun motion(c: MotorCommand, pwm: Int) =
        NavigationDecision(c, pwm, "test", PathType.STRAIGHT, BoundaryState.NOMINAL)

    private fun driveToFirstLeftPulse(s: ActivePathDiscovery) {
        s.update(blocked(), stopped(), 0L, true)
        s.update(blocked(), stopped(), 100L, true)
        s.update(blocked(), stopped(), 200L, true)
        val firstTurn = s.update(blocked(), stopped(), 500L, true, RotationFeedback(0f, 0f))
        assertEquals(MotorCommand.PIVOT_LEFT, firstTurn.decision.command)
    }

    private fun blocked() = PerceptionSnapshot(
        freeSpaceValid = true, freeSpaceAgeMs = 0L,
        obstaclePlan = ObstaclePlan(
            true, 1, 42f, false, false, false, null, 0f, 19f, 17.5f, 37.5f, 10f, 30f,
            BoundaryState.BLOCKED, PathType.STOP
        )
    )

    private fun stopped() = NavigationDecision(
        MotorCommand.STOP, 0, "no feasible path within expanded boundary", PathType.STOP, BoundaryState.BLOCKED
    )
}

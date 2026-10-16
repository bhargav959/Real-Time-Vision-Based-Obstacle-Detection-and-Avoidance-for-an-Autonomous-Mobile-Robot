package com.example.robotcar.video

import com.example.robotcar.navigation.CameraGroundProjector
import com.example.robotcar.navigation.MotorCommand
import com.example.robotcar.navigation.MotorCalibration
import com.example.robotcar.navigation.NavigationGeometryConfig
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class VirtualScenarioProviderTest {
    @Test
    fun virtualRobotMovesFromDifferentialWheelCommandsRatherThanFollowingAnIdealPath() {
        val config = NavigationGeometryConfig(gapClearanceMarginCm = 2f)
        val provider = VirtualScenarioProvider(config, CameraGroundProjector(config))
        provider.scenarioForFrame(0) // empty, bounded world
        val initial = provider.currentPoseSnapshot()

        repeat(20) { frameOffset ->
            assertFalse(
                "straight open-world steering should not collide",
                provider.wouldCollideOnMotorStep(frameOffset, MotorCommand.SLIGHT_LEFT, 80, 0.1f)
            )
            assertTrue(provider.advance(frameOffset, MotorCommand.SLIGHT_LEFT, 80, 0.1f))
        }

        val finalPose = provider.currentPoseSnapshot()
        assertTrue("slight-left must alter heading", finalPose.headingDeg < initial.headingDeg - 5f)
        assertTrue("slight-left must still translate forward", finalPose.forwardCm > initial.forwardCm + 5f)
        assertTrue("left steering should move the camera toward the left", finalPose.lateralCm < initial.lateralCm)
    }


    @Test
    fun scanPivotUsesOneControlTickAndRotatesWithoutTranslation() {
        val config = NavigationGeometryConfig(gapClearanceMarginCm = 2f)
        val provider = VirtualScenarioProvider(config, CameraGroundProjector(config))
        provider.scenarioForFrame(0)
        val start = provider.currentPoseSnapshot()

        assertTrue(
            provider.rotateForScan(
                0, MotorCommand.PIVOT_LEFT, MotorCalibration.SCAN_MAX_PWM,
                MotorCalibration.SCAN_PULSE_DURATION_MS / 1_000f
            )
        )
        val turned = provider.currentPoseSnapshot()
        assertTrue("low-speed left scan step changes heading", turned.headingDeg < start.headingDeg - MotorCalibration.SCAN_MIN_EFFECTIVE_YAW_DEG)
        assertEquals("in-place pivot has no forward translation", start.forwardCm, turned.forwardCm, 0.01f)
        assertEquals("in-place pivot has no lateral translation", start.lateralCm, turned.lateralCm, 0.01f)
        assertEquals(100L, MotorCalibration.SCAN_PULSE_DURATION_MS)
    }

    @Test
    fun consecutiveScanStepsDoNotReapplyMotorStopOrSpinUp() {
        val config = NavigationGeometryConfig(gapClearanceMarginCm = 2f)
        // Spin-up lag is only modelled when realism is enabled (IDEAL motors have no momentum effect).
        val provider = VirtualScenarioProvider(
            config, CameraGroundProjector(config), realism = SimRealism(motorLagTauMs = 60f)
        )
        provider.scenarioForFrame(0)
        val start = provider.currentPoseSnapshot()
        assertTrue(provider.rotateForScan(0, MotorCommand.PIVOT_LEFT, MotorCalibration.SCAN_MAX_PWM, 0.1f))
        val afterFirst = provider.currentPoseSnapshot()
        assertTrue(provider.rotateForScan(1, MotorCommand.PIVOT_LEFT, MotorCalibration.SCAN_MAX_PWM, 0.1f))
        val afterSecond = provider.currentPoseSnapshot()
        val firstStepYaw = start.headingDeg - afterFirst.headingDeg
        val secondStepYaw = afterFirst.headingDeg - afterSecond.headingDeg
        assertTrue("continuous scan step should retain motor momentum between steps", secondStepYaw > firstStepYaw)
        assertEquals(start.forwardCm, afterSecond.forwardCm, 0.01f)
        assertEquals(start.lateralCm, afterSecond.lateralCm, 0.01f)
    }

    @Test
    fun virtualUltrasonicSensorUsesFrontSensorOffsetAndNoEchoOnClearScene() {
        val config = NavigationGeometryConfig(gapClearanceMarginCm = 2f)
        val provider = VirtualScenarioProvider(config, CameraGroundProjector(config))
        provider.scenarioForFrame(0)
        assertNull("empty scenario should report NO_ECHO", provider.forwardUltrasonicDistanceCm(0))

        provider.scenarioForFrame(60) // obstacle starts at 68 cm; sensor is at the front edge
        val reading = provider.forwardUltrasonicDistanceCm(60)
        assertNotNull(reading)
        assertTrue("distance should be measured from the front sensor, not robot centre", reading!! in 50f..60f)
    }

    @Test
    fun reverseDistanceIsDerivedFromTheTimedPwmPulseRatherThanAForcedFiveCentimetres() {
        val config = NavigationGeometryConfig(gapClearanceMarginCm = 2f)
        val provider = VirtualScenarioProvider(
            config,
            CameraGroundProjector(config),
            wheelTrackWidthCm = MotorCalibration.WHEEL_TRACK_WIDTH_CM,
            nominalWheelSpeedCmPerSecAtPwm150 = MotorCalibration.NOMINAL_SPEED_CM_S_AT_PWM_150
        )
        // Wheel PWM is clamped to the firmware cap (ARDUINO_MAX_PWM). The 30 PWM backoff pulse models
        // 30/150 * 18 = 3.6 cm/s, so 250 ms = 0.9 cm. This explicitly uncalibrated default is a placeholder.
        assertEquals(0.75f, provider.modeledBackoffDistanceCm(MotorCalibration.BACKOFF_PWM, 250L), 0.001f)
        // A request above the cap models the capped speed, never a faster one.
        assertEquals(
            provider.modeledBackoffDistanceCm(MotorCalibration.ARDUINO_MAX_PWM, 250L),
            provider.modeledBackoffDistanceCm(150, 250L), 0.001f
        )
    }

    @Test
    fun safetyMarginIsIncludedInVirtualCollisionGeometry() {
        val config = NavigationGeometryConfig(gapClearanceMarginCm = 2f)
        assertEquals(19f, config.requiredGapWidthCm, 0.001f)
        assertEquals(19f, config.effectiveRobotWidthCm, 0.001f)
    }
}

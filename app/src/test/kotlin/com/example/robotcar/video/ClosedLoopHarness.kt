package com.example.robotcar.video

import com.example.robotcar.navigation.ActivePathDiscovery
import com.example.robotcar.navigation.CameraGroundProjector
import com.example.robotcar.navigation.LiveDecisionStabilizer
import com.example.robotcar.navigation.MotorCalibration
import com.example.robotcar.navigation.MotorCommand
import com.example.robotcar.navigation.NavigationController
import com.example.robotcar.navigation.NavigationGeometryConfig
import com.example.robotcar.navigation.ObstaclePlanner
import com.example.robotcar.navigation.ObstacleTracker
import com.example.robotcar.navigation.PerceptionSnapshot
import com.example.robotcar.navigation.RotationFeedback
import com.example.robotcar.navigation.SafetyConstants
import com.example.robotcar.navigation.SpeedController

/**
 * Headless re-implementation of MainActivity.processVirtualVideoFrame for one scenario segment:
 * the same tracker -> planner -> controller -> scan state machine -> stabilizer -> motor model chain.
 * Keep in step with MainActivity when that pipeline changes. Used for Monte-Carlo robustness runs.
 */
class ClosedLoopHarness(
    realism: SimRealism,
    private val config: NavigationGeometryConfig =
        NavigationGeometryConfig(
            robotWidthCm = 15f, robotLengthCm = 25.5f, nominalBoundaryMarginCm = 10f,
            temporaryBoundaryExtensionCm = 20f, cameraHeightCm = 10f, cameraTiltDeg = 7.5f,
            targetLookaheadCm = 75f, gapClearanceMarginCm = 2f
        ),
    private val useGyroFeedback: Boolean = true,
    nominalSpeedCmS: Float = MotorCalibration.NOMINAL_SPEED_CM_S_AT_PWM_150
) {
    data class Result(
        val segment: Int,
        val collisions: Int,
        val forwardCm: Float,
        val moved: Boolean,
        val minSonarCm: Float,
        val scanStates: Set<String>
    )

    private val projector = CameraGroundProjector(config)
    private val provider = VirtualScenarioProvider(config, projector, nominalWheelSpeedCmPerSecAtPwm150 = nominalSpeedCmS, realism = realism)
    private val planner = ObstaclePlanner(config, projector)
    private val tracker = ObstacleTracker()
    private val controller = NavigationController(speedController = SpeedController())
    private val stabilizer = LiveDecisionStabilizer()
    private val discovery = ActivePathDiscovery()
    private val latencyMs = provider.perception.latencyMs

    fun runSegment(segment: Int, frames: Int = 60): Result {
        provider.reset(); planner.reset(); tracker.reset(); stabilizer.reset(); discovery.reset(); controller.reset()
        var collisions = 0
        var moved = false
        var minSonar = Float.MAX_VALUE
        val scanStates = HashSet<String>()
        var backoffStartedAt = Long.MIN_VALUE
        var prevState = ActivePathDiscovery.State.IDLE.label
        var lastHeading = 0f
        val minPivotClear = SafetyConstants.APP_PIVOT_CLEARANCE_CM
        val minScanSonar = SafetyConstants.APP_TURN_MIN_DISTANCE_CM

        for (k in 0 until frames) {
            val frameIndex = segment * 60 + k
            val nowMs = k * 100L
            val scenario = provider.scenarioForFrame(frameIndex)
            val seen = provider.perception.process(scenario.mask, VirtualScenarioProvider.WIDTH, VirtualScenarioProvider.HEIGHT)
            val mask = tracker.update(seen, VirtualScenarioProvider.WIDTH, VirtualScenarioProvider.HEIGHT, nowMs)
            val plan = planner.analyze(mask, VirtualScenarioProvider.WIDTH, VirtualScenarioProvider.HEIGHT)
            val sonar = provider.forwardUltrasonicDistanceCm(frameIndex)
            if (sonar != null) minSonar = minOf(minSonar, sonar)
            val snapshot = PerceptionSnapshot(
                freeSpaceValid = true, obstaclePlan = plan,
                ultrasonicCm = sonar, ultrasonicNoEcho = sonar == null, ultrasonicAgeMs = 0L,
                ultrasonicRequired = true, cameraAgeMs = 0L, freeSpaceAgeMs = latencyMs
            )
            val raw = controller.decide(snapshot, nowMs)
            val nearest = plan.nearestObstacleCm
            val sonarAllowsScan = if (snapshot.ultrasonicNoEcho) {
                nearest?.let { it >= minPivotClear + 3f } == true
            } else sonar?.let { it >= minScanSonar } == true
            val heading = provider.currentPoseSnapshot().headingDeg
            val rotation = if (useGyroFeedback) RotationFeedback(-heading, -(heading - lastHeading) / 0.1f) else null
            lastHeading = heading
            val scan = discovery.update(
                snapshot, raw, nowMs,
                provider.canRotateInPlace(frameIndex) && nearest?.let { it >= minPivotClear } == true && sonarAllowsScan,
                rotation
            )
            scanStates += scan.state.label
            var decision = if (scan.ownsOutput) { stabilizer.reset(); scan.decision } else stabilizer.apply(scan.decision)

            if (scan.state == ActivePathDiscovery.State.BACKOFF && prevState != ActivePathDiscovery.State.BACKOFF.label) {
                backoffStartedAt = nowMs; provider.finishBackoffProgress()
            }
            if (scan.state == ActivePathDiscovery.State.BACKOFF || prevState == ActivePathDiscovery.State.BACKOFF.label) {
                val elapsed = if (backoffStartedAt == Long.MIN_VALUE) 0L else (nowMs - backoffStartedAt).coerceAtLeast(0L)
                val done = prevState == ActivePathDiscovery.State.BACKOFF.label && scan.state !in setOf(
                    ActivePathDiscovery.State.UNSAFE, ActivePathDiscovery.State.SAFETY_HOLD, ActivePathDiscovery.State.NO_ROUTE)
                val fraction = if (done) 1f else (elapsed / 250f).coerceIn(0f, 1f)
                if (!provider.backOffToDistance(frameIndex, provider.modeledBackoffDistanceCm(), fraction)) {
                    collisions++; discovery.forceSafetyStop("rear obstructed")
                }
                if (scan.state != ActivePathDiscovery.State.BACKOFF) { provider.finishBackoffProgress(); backoffStartedAt = Long.MIN_VALUE }
            }
            prevState = scan.state.label

            if (scan.isRotationPulse && !provider.canRotateInPlace(frameIndex)) {
                collisions++; decision = stabilizer.forceStop("pivot footprint intersects")
            } else if (!scan.ownsOutput && provider.wouldCollideOnMotorStep(
                frameIndex, decision.command, decision.speedPwm, 0.1f,
                noEcho = snapshot.ultrasonicNoEcho
            )) {
                collisions++; decision = stabilizer.forceStop("swept footprint intersects")
            }

            if (scan.isRotationPulse && decision.command != MotorCommand.STOP) {
                provider.rotateForScan(frameIndex, decision.command, decision.speedPwm, MotorCalibration.SCAN_PULSE_DURATION_MS / 1000f)
            } else if (scan.ownsOutput) {
                if (decision.command == MotorCommand.STOP) provider.stopMotorSimulation()
            } else if (decision.command == MotorCommand.STOP) {
                provider.stopMotorSimulation()
            } else {
                if (decision.command != MotorCommand.STOP) moved = true
                if (!provider.advance(
                    frameIndex, decision.command, decision.speedPwm, 0.1f,
                    noEcho = snapshot.ultrasonicNoEcho
                )) collisions++
            }
        }
        return Result(segment, collisions, provider.currentPoseSnapshot().forwardCm, moved, minSonar, scanStates)
    }
}

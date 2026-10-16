package com.example.robotcar.video

import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import android.graphics.PointF
import com.example.robotcar.navigation.BoundaryState
import com.example.robotcar.navigation.CameraGroundProjector
import com.example.robotcar.navigation.MotorCommand
import com.example.robotcar.navigation.MotorCalibration
import com.example.robotcar.navigation.NavigationDecision
import com.example.robotcar.navigation.NavigationGeometryConfig
import com.example.robotcar.navigation.ObstaclePlan
import com.example.robotcar.navigation.PathType
import com.example.robotcar.navigation.SafetyConstants
import com.example.robotcar.navigation.TrajectoryProfile
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.floor
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin

/**
 * Deterministic obstacle worlds for the virtual harness. The semantic mask and simulated sonar
 * come from a shared world map, but chassis pose advances from discrete motor commands using an
 * approximate differential-drive/PWM model. It deliberately does not follow a planner curve as if
 * perfect steering/odometry existed. The model is only as accurate as its chassis calibration.
 */
class VirtualScenarioProvider(
    private val config: NavigationGeometryConfig = NavigationGeometryConfig(),
    private val projector: CameraGroundProjector = CameraGroundProjector(config),
    /** Approximate axle track; measure between left/right wheel contact centres on the real chassis. */
    private val wheelTrackWidthCm: Float = MotorCalibration.WHEEL_TRACK_WIDTH_CM,
    /** Placeholder speed at PWM 150; replace with measured loaded straight-line speed. */
    private val nominalWheelSpeedCmPerSecAtPwm150: Float = MotorCalibration.NOMINAL_SPEED_CM_S_AT_PWM_150,
    /** Optional imperfections (latency, mask noise, motor deadband/lag/mismatch, sonar dropouts). IDEAL = old behaviour. */
    private val realism: SimRealism = SimRealism.IDEAL
) {
    companion object {
        const val WIDTH = 256
        const val HEIGHT = 256
        const val RENDER_WIDTH = 1280
        const val RENDER_HEIGHT = 720
        /** MP4 clock is sampled at 10 FPS. 60 frames means 6 seconds per scenario. */
        private const val SEGMENT_FRAMES = 60
        // Initial model assumptions only—not measured values. See docs/SIM_REALITY_CALIBRATION_v0.5.10.md.
        private const val ARDUINO_MAX_PWM = MotorCalibration.ARDUINO_MAX_PWM
        private const val MIN_MOTOR_PWM = MotorCalibration.MIN_MOTOR_PWM
        private const val SLIGHT_TURN_OUTER_MIN_PWM = MotorCalibration.SLIGHT_TURN_OUTER_MIN_PWM
        private const val KICK_PWM = MotorCalibration.STARTUP_KICK_PWM
        private const val KICK_MS = MotorCalibration.STARTUP_KICK_MS
        private const val KICK_ONLY_BELOW_PWM = MotorCalibration.STARTUP_KICK_THRESHOLD_PWM
        private const val COLLISION_SAMPLE_SPACING_CM = 0.45f
        private const val SCAN_ROTATION_CLEARANCE_CM = 2f
        private const val SONAR_MAX_RANGE_CM = 400f
        private const val SONAR_HALF_BEAM_DEG = 7.5f
        // A small deterministic range perturbation approximates quantisation/repeatability error.
        private const val CONTACT_TOLERANCE_CM = 0.12f
    }

    data class Pose(val lateralCm: Float, val forwardCm: Float, val headingDeg: Float)

    data class Scenario(
        val segment: Int,
        val title: String,
        val mask: ByteArray,
        val pose: Pose
    )

    private data class Box(
        val leftCm: Float,
        val rightCm: Float,
        val nearCm: Float,
        val farCm: Float
    )

    private data class PointWorld(val x: Float, val z: Float)

    private var activeSegment = -1
    private var lateralCm = 0f
    private var forwardCm = 0f
    private var headingDeg = 0f
    private var virtualMotorsRunning = false
    private var backoffOrigin: Pose? = null
    private var backoffAppliedDistanceCm = 0f

    private data class WheelSegment(
        val leftPwm: Int,
        val rightPwm: Int,
        val durationSeconds: Float,
        /** Mean fraction of nominal speed achieved (motor spin-up from rest). */
        val efficiency: Float = 1f
    )

    /** Perception-side imperfections (latency queue, mask corruption); the harness routes masks through it. */
    val perception = SimPerceptionModel(realism)
    private val sonarRng = java.util.Random(realism.seed * 31L + 7L)

    @Synchronized
    fun reset() {
        activeSegment = -1
        lateralCm = 0f
        forwardCm = 0f
        headingDeg = 0f
        virtualMotorsRunning = false
        backoffOrigin = null
        backoffAppliedDistanceCm = 0f
        perception.reset()
    }

    @Synchronized
    fun scenarioForFrame(frameIndex: Int): Scenario {
        val segment = (frameIndex / SEGMENT_FRAMES).coerceIn(0, 8)
        ensureSegment(segment)
        val pose = currentPose()
        val boxes = boxesForSegment(segment)
        return Scenario(
            segment = segment,
            title = titleForSegment(segment),
            mask = maskForPose(boxes, pose),
            pose = pose
        )
    }

    /**
     * Predict collision using the same discrete differential-drive commands sent to the Arduino.
     * This is intentionally not an ideal-path follower: it models motor PWM, the Arduino's
     * minimum PWM floor, its slight-turn inner/outer ratio, and its short startup kick.
     */
    @Synchronized
    fun wouldCollideOnMotorStep(
        frameIndex: Int,
        command: MotorCommand,
        speedPwm: Int,
        deltaSeconds: Float = 0.1f,
        noEcho: Boolean = false
    ): Boolean {
        val segment = (frameIndex / SEGMENT_FRAMES).coerceIn(0, 8)
        ensureSegment(segment)
        if (command == MotorCommand.STOP || command == MotorCommand.BACKOFF || speedPwm <= 0 || deltaSeconds <= 0f) return false
        var pose = currentPose()
        val boxes = boxesForSegment(segment)
        for (step in motionSegments(command, speedPwm, deltaSeconds, virtualMotorsRunning, noEcho)) {
            val end = poseAfterWheelPwm(pose, step.leftPwm, step.rightPwm, step.durationSeconds, step.efficiency)
            val maxTravel = max(
                abs(pwmToSpeed(step.leftPwm)), abs(pwmToSpeed(step.rightPwm, true))
            ) * step.durationSeconds
            val yawTravel = abs(Math.toRadians(headingDeltaDeg(end.headingDeg, pose.headingDeg).toDouble()).toFloat())
            val samples = max(1, maxOf(
                kotlin.math.ceil(maxTravel / COLLISION_SAMPLE_SPACING_CM).toInt(),
                kotlin.math.ceil(yawTravel / Math.toRadians(2.0)).toInt()
            ))
            for (i in 1..samples) {
                val fraction = i.toFloat() / samples
                val candidate = poseAfterWheelPwm(
                    pose,
                    step.leftPwm,
                    step.rightPwm,
                    step.durationSeconds * fraction,
                    step.efficiency
                )
                if (boxes.any { intersectsRobotFootprint(candidate, it) }) return true
            }
            pose = end
        }
        return false
    }

    /** Advance the virtual robot from motor commands, not by snapping it onto the selected route. */
    @Synchronized
    fun advance(
        frameIndex: Int,
        command: MotorCommand,
        speedPwm: Int,
        deltaSeconds: Float = 0.1f,
        noEcho: Boolean = false
    ): Boolean {
        val segment = (frameIndex / SEGMENT_FRAMES).coerceIn(0, 8)
        ensureSegment(segment)
        if (command == MotorCommand.STOP || speedPwm <= 0 || deltaSeconds <= 0f) {
            stopMotorSimulation()
            return true
        }
        if (command == MotorCommand.BACKOFF) return true // a separate timed, collision-checked operation
        var pose = currentPose()
        var lastSafe = pose
        val boxes = boxesForSegment(segment)
        for (step in motionSegments(command, speedPwm, deltaSeconds, virtualMotorsRunning, noEcho)) {
            val maxTravel = max(abs(pwmToSpeed(step.leftPwm)), abs(pwmToSpeed(step.rightPwm, true))) * step.durationSeconds
            val fullEnd = poseAfterWheelPwm(pose, step.leftPwm, step.rightPwm, step.durationSeconds, step.efficiency)
            val yawTravel = abs(Math.toRadians(headingDeltaDeg(fullEnd.headingDeg, pose.headingDeg).toDouble()).toFloat())
            val samples = max(1, maxOf(
                kotlin.math.ceil(maxTravel / COLLISION_SAMPLE_SPACING_CM).toInt(),
                kotlin.math.ceil(yawTravel / Math.toRadians(2.0)).toInt()
            ))
            for (i in 1..samples) {
                val candidate = poseAfterWheelPwm(
                    pose, step.leftPwm, step.rightPwm, step.durationSeconds * (i.toFloat() / samples), step.efficiency
                )
                if (boxes.any { intersectsRobotFootprint(candidate, it) }) {
                    lateralCm = lastSafe.lateralCm
                    forwardCm = lastSafe.forwardCm
                    headingDeg = lastSafe.headingDeg
                    stopMotorSimulation()
                    return false
                }
                lastSafe = candidate
            }
            pose = fullEnd
        }
        lateralCm = pose.lateralCm
        forwardCm = pose.forwardCm
        headingDeg = pose.headingDeg
        virtualMotorsRunning = true
        return true
    }

    @Synchronized
    fun stopMotorSimulation() {
        virtualMotorsRunning = false
    }

    private fun motionSegments(
        command: MotorCommand,
        requestedPwm: Int,
        deltaSeconds: Float,
        wasRunning: Boolean,
        noEcho: Boolean = false
    ): List<WheelSegment> {
        val pwm = requestedPwm.coerceIn(0, ARDUINO_MAX_PWM)
        val pair = wheelPwmPair(command, pwm, noEcho) ?: return emptyList()
        val top = max(abs(pair.first), abs(pair.second))
        if (!wasRunning && top in 1 until KICK_ONLY_BELOW_PWM) {
            val kickSeconds = minOf(deltaSeconds, KICK_MS / 1_000f)
            val kickLeft = if (pair.first == 0) 0 else if (pair.first < 0) -KICK_PWM else KICK_PWM
            val kickRight = if (pair.second == 0) 0 else if (pair.second < 0) -KICK_PWM else KICK_PWM
            val kickEff = if (wasRunning) 1f else spinUpEfficiency(kickSeconds * 1000f, realism.motorLagTauMs)
            val result = mutableListOf(WheelSegment(kickLeft, kickRight, kickSeconds, kickEff))
            val remainder = deltaSeconds - kickSeconds
            // The wheels are already spinning after the kick; only the first segment pays the spin-up.
            if (remainder > 0f) result += WheelSegment(pair.first, pair.second, remainder)
            return result
        }
        val eff = if (wasRunning) 1f else spinUpEfficiency(deltaSeconds * 1000f, realism.motorLagTauMs)
        return listOf(WheelSegment(pair.first, pair.second, deltaSeconds, eff))
    }

    private fun wheelPwmPair(command: MotorCommand, speed: Int, noEcho: Boolean = false): Pair<Int, Int>? {
        if (speed <= 0) return 0 to 0
        fun floorPwm(pwm: Int): Int = if (pwm <= 0) 0 else max(MIN_MOTOR_PWM, pwm).coerceAtMost(ARDUINO_MAX_PWM)
        fun outerTurnPwm(requested: Int): Int {
            // Match Arduino's NO_ECHO safety override: its 30 PWM cap takes precedence over
            // the normal 40 PWM slight-turn outer-wheel floor.
            val outerCap = if (noEcho) SafetyConstants.NO_ECHO_MAX_PWM else ARDUINO_MAX_PWM
            val outerFloor = if (noEcho) MIN_MOTOR_PWM else SLIGHT_TURN_OUTER_MIN_PWM
            return max(requested, outerFloor).coerceAtMost(outerCap)
        }
        val s = floorPwm(speed)
        return when (command) {
            MotorCommand.FORWARD -> s to s
            MotorCommand.REVERSE -> -s to -s
            MotorCommand.LEFT, MotorCommand.PIVOT_LEFT -> -s to s
            MotorCommand.RIGHT, MotorCommand.PIVOT_RIGHT -> s to -s
            MotorCommand.SLIGHT_LEFT -> {
                val outer = outerTurnPwm(speed)
                floorPwm(outer * 45 / 100) to floorPwm(outer)
            }
            MotorCommand.SLIGHT_RIGHT -> {
                val outer = outerTurnPwm(speed)
                floorPwm(outer) to floorPwm(outer * 45 / 100)
            }
            else -> null
        }
    }

    /**
     * Loaded wheel speed for a signed PWM. IDEAL: linear through zero. With a motor deadband the
     * wheel does not turn below [SimRealism.motorDeadbandPwm] and speed rises linearly to the
     * PWM-150 nominal. [right] applies the right-wheel gain (motor mismatch).
     */
    private fun pwmToSpeed(signedPwm: Int, right: Boolean = false): Float {
        val magnitude = abs(signedPwm).coerceAtMost(ARDUINO_MAX_PWM)
        val base = nominalWheelSpeedCmPerSecAtPwm150.coerceAtLeast(0f)
        val dead = realism.motorDeadbandPwm
        var speed = if (dead > 0) {
            if (magnitude <= dead) 0f else base * (magnitude - dead) / (150f - dead)
        } else base * magnitude / 150f
        if (right) speed *= realism.rightWheelGain
        return if (signedPwm < 0) -speed else speed
    }

    /** Predicted reverse displacement for the exact PWM-duration pulse sent to firmware. */
    @Synchronized
    fun modeledBackoffDistanceCm(
        speedPwm: Int = MotorCalibration.BACKOFF_PWM,
        durationMs: Long = MotorCalibration.BACKOFF_DURATION_MS
    ): Float =
        abs(pwmToSpeed(speedPwm)) * durationMs.coerceAtLeast(0L) / 1_000f

    private fun wrapHeadingDeg(angleDeg: Float): Float {
        val wrapped = (angleDeg + 180f) % 360f
        return (if (wrapped < 0f) wrapped + 360f else wrapped) - 180f
    }

    private fun headingDeltaDeg(toDeg: Float, fromDeg: Float): Float =
        wrapHeadingDeg(toDeg - fromDeg)

    private fun poseAfterWheelPwm(origin: Pose, leftPwm: Int, rightPwm: Int, dt: Float, efficiency: Float = 1f): Pose {
        val leftSpeed = pwmToSpeed(leftPwm) * efficiency
        val rightSpeed = pwmToSpeed(rightPwm, true) * efficiency
        val linear = (leftSpeed + rightSpeed) * 0.5f
        // Positive heading in this world coordinate system turns toward the robot's right.
        var omega = (leftSpeed - rightSpeed) / wheelTrackWidthCm.coerceAtLeast(1f)
        if (leftSpeed * rightSpeed < 0f) omega *= (1f - realism.pivotSlipLoss) // skid-steer scrub
        val theta = Math.toRadians(origin.headingDeg.toDouble()).toFloat()
        val theta2 = theta + omega * dt
        val dx: Float
        val dz: Float
        if (abs(omega) < 1e-4f) {
            dx = linear * kotlin.math.sin(theta) * dt
            dz = linear * kotlin.math.cos(theta) * dt
        } else {
            dx = linear / omega * (kotlin.math.cos(theta) - kotlin.math.cos(theta2))
            dz = linear / omega * (kotlin.math.sin(theta2) - kotlin.math.sin(theta))
        }
        return Pose(
            origin.lateralCm + dx,
            origin.forwardCm + dz,
            wrapHeadingDeg(Math.toDegrees(theta2.toDouble()).toFloat())
        )
    }

    /**
     * Apply a progressive reverse distance target derived from the same PWM/time pulse as firmware.
     * The distance is not forced to 5 cm: it depends on calibrated loaded speed. Each incremental
     * swept footprint is checked against the scenario world.
     */
    @Synchronized
    fun backOffToDistance(frameIndex: Int, distanceCm: Float, progressFraction: Float): Boolean {
        val segment = (frameIndex / SEGMENT_FRAMES).coerceIn(0, 8)
        ensureSegment(segment)
        if (!distanceCm.isFinite() || distanceCm <= 0f || !progressFraction.isFinite()) return false
        if (backoffOrigin == null) {
            backoffOrigin = currentPose()
            backoffAppliedDistanceCm = 0f
        }
        val origin = backoffOrigin ?: return false
        val target = (distanceCm * progressFraction.coerceIn(0f, 1f)).coerceAtLeast(backoffAppliedDistanceCm)
        val steps = kotlin.math.ceil((target - backoffAppliedDistanceCm) / 0.4f).toInt().coerceAtLeast(1)
        val boxes = boxesForSegment(segment)
        var lastSafe = currentPose()
        for (i in 1..steps) {
            val travelled = backoffAppliedDistanceCm + (target - backoffAppliedDistanceCm) * i.toFloat() / steps
            val yaw = Math.toRadians(origin.headingDeg.toDouble()).toFloat()
            val candidate = Pose(
                origin.lateralCm - travelled * sin(yaw),
                origin.forwardCm - travelled * cos(yaw),
                origin.headingDeg
            )
            if (boxes.any { intersectsRobotFootprint(candidate, it) }) {
                lateralCm = lastSafe.lateralCm
                forwardCm = lastSafe.forwardCm
                headingDeg = lastSafe.headingDeg
                stopMotorSimulation()
                return false
            }
            lastSafe = candidate
        }
        lateralCm = origin.lateralCm - target * sin(Math.toRadians(origin.headingDeg.toDouble()).toFloat())
        forwardCm = origin.forwardCm - target * cos(Math.toRadians(origin.headingDeg.toDouble()).toFloat())
        headingDeg = origin.headingDeg
        backoffAppliedDistanceCm = target
        stopMotorSimulation()
        return true
    }

    @Synchronized
    fun finishBackoffProgress() {
        backoffOrigin = null
        backoffAppliedDistanceCm = 0f
    }

    /** Simulate the HC-SR04 as a narrow forward beam, measured from the robot's front edge. */
    @Synchronized
    fun forwardUltrasonicDistanceCm(frameIndex: Int): Float? {
        val segment = (frameIndex / SEGMENT_FRAMES).coerceIn(0, 8)
        ensureSegment(segment)
        val boxes = boxesForSegment(segment)
        val yaw = Math.toRadians(headingDeg.toDouble()).toFloat()
        val sensorForwardOffset = config.sonarForwardOffsetCm
        val originX = lateralCm + sensorForwardOffset * sin(yaw)
        val originZ = forwardCm + sensorForwardOffset * cos(yaw)
        val angles = listOf(-SONAR_HALF_BEAM_DEG, -SONAR_HALF_BEAM_DEG / 2f, 0f, SONAR_HALF_BEAM_DEG / 2f, SONAR_HALF_BEAM_DEG)
        var nearest: Float? = null
        for (offset in angles) {
            val rayAngle = Math.toRadians((headingDeg + offset).toDouble()).toFloat()
            val dx = sin(rayAngle)
            val dz = cos(rayAngle)
            for (box in boxes) {
                val hit = rayBoxDistance(originX, originZ, dx, dz, box)
                if (hit != null && hit <= SONAR_MAX_RANGE_CM && (nearest == null || hit < nearest!!)) nearest = hit
            }
        }
        if (realism.enabled) {
            // Acoustic imperfections: random misses, spurious near echoes, Gaussian range noise.
            if (sonarRng.nextFloat() < realism.sonarSpuriousProb) return 10f + sonarRng.nextFloat() * 15f
            val hit = nearest ?: return null
            if (sonarRng.nextFloat() < realism.sonarDropoutProb) return null
            return (hit + sonarRng.nextGaussian().toFloat() * realism.sonarSigmaCm).coerceAtLeast(0.1f)
        }
        val raw = nearest ?: return null
        val noise = when (frameIndex % 3) { 0 -> 0f; 1 -> 0.5f; else -> -0.5f }
        return (raw + noise).coerceAtLeast(0.1f)
    }

    private fun rayBoxDistance(ox: Float, oz: Float, dx: Float, dz: Float, box: Box): Float? {
        var tMin = 0f
        var tMax = SONAR_MAX_RANGE_CM
        fun clip(origin: Float, direction: Float, low: Float, high: Float): Boolean {
            if (abs(direction) < 1e-6f) return origin in low..high
            val t1 = (low - origin) / direction
            val t2 = (high - origin) / direction
            tMin = max(tMin, min(t1, t2))
            tMax = min(tMax, max(t1, t2))
            return tMax >= tMin
        }
        if (!clip(ox, dx, box.leftCm, box.rightCm)) return null
        if (!clip(oz, dz, box.nearCm, box.farCm)) return null
        return if (tMax >= 0f) max(0f, tMin) else null
    }

    /** Separating-axis test between the rotated effective-width × physical-length body and an obstacle box. */
    private fun intersectsRobotFootprint(pose: Pose, box: Box): Boolean {
        val yaw = Math.toRadians(pose.headingDeg.toDouble()).toFloat()
        val latX = cos(yaw)
        val latZ = -sin(yaw)
        val fwdX = sin(yaw)
        val fwdZ = cos(yaw)
        val dx = (box.leftCm + box.rightCm) * 0.5f - pose.lateralCm
        val dz = (box.nearCm + box.farCm) * 0.5f - pose.forwardCm
        val boxHalfX = (box.rightCm - box.leftCm) * 0.5f
        val boxHalfZ = (box.farCm - box.nearCm) * 0.5f
        val robotHalfW = config.effectiveRobotWidthCm * 0.5f
        // The configured gap margin pads the lateral corridor; the longitudinal body dimension
        // remains the measured physical length, matching ObstaclePlanner.cornersFit().
        val robotHalfL = config.robotLengthCm * 0.5f

        fun separated(distance: Float, combinedRadius: Float) =
            abs(distance) >= combinedRadius - CONTACT_TOLERANCE_CM

        if (separated(dx, robotHalfW * abs(latX) + robotHalfL * abs(fwdX) + boxHalfX)) return false
        if (separated(dz, robotHalfW * abs(latZ) + robotHalfL * abs(fwdZ) + boxHalfZ)) return false
        if (separated(dx * latX + dz * latZ, robotHalfW + boxHalfX * abs(latX) + boxHalfZ * abs(latZ))) return false
        if (separated(dx * fwdX + dz * fwdZ, robotHalfL + boxHalfX * abs(fwdX) + boxHalfZ * abs(fwdZ))) return false
        return true
    }

    @Synchronized
    fun renderFrame(
        scenario: Scenario,
        plan: ObstaclePlan?,
        decision: NavigationDecision,
        frameIndex: Int,
        timeUs: Long
    ): Bitmap {
        val bitmap = Bitmap.createBitmap(RENDER_WIDTH, RENDER_HEIGHT, Bitmap.Config.ARGB_8888)
        val canvas = Canvas(bitmap)
        canvas.drawColor(Color.rgb(224, 227, 224))

        val pose = scenario.pose
        drawGround(canvas)
        drawWorldGrid(canvas, pose)
        drawWorldEdges(canvas, pose)
        drawObstacles(canvas, boxesForSegment(scenario.segment), pose)
        drawLocalBoundaries(canvas, plan)
        drawSelectedTrajectory(canvas, plan, decision, pose)
        drawHud(canvas, scenario, decision, frameIndex, timeUs)
        return bitmap
    }

    private fun ensureSegment(segment: Int) {
        if (activeSegment == segment) return
        activeSegment = segment
        lateralCm = 0f
        forwardCm = 0f
        headingDeg = 0f
        // Each synthetic scene starts from rest; do not carry a virtual motor's kick/run state or
        // a partial reverse transaction across a scenario boundary.
        virtualMotorsRunning = false
        backoffOrigin = null
        backoffAppliedDistanceCm = 0f
    }

    private fun currentPose() = Pose(lateralCm, forwardCm, headingDeg)

    @Synchronized
    fun currentPoseSnapshot(): Pose = currentPose()

    /** Conservative swept-corner clearance check for a pivot-in-place scan. */
    @Synchronized
    fun canRotateInPlace(frameIndex: Int): Boolean {
        val segment = (frameIndex / SEGMENT_FRAMES).coerceIn(0, 8)
        ensureSegment(segment)
        val halfWidth = config.effectiveRobotWidthCm * 0.5f
        val halfLength = config.robotLengthCm * 0.5f
        val sweepRadius = kotlin.math.sqrt(halfWidth * halfWidth + halfLength * halfLength) + SCAN_ROTATION_CLEARANCE_CM
        return boxesForSegment(segment).none { box ->
            val dx = when {
                lateralCm < box.leftCm -> box.leftCm - lateralCm
                lateralCm > box.rightCm -> lateralCm - box.rightCm
                else -> 0f
            }
            val dz = when {
                forwardCm < box.nearCm -> box.nearCm - forwardCm
                forwardCm > box.farCm -> forwardCm - box.farCm
                else -> 0f
            }
            kotlin.math.sqrt(dx * dx + dz * dz) < sweepRadius
        }
    }

    /** Rotate in place by one simulated scan time-step. Returns false if the pivot sweep is unsafe. */
    @Synchronized
    fun rotateForScan(frameIndex: Int, command: MotorCommand, speedPwm: Int, deltaSeconds: Float): Boolean {
        if (command != MotorCommand.PIVOT_LEFT && command != MotorCommand.PIVOT_RIGHT &&
            command != MotorCommand.LEFT && command != MotorCommand.RIGHT) return false
        if (speedPwm <= 0 || deltaSeconds <= 0f || !canRotateInPlace(frameIndex)) return false
        val pair = wheelPwmPair(command, speedPwm) ?: return false
        // A scan command is refreshed every control tick without a STOP between steps. Apply
        // spin-up only on the first step; later steps model already-running motors like hardware.
        val eff = if (virtualMotorsRunning) 1f else spinUpEfficiency(deltaSeconds * 1000f, realism.motorLagTauMs)
        val leftSpeed = pwmToSpeed(pair.first) * eff
        val rightSpeed = pwmToSpeed(pair.second, true) * eff
        var omega = (leftSpeed - rightSpeed) / wheelTrackWidthCm.coerceAtLeast(1f)
        if (leftSpeed * rightSpeed < 0f) omega *= (1f - realism.pivotSlipLoss)
        headingDeg = wrapHeadingDeg(headingDeg + Math.toDegrees((omega * deltaSeconds).toDouble()).toFloat())
        virtualMotorsRunning = true
        return true
    }

    private fun titleForSegment(segment: Int): String = when (segment) {
        0 -> "OPEN STRAIGHT"
        1 -> "CENTER OBSTACLE — GENTLE AVOIDANCE"
        2 -> "12 CM GAP — EXPANDED BOUNDARY"
        3 -> "15 CM GAP — LIVE CLEARANCE SHOULD REJECT"
        4 -> "LEFT OBSTACLE — GENTLE RIGHT CURVE"
        5 -> "RIGHT OBSTACLE — GENTLE LEFT CURVE"
        6 -> "MULTI-OBSTACLE SLALOM"
        7 -> "FULL-VIEW WALL — ACTIVE PATH SCAN"
        else -> "DEAD END — SCAN THEN STOP"
    }

    private fun boxesForSegment(segment: Int): List<Box> = when (segment) {
        0 -> emptyList()
        // The central object is narrower than the robot but still blocks the straight footprint.
        // This leaves a genuinely passable side route with room to make a smooth lateral transition.
        // It is placed far enough ahead for the full 25.5 cm swept body to clear the near edge.
        1 -> listOf(Box(-5f, 5f, 68f, 100f))
        2 -> listOf(Box(-11f, -6f, 60f, 92f), Box(6f, 11f, 60f, 92f))
        3 -> listOf(Box(-12f, -7.5f, 60f, 92f), Box(7.5f, 12f, 60f, 92f))
        // Side-obstacle scenes leave a physically passable corridor after accounting for the
        // robot's rotated front corners, not just its centre point. The old edges at +1/-1 cm
        // created a true swept-footprint collision despite the image-space planner's pass result.
        4 -> listOf(Box(-14f, -2f, 68f, 100f))
        5 -> listOf(Box(2f, 14f, 68f, 100f))
        6 -> listOf(
            Box(-8f, 8f, 60f, 75f),
            Box(-13f, -4f, 75f, 98f),
            Box(4f, 13f, 75f, 98f)
        )
        // A broad near-field wall nearly fills the forward image. Its right edge is discoverable
        // only after a scan turn; the right-side route is still inside the expanded corridor.
        7 -> listOf(Box(-45f, 24f, 42f, 72f))
        else -> listOf(Box(-20f, 20f, 58f, 92f))
    }

    private fun maskForPose(boxes: List<Box>, pose: Pose): ByteArray {
        val mask = ByteArray(WIDTH * HEIGHT)
        if (boxes.isEmpty()) return mask
        val yaw = Math.toRadians(pose.headingDeg.toDouble()).toFloat()
        val c = cos(yaw)
        val s = sin(yaw)
        for (y in 0 until HEIGHT) {
            // Ground distance is camera-relative; world geometry is relative to the pivot centre.
            val forwardLocal = (projector.groundDistanceCmAtRow(y, HEIGHT) ?: continue) + config.cameraForwardOffsetCm
            val cmPerPixel = projector.cmPerPixelAtRow(y, WIDTH, HEIGHT) ?: continue
            val rowBase = y * WIDTH
            for (x in 0 until WIDTH) {
                val lateralLocal = (x + 0.5f - WIDTH * 0.5f) * cmPerPixel
                val worldX = pose.lateralCm + lateralLocal * c + forwardLocal * s
                val worldZ = pose.forwardCm - lateralLocal * s + forwardLocal * c
                if (boxes.any { worldX >= it.leftCm && worldX <= it.rightCm && worldZ >= it.nearCm && worldZ <= it.farCm }) {
                    mask[rowBase + x] = 1
                }
            }
        }
        return mask
    }

    private fun drawGround(canvas: Canvas) {
        val paint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.rgb(239, 240, 237)
            style = Paint.Style.FILL
        }
        // The floor remains under the camera even during a large yaw; world markings and obstacles
        // move with the pose. Heading is normalized, not artificially limited to a narrow scan angle.
        val visibleNear = max(config.nearPlanningDistanceCm, 18f)
        val nearLeft = localPoint(visibleNear, -65f) ?: return
        val nearRight = localPoint(visibleNear, 65f) ?: return
        val farRight = localPoint(120f, 65f) ?: return
        val farLeft = localPoint(120f, -65f) ?: return
        val path = Path().apply {
            moveTo(nearLeft.x, nearLeft.y)
            lineTo(nearRight.x, nearRight.y)
            lineTo(farRight.x, farRight.y)
            lineTo(farLeft.x, farLeft.y)
            close()
        }
        canvas.drawPath(path, paint)
    }

    private fun drawWorldGrid(canvas: Canvas, pose: Pose) {
        val paint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.rgb(204, 208, 204); strokeWidth = 2f; style = Paint.Style.STROKE
        }
        var z = floor((pose.forwardCm + 12f) / 15f) * 15f
        val end = pose.forwardCm + 105f
        while (z <= end) {
            drawWorldLine(canvas, PointWorld(-55f, z), PointWorld(55f, z), pose, paint)
            z += 15f
        }
    }

    private fun drawWorldEdges(canvas: Canvas, pose: Pose) {
        val paint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.rgb(156, 162, 158); strokeWidth = 3f; style = Paint.Style.STROKE
        }
        drawWorldLine(canvas, PointWorld(-55f, pose.forwardCm + 10f), PointWorld(-55f, pose.forwardCm + 110f), pose, paint)
        drawWorldLine(canvas, PointWorld(55f, pose.forwardCm + 10f), PointWorld(55f, pose.forwardCm + 110f), pose, paint)
    }

    private fun drawObstacles(canvas: Canvas, boxes: List<Box>, pose: Pose) {
        val fill = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.rgb(48, 50, 49); style = Paint.Style.FILL }
        val edge = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.rgb(20, 21, 20); strokeWidth = 3f; style = Paint.Style.STROKE }
        boxes.forEach { box ->
            val points = listOf(
                PointWorld(box.leftCm, box.nearCm), PointWorld(box.rightCm, box.nearCm),
                PointWorld(box.rightCm, box.farCm), PointWorld(box.leftCm, box.farCm)
            )
            drawProjectedPolygon(canvas, points, pose, fill, edge)
        }
    }

    private fun drawLocalBoundaries(canvas: Canvas, plan: ObstaclePlan?) {
        val nominalHalf = plan?.nominalBoundaryHalfWidthCm ?: config.nominalBoundaryHalfWidthCm
        val expandedHalf = plan?.expandedBoundaryHalfWidthCm ?: config.expandedBoundaryHalfWidthCm
        val nominal = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.WHITE; strokeWidth = 4f; style = Paint.Style.STROKE
            alpha = 230
        }
        val expanded = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.rgb(255, 193, 7); strokeWidth = 3f; style = Paint.Style.STROKE; alpha = 220
        }
        drawLocalLine(canvas, -nominalHalf, 20f, 90f, nominal)
        drawLocalLine(canvas, nominalHalf, 20f, 90f, nominal)
        drawLocalLine(canvas, -expandedHalf, 20f, 90f, expanded)
        drawLocalLine(canvas, expandedHalf, 20f, 90f, expanded)
    }

    private fun drawSelectedTrajectory(canvas: Canvas, plan: ObstaclePlan?, decision: NavigationDecision, cameraPose: Pose) {
        val paint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = when {
                decision.reason.contains("simulated collision", ignoreCase = true) -> Color.RED
                decision.command == MotorCommand.STOP && plan?.bestPath == null -> Color.RED
                decision.command == MotorCommand.STOP -> Color.rgb(255, 193, 7) // waiting for stable path
                else -> Color.rgb(0, 200, 83)
            }
            strokeWidth = 7f; style = Paint.Style.STROKE; strokeCap = Paint.Cap.ROUND; strokeJoin = Paint.Join.ROUND
        }
        val path = Path()
        var first = true
        val candidate = plan?.bestPath
        if (candidate != null) {
            // Display the planner's candidate, not an idealized curve that the motor model follows.
            val endDistance = candidate.targetDistanceCm
                .coerceAtLeast(config.nearPlanningDistanceCm + 1f)
            val targetLateral = candidate.lateralOffsetCm
            var d = max(config.nearPlanningDistanceCm, 18f)
            while (d <= endDistance + 0.1f) {
                val p = (d / endDistance).coerceIn(0f, 1f)
                val point = localPoint(d, targetLateral * TrajectoryProfile.lateralFraction(p))
                if (point != null) {
                    if (first) { path.moveTo(point.x, point.y); first = false } else path.lineTo(point.x, point.y)
                }
                d += 1.5f
            }
        }
        if (!first) canvas.drawPath(path, paint)
        if (decision.command == MotorCommand.STOP && (plan?.bestPath == null || decision.reason.contains("simulated collision", ignoreCase = true))) {
            val stopPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.RED; textSize = 40f; typeface = android.graphics.Typeface.DEFAULT_BOLD }
            val message = if (decision.reason.contains("simulated collision", ignoreCase = true)) "SIMULATED COLLISION — STOP" else "STOP — NO VERIFIED PATH"
            canvas.drawText(message, 48f, RENDER_HEIGHT * 0.66f, stopPaint)
        }
    }

    private fun drawHud(canvas: Canvas, scenario: Scenario, decision: NavigationDecision, frameIndex: Int, timeUs: Long) {
        val panel = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.argb(205, 18, 25, 28); style = Paint.Style.FILL }
        canvas.drawRect(0f, 0f, RENDER_WIDTH.toFloat(), 105f, panel)
        val title = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.WHITE; textSize = 28f; typeface = android.graphics.Typeface.DEFAULT_BOLD
        }
        val sub = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.rgb(220, 229, 224); textSize = 20f }
        canvas.drawText("VIRTUAL ROBOT POV • SCENE ${scenario.segment + 1}/9", 24f, 32f, title)
        canvas.drawText(scenario.title, 24f, 62f, sub)
        val elapsed = String.format(java.util.Locale.US, "%.1fs", timeUs / 1_000_000.0)
        val commandText = "${decision.command.name} • ${decision.speedPwm} PWM • PATH ${decision.pathAngleDeg.toInt()}° • t=$elapsed • frame=$frameIndex"
        canvas.drawText(commandText, 24f, 91f, sub)
        val poseText = "x=${scenario.pose.lateralCm.toInt()} cm   z=${scenario.pose.forwardCm.toInt()} cm   yaw=${scenario.pose.headingDeg.toInt()}°"
        canvas.drawText(poseText, 24f, RENDER_HEIGHT - 22f, sub)
    }

    private fun drawLocalLine(canvas: Canvas, lateralCm: Float, startDistance: Float, endDistance: Float, paint: Paint) {
        val path = Path()
        var started = false
        var distance = startDistance
        while (distance <= endDistance + 0.1f) {
            val point = localPoint(distance, lateralCm)
            if (point == null) {
                distance += 3f
                continue
            }
            if (!started) { path.moveTo(point.x, point.y); started = true } else path.lineTo(point.x, point.y)
            distance += 3f
        }
        if (started) canvas.drawPath(path, paint)
    }

    private fun drawWorldLine(canvas: Canvas, a: PointWorld, b: PointWorld, pose: Pose, paint: Paint) {
        val path = Path()
        var started = false
        val segments = 36
        for (i in 0..segments) {
            val t = i.toFloat() / segments
            val worldX = a.x + (b.x - a.x) * t
            val worldZ = a.z + (b.z - a.z) * t
            val point = worldPoint(worldX, worldZ, pose) ?: continue
            if (!started) { path.moveTo(point.x, point.y); started = true } else path.lineTo(point.x, point.y)
        }
        if (started) canvas.drawPath(path, paint)
    }

    private fun drawProjectedPolygon(
        canvas: Canvas,
        points: List<PointWorld>,
        pose: Pose,
        fill: Paint,
        stroke: Paint? = null
    ) {
        val projected = ArrayList<PointF>(points.size)
        for (point in points) projected += worldPoint(point.x, point.z, pose) ?: return
        if (projected.size < 3) return
        val path = Path().apply {
            moveTo(projected[0].x, projected[0].y)
            for (i in 1 until projected.size) lineTo(projected[i].x, projected[i].y)
            close()
        }
        canvas.drawPath(path, fill)
        if (stroke != null) canvas.drawPath(path, stroke)
    }

    private fun localPoint(distanceCm: Float, lateralCm: Float): PointF? {
        val row = projector.rowForDistanceCm(distanceCm - config.cameraForwardOffsetCm, RENDER_HEIGHT) ?: return null
        val x = projector.xForLateralCm(row, lateralCm, RENDER_WIDTH, RENDER_HEIGHT) ?: return null
        return PointF(x, row.toFloat())
    }

    private fun worldPoint(worldX: Float, worldZ: Float, pose: Pose): PointF? {
        val dx = worldX - pose.lateralCm
        val dz = worldZ - pose.forwardCm
        val yaw = Math.toRadians(pose.headingDeg.toDouble()).toFloat()
        val lateralLocal = dx * cos(yaw) - dz * sin(yaw)
        val forwardLocal = dx * sin(yaw) + dz * cos(yaw)
        if (forwardLocal < 1f || forwardLocal > 160f) return null
        return localPoint(forwardLocal, lateralLocal)
    }
}

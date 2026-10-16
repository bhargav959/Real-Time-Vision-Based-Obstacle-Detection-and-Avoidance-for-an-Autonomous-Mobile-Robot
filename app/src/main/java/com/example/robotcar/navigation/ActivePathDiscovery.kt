package com.example.robotcar.navigation

/**
 * Bounded active-perception scan used when a real obstacle blocks every verified local path.
 * This hardware profile disables blind reverse: the pictured chassis has no clearly visible rear sensor,
 * and reversing cannot be verified from the front camera. The car must stop rather than back up blindly. After every backoff the controller MUST stop, wait for a camera/free-space
 * sample captured after the reverse ended, scan both sides, and resume only on a fresh path whose
 * command actually steers around a still-blocking obstacle. Sign, stale-data, USB and user stops
 * remain hard stops.
 *
 * The scan holds a low-PWM pivot during each bounded left/right sweep while Android refreshes the same direction every control tick. It does not stop for a camera-settle interval after every pulse; it stops once at the end of each directional sweep for a stable, fresh image. The right sweep traverses back across centre, giving symmetric coverage. This is open-loop steering (no wheel encoders/IMU for wheel position), so clearance is rechecked every control tick and any stale/unsafe evidence stops the scan immediately.
 */
class ActivePathDiscovery(
    private val blockedFramesToStart: Int = 3,
    // MainActivity's live controller and virtual harness tick at 100 ms. One scan step
    // therefore maps to one control interval, matching the keepalive refresh cadence.
    private val turnStepMs: Long = MotorCalibration.SCAN_PULSE_DURATION_MS,
    private val sampleSettleMs: Long = 2L * MotorCalibration.CONTROL_TICK_MS,
    private val maxLeftSteps: Int = 4,
    private val maxRightSteps: Int = 8,
    // Start below normal movement speed; firmware bypasses the normal startup kick for scan pivots.
    private val scanPwm: Int = MotorCalibration.SCAN_START_PWM,
    private val maxScanPwm: Int = MotorCalibration.SCAN_MAX_PWM,
    // User-requested short blind-backoff pulse. There is still no rear-facing range sensor.
    // The firmware bounds this to 250 ms / PWM 25; a fresh post-reverse scan is mandatory.
    private val allowBlindBackoff: Boolean = false,
    // Keep BACKOFF commanded one control tick beyond the firmware pulse. This hold interval is
    // not the motor-on duration; the firmware itself stops the reverse at BACKOFF_DURATION_MS.
    private val backoffCommandHoldMs: Long =
        ((MotorCalibration.BACKOFF_DURATION_MS + MotorCalibration.CONTROL_TICK_MS - 1L) /
            MotorCalibration.CONTROL_TICK_MS) * MotorCalibration.CONTROL_TICK_MS,
    private val backoffPwm: Int = MotorCalibration.BACKOFF_PWM
) {
    enum class State(val label: String) {
        IDLE("IDLE"),
        BACKOFF("BOUNDED REVERSE"),
        WAIT_CLEARANCE("WAIT FOR CLEARANCE"),
        TURN_LEFT("SCAN LEFT"),
        SETTLE_LEFT("SAMPLE LEFT"),
        TURN_RIGHT("SCAN RIGHT"),
        SETTLE_RIGHT("SAMPLE RIGHT"),
        UNSAFE("SCAN DENIED / UNSAFE"),
        NO_ROUTE("SCAN COMPLETE / NO PATH"),
        SAFETY_HOLD("SAFETY STOP")
    }

    data class Output(
        val decision: NavigationDecision,
        val state: State,
        /** True while this module owns the movement/stop output rather than normal path following. */
        val ownsOutput: Boolean,
        /** True only while an intentional low-speed scan step is commanded. */
        val isRotationPulse: Boolean
    )

    private var state = State.IDLE
    private var blockedFrames = 0
    private var stateStartedAtMs = 0L
    private var leftStepsCompleted = 0
    private var rightStepsCompleted = 0

    // --- closed-loop pivot verification (v0.5.11) ---------------------------------------------
    // When gyro feedback is available, each scan step is checked. PWM rises by 5 only when the step
    // produces insufficient yaw, and respects PWM 30 on NO_ECHO / PWM 41 with a valid sonar echo. The camera is sampled
    // once after a complete side sweep rather than after each individual scan step.
    private var currentScanPwm = scanPwm
    private var activeMaxScanPwm = maxScanPwm
    private var stepStartYawDeg: Float? = null
    private var ineffectiveStreak = 0
    private var openLoopStepCount = 0

    /** PWM the next scan step will use (rises only when the gyro shows steps are not rotating). */
    val currentScanPwmValue: Int get() = currentScanPwm

    val stateLabel: String get() = state.label
    val isScanning: Boolean get() = state in setOf(
        State.BACKOFF, State.WAIT_CLEARANCE, State.TURN_LEFT, State.SETTLE_LEFT, State.TURN_RIGHT, State.SETTLE_RIGHT
    )

    fun reset() {
        currentScanPwm = scanPwm
        activeMaxScanPwm = maxScanPwm
        stepStartYawDeg = null
        ineffectiveStreak = 0
        openLoopStepCount = 0
        state = State.IDLE
        blockedFrames = 0
        stateStartedAtMs = 0L
        leftStepsCompleted = 0
        rightStepsCompleted = 0
    }

    fun forceSafetyStop(reason: String): Output {
        state = State.SAFETY_HOLD
        blockedFrames = 0
        return finalStop("active path discovery safety stop: $reason")
    }

    fun update(
        snapshot: PerceptionSnapshot,
        normalDecision: NavigationDecision,
        nowMs: Long,
        scanClearanceVerified: Boolean,
        /** Phone-gyro yaw feedback; null (virtual unit tests, no sensor) keeps the open-loop behaviour. */
        rotation: RotationFeedback? = null
    ): Output {
        // Keep the app's commanded scan ceiling aligned with the Arduino's conservative
        // no-echo cap. A missing ultrasonic echo never raises speed; with a valid echo the
        // global scan cap remains 41 PWM with valid sonar.
        activeMaxScanPwm = if (snapshot.ultrasonicRequired && snapshot.ultrasonicNoEcho) {
            minOf(maxScanPwm, SafetyConstants.NO_ECHO_MAX_PWM)
        } else {
            maxScanPwm
        }
        if (currentScanPwm > activeMaxScanPwm) currentScanPwm = activeMaxScanPwm

        // Hard safety conditions always take precedence, except for a fresh, vision-confirmed
        // obstacle directly ahead: that specific case may request one timed reverse pulse, then
        // waits for safe front/pivot clearance before any turning. Stale data, signs, USB faults,
        // and user STOP still override the entire manoeuvre.
        val ultrasonicStop = normalDecision.reason.contains("ultrasonic <", ignoreCase = true)
        val closeObstacleManeuverAllowed = ultrasonicStop && when (state) {
            // A close-sonar STOP may be overridden only for a scan pivot with independent
            // front-clearance checks, or for the explicitly bounded blind-backoff pulse.
            State.IDLE, State.UNSAFE -> scanClearanceVerified && scanEvidenceFresh(snapshot) ||
                (allowBlindBackoff && backoffStartEvidenceFresh(snapshot))
            State.BACKOFF -> allowBlindBackoff && backoffContinuitySafe(snapshot)
            State.WAIT_CLEARANCE -> scanEvidenceFresh(snapshot)
            State.TURN_LEFT, State.SETTLE_LEFT, State.TURN_RIGHT, State.SETTLE_RIGHT ->
                scanClearanceVerified
            else -> false
        }
        if (isHardSafetyStop(normalDecision.reason) && !closeObstacleManeuverAllowed) {
            if (isScanning || state == State.UNSAFE) state = State.SAFETY_HOLD
            return Output(normalDecision, state, ownsOutput = false, isRotationPulse = false)
        }

        val pathFound = snapshot.freeSpaceValid &&
            snapshot.obstaclePlan?.bestPath?.feasible == true &&
            snapshot.freeSpaceAgeMs <= 600L
        // Never let an old/currently-preliminary path candidate cancel a backoff or an active
        // scan. In particular, a feasible candidate is not proof that the commanded motion avoids
        // the obstacle. Only terminal states may resume, and a blocked centre corridor requires
        // a signed left/right avoidance command from a fresh map.
        if (state in setOf(State.UNSAFE, State.NO_ROUTE, State.SAFETY_HOLD) &&
            pathFound && safePathToResume(snapshot, normalDecision)) {
            reset()
            return Output(normalDecision, State.IDLE, ownsOutput = false, isRotationPulse = false)
        }

        val blocked = isPersistentObstacleBlock(snapshot, normalDecision)
        if (!blocked && state == State.IDLE) {
            blockedFrames = 0
            return Output(normalDecision, state, ownsOutput = false, isRotationPulse = false)
        }

        when (state) {
            State.IDLE -> {
                if (!blocked) {
                    blockedFrames = 0
                    return Output(normalDecision, state, ownsOutput = false, isRotationPulse = false)
                }
                blockedFrames++
                if (blockedFrames < blockedFramesToStart) {
                    return Output(normalDecision, state, ownsOutput = false, isRotationPulse = false)
                }
                if (!scanEvidenceFresh(snapshot)) {
                    state = State.UNSAFE
                    stateStartedAtMs = nowMs
                    return denied(normalDecision, "path blocked; camera/sensor evidence is not fresh")
                }
                leftStepsCompleted = 0
                rightStepsCompleted = 0
                if (scanClearanceVerified) {
                    // If the robot can pivot safely already, scan in place instead of reversing.
                    state = State.TURN_LEFT
                    stateStartedAtMs = nowMs
                    stepStartYawDeg = rotation?.yawDeg
                    return scanTurn(MotorCommand.PIVOT_LEFT, "left sweep 1/$maxLeftSteps; verified pivot")
                }
                if (!allowBlindBackoff || !backoffStartEvidenceFresh(snapshot)) {
                    state = State.UNSAFE
                    stateStartedAtMs = nowMs
                    return denied(normalDecision, "path blocked; reverse conditions are not met and pivot clearance is not verified")
                }
                state = State.BACKOFF
                stateStartedAtMs = nowMs
                return scanBackoff("bounded blind reverse pulse to create turning room; rear area is not sensed")
            }

            State.UNSAFE -> {
                if (!blocked) {
                    if (safePathToResume(snapshot, normalDecision)) {
                        reset()
                        return Output(normalDecision, State.IDLE, ownsOutput = false, isRotationPulse = false)
                    }
                    return finalStop("unsafe recovery state; refusing an unverified path")
                }
                if (!scanEvidenceFresh(snapshot)) {
                    return denied(normalDecision, "path blocked; camera/sensor evidence is not fresh")
                }
                if (scanClearanceVerified) {
                    state = State.TURN_LEFT
                    stateStartedAtMs = nowMs
                    leftStepsCompleted = 0
                    rightStepsCompleted = 0
                    stepStartYawDeg = rotation?.yawDeg
                    return scanTurn(MotorCommand.PIVOT_LEFT, "left sweep 1/$maxLeftSteps; verified pivot")
                }
                if (!allowBlindBackoff || !backoffStartEvidenceFresh(snapshot)) {
                    return denied(normalDecision, "path blocked; reverse conditions are not met and pivot clearance is not verified")
                }
                state = State.BACKOFF
                stateStartedAtMs = nowMs
                leftStepsCompleted = 0
                rightStepsCompleted = 0
                return scanBackoff("bounded blind reverse pulse to create turning room; rear area is not sensed")
            }

            State.BACKOFF -> {
                if (!backoffContinuitySafe(snapshot)) return unsafeStop("camera/sensor evidence became stale during backoff")
                if (nowMs - stateStartedAtMs < backoffCommandHoldMs) {
                    return scanBackoff("bounded blind reverse pulse to create turning room; distance requires chassis calibration")
                }
                // Crucial: never accept a path based on the pre-reverse view. End the reverse,
                // stop, and require a fresh post-reverse image/map before any turn or forward path.
                state = State.WAIT_CLEARANCE
                stateStartedAtMs = nowMs
                return scanWait("reverse complete; stopped for a fresh post-reverse map before selecting an avoidance path")
            }

            State.WAIT_CLEARANCE -> {
                if (!scanEvidenceFresh(snapshot)) return unsafeStop("camera/sensor evidence became stale while waiting after reverse")
                if (!postReverseSampleReady(snapshot, nowMs, rotation)) {
                    return scanWait("waiting for a fresh camera/free-space sample captured after reverse ended")
                }
                if (!scanClearanceVerified) {
                    return scanWait("post-reverse map received; waiting for verified front/pivot clearance")
                }
                state = State.TURN_LEFT
                stateStartedAtMs = nowMs
                leftStepsCompleted = 0
                rightStepsCompleted = 0
                stepStartYawDeg = rotation?.yawDeg
                return scanTurn(MotorCommand.PIVOT_LEFT, "left sweep 1/$maxLeftSteps from post-reverse view")
            }

            State.TURN_LEFT -> {
                if (!scanClearanceVerified) return unsafeStop("turn clearance became uncertain")
                if (nowMs - stateStartedAtMs < turnStepMs) {
                    return scanTurn(MotorCommand.PIVOT_LEFT, "left sweep ${leftStepsCompleted + 1}/$maxLeftSteps")
                }
                stepEffectivenessAbort(rotation, +1f)?.let { return it }
                leftStepsCompleted++
                if (!scanClearanceVerified) return unsafeStop("left sweep clearance is not verified")
                if (leftStepsCompleted < maxLeftSteps) {
                    // Refresh the same-direction scan keepalive on every control tick. Firmware keeps
                    // the pivot motors running between steps and stops if refreshes go stale.
                    stateStartedAtMs = nowMs
                    stepStartYawDeg = rotation?.yawDeg
                    return scanTurn(MotorCommand.PIVOT_LEFT, "left sweep ${leftStepsCompleted + 1}/$maxLeftSteps")
                }
                // Stop only once per sweep, so the camera can provide an unblurred fresh map.
                state = State.SETTLE_LEFT
                stateStartedAtMs = nowMs
                return scanWait("left sweep complete; sampling stable left-side view")
            }

            State.SETTLE_LEFT -> {
                if (!sampleReady(snapshot, nowMs, rotation)) {
                    if (nowMs - stateStartedAtMs > MotorCalibration.SCAN_MAX_SETTLE_MS) {
                        return unsafeStop("no fresh post-sweep camera frame within ${MotorCalibration.SCAN_MAX_SETTLE_MS} ms")
                    }
                    return scanWait("sampling left-side view")
                }
                if (!scanClearanceVerified) return unsafeStop("left/right sweep clearance is not verified")
                state = State.TURN_RIGHT
                stateStartedAtMs = nowMs
                stepStartYawDeg = rotation?.yawDeg
                return scanTurn(MotorCommand.PIVOT_RIGHT, "right sweep 1/$maxRightSteps")
            }

            State.TURN_RIGHT -> {
                if (!scanClearanceVerified) return unsafeStop("turn clearance became uncertain")
                if (nowMs - stateStartedAtMs < turnStepMs) {
                    return scanTurn(MotorCommand.PIVOT_RIGHT, "right sweep ${rightStepsCompleted + 1}/$maxRightSteps")
                }
                stepEffectivenessAbort(rotation, -1f)?.let { return it }
                rightStepsCompleted++
                if (rightStepsCompleted < maxRightSteps) {
                    if (!scanClearanceVerified) return unsafeStop("rightward sweep clearance is not verified")
                    stateStartedAtMs = nowMs
                    stepStartYawDeg = rotation?.yawDeg
                    return scanTurn(MotorCommand.PIVOT_RIGHT, "right sweep ${rightStepsCompleted + 1}/$maxRightSteps")
                }
                state = State.SETTLE_RIGHT
                stateStartedAtMs = nowMs
                return scanWait("right sweep complete; sampling stable right-side view")
            }

            State.SETTLE_RIGHT -> {
                if (!sampleReady(snapshot, nowMs, rotation)) {
                    if (nowMs - stateStartedAtMs > MotorCalibration.SCAN_MAX_SETTLE_MS) {
                        return unsafeStop("no fresh post-sweep camera frame within ${MotorCalibration.SCAN_MAX_SETTLE_MS} ms")
                    }
                    return scanWait("sampling right-side view")
                }
                if (!scanClearanceVerified) return unsafeStop("completed scan view is not safe to act on")
                // This map was captured after the scan. Resume only if the chosen command follows
                // a geometrically feasible route; when the centre is blocked, straight F is never
                // accepted, even if a stale/incorrect planner state claims some path is feasible.
                if (safePathToResume(snapshot, normalDecision)) {
                    reset()
                    return Output(normalDecision, State.IDLE, ownsOutput = false, isRotationPulse = false)
                }
                state = State.NO_ROUTE
                stateStartedAtMs = nowMs
                return finalStop("scan complete; no verified avoidance path; remain stopped")
            }

            State.NO_ROUTE -> {
                if (safePathToResume(snapshot, normalDecision)) {
                    reset()
                    return Output(normalDecision, State.IDLE, ownsOutput = false, isRotationPulse = false)
                }
                return finalStop("scan complete; no verified avoidance path; remain stopped until a new safe route appears")
            }

            State.SAFETY_HOLD -> {
                // Do not automatically resume after a safety fault. A clean new path observation
                // above can reset this state; otherwise remain stopped.
                if (pathFound && safePathToResume(snapshot, normalDecision)) {
                    reset()
                    return Output(normalDecision, State.IDLE, ownsOutput = false, isRotationPulse = false)
                }
                return finalStop("safety stop during active scan")
            }
        }
    }

    /**
     * A post-sweep camera sample is usable only when (a) the settle time elapsed, (b) the chassis has
     * stopped rotating (gyro, if present), and (c) the free-space map was built from a frame CAPTURED
     * after the end-of-sweep STOP plus a guard time. Without (c) the "sample" can be a pre-stop, motion-blurred
     * frame that merely finished inference late. freeSpaceAgeMs counts from capture.
     */
    private fun sampleReady(snapshot: PerceptionSnapshot, nowMs: Long, rotation: RotationFeedback?): Boolean {
        val elapsed = nowMs - stateStartedAtMs
        if (elapsed < sampleSettleMs) return false
        if (rotation != null && !YawTracker.isSettled(rotation.yawRateDegPerSec)) return false
        if (!snapshot.freeSpaceValid) return false
        return snapshot.freeSpaceAgeMs <= elapsed - MotorCalibration.SCAN_FRAME_AFTER_STOP_GUARD_MS
    }

    /**
     * Require a free-space map and camera frame captured at least one guard interval after reverse
     * stopped. A merely recent pre-reverse frame is not enough to choose the next movement.
     */
    private fun postReverseSampleReady(
        snapshot: PerceptionSnapshot,
        nowMs: Long,
        rotation: RotationFeedback?
    ): Boolean {
        val elapsed = nowMs - stateStartedAtMs
        if (elapsed < sampleSettleMs + MotorCalibration.SCAN_FRAME_AFTER_STOP_GUARD_MS) return false
        if (rotation != null && !YawTracker.isSettled(rotation.yawRateDegPerSec)) return false
        val maxAgeForPostReverseCapture = elapsed - MotorCalibration.SCAN_FRAME_AFTER_STOP_GUARD_MS
        if (snapshot.cameraAgeMs > maxAgeForPostReverseCapture ||
            snapshot.freeSpaceAgeMs > maxAgeForPostReverseCapture) return false
        return snapshot.cameraAgeMs <= 350L &&
            snapshot.freeSpaceValid &&
            snapshot.freeSpaceAgeMs <= SafetyConstants.SCAN_EVIDENCE_MAX_AGE_MS
    }

    /**
     * Final guard before handing control back to normal navigation. If an obstacle still blocks
     * the centre corridor, the command must match the signed left/right path; FORWARD is rejected.
     */
    private fun safePathToResume(
        snapshot: PerceptionSnapshot,
        decision: NavigationDecision
    ): Boolean {
        if (decision.command == MotorCommand.STOP ||
            snapshot.cameraAgeMs > 350L ||
            !snapshot.freeSpaceValid ||
            snapshot.freeSpaceAgeMs > SafetyConstants.SCAN_EVIDENCE_MAX_AGE_MS ||
            (snapshot.ultrasonicRequired && snapshot.ultrasonicAgeMs > SafetyConstants.ULTRASONIC_MAX_AGE_MS)) {
            return false
        }
        val plan = snapshot.obstaclePlan ?: return false
        val candidate = plan.bestPath ?: return false
        if (!candidate.feasible) return false

        if (plan.obstacleDetected && !plan.centerPathFeasible) {
            if (decision.command !in setOf(
                    MotorCommand.SLIGHT_LEFT, MotorCommand.SLIGHT_RIGHT,
                    MotorCommand.LEFT, MotorCommand.RIGHT
                )) return false
            val signedPath = when {
                kotlin.math.abs(candidate.headingAngleDeg) >= 0.25f -> candidate.headingAngleDeg
                kotlin.math.abs(candidate.lateralOffsetCm) >= 0.25f -> candidate.lateralOffsetCm
                else -> return false
            }
            val commandGoesLeft = decision.command in setOf(MotorCommand.SLIGHT_LEFT, MotorCommand.LEFT)
            if (commandGoesLeft != (signedPath < 0f)) return false
            if (decision.pathType != PathType.AVOIDANCE_CURVE && plan.pathType != PathType.AVOIDANCE_CURVE) return false
        }
        return true
    }

    /**
     * Gyro check of the scan step that just finished. [direction] +1 = left/CCW expected, -1 = right.
     * Returns a non-null stop Output when the scan must abort; otherwise may raise currentScanPwm.
     */
    private fun stepEffectivenessAbort(rotation: RotationFeedback?, direction: Float): Output? {
        val start = stepStartYawDeg
        stepStartYawDeg = null
        if (rotation == null) {
            // A device without usable gyro feedback cannot verify an individual step. Keep the
            // crawl speed for two pulses, then increase by only 5 PWM so a below-deadband motor
            // can still turn. The bounded global cap and post-sweep camera check remain in force.
            openLoopStepCount++
            if (openLoopStepCount >= 2 && currentScanPwm < activeMaxScanPwm) {
                currentScanPwm = minOf(activeMaxScanPwm, currentScanPwm + 5)
                openLoopStepCount = 0
            }
            return null
        }
        if (start == null) return null
        openLoopStepCount = 0
        val rotated = (rotation.yawDeg - start) * direction
        if (rotated >= MotorCalibration.SCAN_MIN_EFFECTIVE_YAW_DEG) {
            ineffectiveStreak = 0
            return null
        }
        if (currentScanPwm < activeMaxScanPwm) {
            currentScanPwm = minOf(activeMaxScanPwm, currentScanPwm + 5)
            return null // stronger step next time; this pulse still counts toward the bounded total
        }
        ineffectiveStreak++
        return if (ineffectiveStreak >= 2) {
            unsafeStop("scan pivots at PWM $currentScanPwm produce no measured rotation (stalled/low battery?)")
        } else null
    }

    private fun scanEvidenceFresh(snapshot: PerceptionSnapshot): Boolean {
        if (snapshot.cameraAgeMs > 350L || !snapshot.freeSpaceValid ||
            snapshot.freeSpaceAgeMs > SafetyConstants.SCAN_EVIDENCE_MAX_AGE_MS) return false
        if (snapshot.ultrasonicRequired && snapshot.ultrasonicAgeMs > SafetyConstants.ULTRASONIC_MAX_AGE_MS) return false
        val sign = if (snapshot.trafficSignAgeMs <= 1_200L) snapshot.trafficSign else null
        return sign == null || sign.confidence < 0.75f || sign.label.lowercase() !in setOf("stop", "no_entry")
    }

    /** Keep an already-started reverse pulse alive through its firmware timer if evidence is fresh. */
    private fun backoffContinuitySafe(snapshot: PerceptionSnapshot): Boolean = scanEvidenceFresh(snapshot)

    /** Fresh front evidence authorizes only the bounded backoff, never a straight-through move. */
    private fun backoffStartEvidenceFresh(snapshot: PerceptionSnapshot): Boolean {
        if (!allowBlindBackoff || !scanEvidenceFresh(snapshot)) return false
        val plan = snapshot.obstaclePlan ?: return false
        if (!plan.obstacleDetected || plan.centerPathFeasible || plan.nearestObstacleCm == null) return false

        if (snapshot.ultrasonicRequired) {
            if (snapshot.ultrasonicAgeMs > SafetyConstants.ULTRASONIC_MAX_AGE_MS) return false
            val distance = snapshot.ultrasonicCm ?: return false
            if (!distance.isFinite() || distance <= 0f) return false // NO_ECHO is not proof of rearward safety
            // If the planner has found a side route, backoff is considered only when the sonar
            // is keeping forward motion inhibited. Otherwise an open route should be followed.
            if (plan.bestPath != null && distance >= SafetyConstants.ULTRASONIC_STOP_CM) return false
        } else if (plan.bestPath != null) {
            return false
        }

        return true
    }

    private fun isPersistentObstacleBlock(
        snapshot: PerceptionSnapshot,
        decision: NavigationDecision
    ): Boolean {
        val p = snapshot.obstaclePlan ?: return false
        if (decision.command != MotorCommand.STOP || !snapshot.freeSpaceValid ||
            snapshot.freeSpaceAgeMs > 600L || !p.obstacleDetected ||
            p.nearestObstacleCm == null || p.centerPathFeasible) return false

        val geometryBlocked = p.bestPath == null
        val forwardInhibitedByNearSonar =
            decision.reason.contains("forward motion inhibited", ignoreCase = true) &&
                snapshot.ultrasonicRequired &&
                snapshot.ultrasonicAgeMs <= SafetyConstants.ULTRASONIC_MAX_AGE_MS &&
                snapshot.ultrasonicCm?.let { it > 0f && it < SafetyConstants.ULTRASONIC_STOP_CM } == true
        return geometryBlocked || forwardInhibitedByNearSonar
    }

    private fun isHardSafetyStop(reason: String): Boolean {
        val r = reason.lowercase()
        return listOf(
            "ultrasonic", "traffic sign", "stop sign", "no_entry", "stale camera",
            "free-space model unavailable", "user pressed stop", "simulated collision",
            "arduino not ready", "usb write failed", "usb disconnected"
        ).any { r.contains(it) }
    }

    private fun scanBackoff(note: String) = Output(
        decision = NavigationDecision(
            command = MotorCommand.BACKOFF,
            speedPwm = backoffPwm,
            reason = "active path discovery: $note",
            pathType = PathType.SCANNING,
            boundaryState = BoundaryState.BLOCKED,
            pathAngleDeg = 0f
        ),
        state = state,
        ownsOutput = true,
        isRotationPulse = false
    )

    private fun scanTurn(command: MotorCommand, note: String) = Output(
        decision = NavigationDecision(
            command = command,
            speedPwm = currentScanPwm,
            reason = "active path discovery: $note",
            pathType = PathType.SCANNING,
            boundaryState = BoundaryState.BLOCKED,
            pathAngleDeg = if (command == MotorCommand.PIVOT_LEFT) -10f else 10f
        ),
        state = state,
        ownsOutput = true,
        isRotationPulse = true
    )

    private fun scanWait(note: String) = Output(
        decision = NavigationDecision(
            command = MotorCommand.STOP,
            speedPwm = 0,
            reason = "active path discovery: $note",
            pathType = PathType.SCANNING,
            boundaryState = BoundaryState.BLOCKED
        ),
        state = state,
        ownsOutput = true,
        isRotationPulse = false
    )

    private fun denied(base: NavigationDecision, note: String): Output = Output(
        decision = base.copy(
            command = MotorCommand.STOP,
            speedPwm = 0,
            reason = "active path discovery denied: $note",
            pathType = PathType.STOP,
            boundaryState = BoundaryState.BLOCKED
        ),
        state = state,
        ownsOutput = true,
        isRotationPulse = false
    )

    private fun unsafeStop(note: String): Output {
        state = State.UNSAFE
        return finalStop("active path discovery aborted: $note")
    }

    private fun finalStop(note: String): Output = Output(
        decision = NavigationDecision(
            command = MotorCommand.STOP,
            speedPwm = 0,
            reason = note,
            pathType = PathType.STOP,
            boundaryState = BoundaryState.BLOCKED
        ),
        state = state,
        ownsOutput = true,
        isRotationPulse = false
    )
}

package com.example.robotcar.navigation

/**
 * Shared command stabilizer for the physical and virtual 10 Hz control loops.
 *
 * Any STOP remains immediate, while a new moving
 * direction must persist for two control ticks. During confirmation the output is STOP (never keep
 * driving along an old path whose validity may have changed). PWM increases gradually; reductions
 * and safety stops are immediate. Explicit ActivePathDiscovery pulses bypass this class because
 * that state machine already bounds their duration and checks clearance each tick.
 */
class LiveDecisionStabilizer(
    private val directionConfirmTicks: Int = 2,
    private val pwmRampUpPerTick: Int = 5
) {
    private companion object {
        val TRANSLATING = setOf(MotorCommand.FORWARD, MotorCommand.SLIGHT_LEFT, MotorCommand.SLIGHT_RIGHT)
    }

    private var activeCommand: MotorCommand = MotorCommand.STOP
    private var activePwm: Int = 0
    private var pendingCommand: MotorCommand? = null
    private var pendingTicks: Int = 0
    private var lastSteeringSide: Int = 0
    private var straightTicksSinceTurn: Int = 0

    fun reset() {
        activeCommand = MotorCommand.STOP
        activePwm = 0
        pendingCommand = null
        pendingTicks = 0
        lastSteeringSide = 0
        straightTicksSinceTurn = 0
    }

    /** Immediate virtual swept-footprint/fatal-fault stop using the same output shape as live STOP. */
    fun forceStop(reason: String): NavigationDecision {
        reset()
        return NavigationDecision(
            command = MotorCommand.STOP,
            speedPwm = 0,
            reason = reason,
            pathType = PathType.STOP,
            boundaryState = BoundaryState.BLOCKED
        )
    }

    fun apply(candidate: NavigationDecision): NavigationDecision {
        // Never debounce a STOP, whatever its cause. This includes stale sensors, obstacles,
        // traffic signs, user stop, USB faults, and missing geometry.
        if (candidate.command == MotorCommand.STOP) {
            reset()
            return candidate
        }

        // A short forward-only interval does not erase the last steering side. This catches a noisy
        // LEFT -> FORWARD -> RIGHT sequence as an opposite-direction reversal, not just a direct flip.
        if (candidate.command == MotorCommand.FORWARD) {
            straightTicksSinceTurn++
            if (straightTicksSinceTurn >= 3) lastSteeringSide = 0
        }

        // Same-side changes and F <-> SL/SR remain smooth to avoid stutter from segmentation noise.
        // Opposite steering directions must pass through STOP and confirmation first.
        if (activeCommand in TRANSLATING && candidate.command in TRANSLATING &&
            candidate.command != activeCommand) {
            val activeSide = steeringSide(activeCommand)
            val oldSide = if (activeSide != 0) activeSide else
                if (straightTicksSinceTurn < 3) lastSteeringSide else 0
            val newSide = steeringSide(candidate.command)
            // A direct LEFT-ARC -> RIGHT-ARC reversal is not ordinary frame-to-frame noise.
            // Insert a STOP and require confirmation rather than snapping the chassis across sides.
            if (oldSide != 0 && newSide != 0 && oldSide != newSide) {
                pendingCommand = candidate.command
                pendingTicks = 1
                activeCommand = MotorCommand.STOP
                activePwm = 0
                return candidate.copy(
                    command = MotorCommand.STOP,
                    speedPwm = 0,
                    reason = "live command stabilizer: stopping before opposite steering direction",
                    pathType = PathType.STOP,
                    boundaryState = BoundaryState.BLOCKED,
                    pathAngleDeg = 0f
                )
            }
            clearPending()
            activeCommand = candidate.command
            if (newSide != 0) {
                lastSteeringSide = newSide
                straightTicksSinceTurn = 0
            }
            val nextPwm = if (candidate.speedPwm <= activePwm) candidate.speedPwm
            else minOf(candidate.speedPwm, activePwm + pwmRampUpPerTick)
            activePwm = nextPwm
            return candidate.copy(speedPwm = nextPwm)
        }

        if (candidate.command == activeCommand && activeCommand != MotorCommand.STOP) {
            clearPending()
            val sameSide = steeringSide(candidate.command)
            if (sameSide != 0) {
                lastSteeringSide = sameSide
                straightTicksSinceTurn = 0
            }
            val nextPwm = if (candidate.speedPwm <= activePwm) {
                candidate.speedPwm // speed reductions are immediate
            } else {
                minOf(candidate.speedPwm, activePwm + pwmRampUpPerTick)
            }
            activePwm = nextPwm
            return candidate.copy(speedPwm = nextPwm)
        }

        if (pendingCommand == candidate.command) pendingTicks++ else {
            pendingCommand = candidate.command
            pendingTicks = 1
        }

        if (pendingTicks < directionConfirmTicks) {
            activeCommand = MotorCommand.STOP
            activePwm = 0
            return candidate.copy(
                command = MotorCommand.STOP,
                speedPwm = 0,
                reason = "live command stabilizer: confirming ${candidate.command.name} " +
                    "($pendingTicks/$directionConfirmTicks) - stopped between directions",
                pathType = PathType.STOP,
                boundaryState = BoundaryState.BLOCKED,
                pathAngleDeg = 0f
            )
        }

        activeCommand = candidate.command
        val acceptedSide = steeringSide(candidate.command)
        if (acceptedSide != 0) {
            lastSteeringSide = acceptedSide
            straightTicksSinceTurn = 0
        }
        // The firmware floors moving PWM to MIN_MOTOR_PWM. Start near that real chassis minimum
        // then ramp upward instead of pretending PWM=1..59 is a meaningful physical ramp.
        activePwm = minOf(candidate.speedPwm, SafetyConstants.MIN_MOTOR_PWM + pwmRampUpPerTick)
        clearPending()
        return candidate.copy(speedPwm = activePwm)
    }

    private fun steeringSide(command: MotorCommand): Int = when (command) {
        MotorCommand.SLIGHT_LEFT -> -1
        MotorCommand.SLIGHT_RIGHT -> 1
        else -> 0
    }

    private fun clearPending() {
        pendingCommand = null
        pendingTicks = 0
    }
}

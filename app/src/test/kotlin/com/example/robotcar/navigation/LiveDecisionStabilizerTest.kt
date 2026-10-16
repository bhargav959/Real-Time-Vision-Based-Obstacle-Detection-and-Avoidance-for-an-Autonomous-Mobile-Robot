package com.example.robotcar.navigation

import org.junit.Assert.assertEquals
import org.junit.Test

class LiveDecisionStabilizerTest {
    @Test
    fun requiresTwoMatchingTicksBeforeStartingMotion() {
        val stabilizer = LiveDecisionStabilizer()
        assertEquals(MotorCommand.STOP, stabilizer.apply(motion(MotorCommand.FORWARD, 120)).command)
        val confirmed = stabilizer.apply(motion(MotorCommand.FORWARD, 120))
        assertEquals(MotorCommand.FORWARD, confirmed.command)
        assertEquals(30, confirmed.speedPwm)
    }

    @Test
    fun stopsImmediatelyBetweenDirectionChangesAndRequiresConfirmation() {
        val stabilizer = LiveDecisionStabilizer()
        stabilizer.apply(motion(MotorCommand.FORWARD, 80))
        stabilizer.apply(motion(MotorCommand.FORWARD, 80))
        assertEquals(MotorCommand.STOP, stabilizer.apply(motion(MotorCommand.LEFT, 80)).command)
        assertEquals(MotorCommand.LEFT, stabilizer.apply(motion(MotorCommand.LEFT, 80)).command)
    }

    @Test
    fun anyExplicitStopIsImmediateAndResetsPendingCommand() {
        val stabilizer = LiveDecisionStabilizer()
        stabilizer.apply(motion(MotorCommand.FORWARD, 80))
        val hardStop = NavigationDecision(
            MotorCommand.STOP, 0, "ultrasonic < 30 cm", PathType.STOP, BoundaryState.BLOCKED
        )
        assertEquals(MotorCommand.STOP, stabilizer.apply(hardStop).command)
        assertEquals(MotorCommand.STOP, stabilizer.apply(motion(MotorCommand.LEFT, 80)).command)
    }

    @Test
    fun speedDecreasesImmediatelyButIncreasesGradually() {
        val stabilizer = LiveDecisionStabilizer()
        stabilizer.apply(motion(MotorCommand.FORWARD, 40))
        val start = stabilizer.apply(motion(MotorCommand.FORWARD, 40))
        assertEquals(30, start.speedPwm)
        assertEquals(25, stabilizer.apply(motion(MotorCommand.FORWARD, 25)).speedPwm)
        assertEquals(30, stabilizer.apply(motion(MotorCommand.FORWARD, 40)).speedPwm)
    }


    @Test
    fun stopsBeforeSwitchingFromLeftArcToRightArc() {
        val stabilizer = LiveDecisionStabilizer()
        stabilizer.apply(motion(MotorCommand.SLIGHT_LEFT, 40))
        stabilizer.apply(motion(MotorCommand.SLIGHT_LEFT, 40))
        assertEquals(MotorCommand.STOP, stabilizer.apply(motion(MotorCommand.SLIGHT_RIGHT, 40)).command)
        assertEquals(MotorCommand.SLIGHT_RIGHT, stabilizer.apply(motion(MotorCommand.SLIGHT_RIGHT, 40)).command)
    }


    @Test
    fun stopsBeforeOppositeArcEvenWithOneForwardTickBetween() {
        val stabilizer = LiveDecisionStabilizer()
        stabilizer.apply(motion(MotorCommand.SLIGHT_LEFT, 40))
        stabilizer.apply(motion(MotorCommand.SLIGHT_LEFT, 40))
        stabilizer.apply(motion(MotorCommand.FORWARD, 40))
        assertEquals(MotorCommand.STOP, stabilizer.apply(motion(MotorCommand.SLIGHT_RIGHT, 40)).command)
        assertEquals(MotorCommand.SLIGHT_RIGHT, stabilizer.apply(motion(MotorCommand.SLIGHT_RIGHT, 40)).command)
    }

    private fun motion(command: MotorCommand, pwm: Int) = NavigationDecision(
        command, pwm, "test path", PathType.STRAIGHT, BoundaryState.NOMINAL
    )
}

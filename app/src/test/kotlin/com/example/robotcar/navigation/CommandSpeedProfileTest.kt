package com.example.robotcar.navigation

import org.junit.Assert.assertEquals
import org.junit.Test

class CommandSpeedProfileTest {
    @Test fun straightCruiseIsReducedByTwentyPercent() {
        assertEquals(32, CommandSpeedProfile.adjust(MotorCommand.FORWARD, 40))
        assertEquals(32, CommandSpeedProfile.adjust(MotorCommand.REVERSE, 40))
    }

    @Test fun turnCruiseIsRaisedByFortyFivePercent() {
        assertEquals(58, CommandSpeedProfile.adjust(MotorCommand.LEFT, 40))
        assertEquals(58, CommandSpeedProfile.adjust(MotorCommand.SLIGHT_RIGHT, 40))
    }

    @Test fun noEchoCapStillOverridesTurnSpeed() {
        assertEquals(SafetyConstants.NO_ECHO_MAX_PWM, CommandSpeedProfile.adjust(MotorCommand.RIGHT, 40, noEcho = true))
    }

    @Test fun stopDoesNotReceiveSpeed() {
        assertEquals(0, CommandSpeedProfile.adjust(MotorCommand.STOP, 40))
    }
}

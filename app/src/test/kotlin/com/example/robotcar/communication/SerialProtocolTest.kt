package com.example.robotcar.communication

import com.example.robotcar.navigation.MotorCommand
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Test

class SerialProtocolTest {
    @Test
    fun androidMotionNamesMatchSerialProtocol() {
        assertEquals(
            setOf("F", "SL", "SR", "L", "R", "REV", "BACKOFF", "SCAN_L", "SCAN_R", "S"),
            MotorCommand.values().map { it.wire }.toSet()
        )
    }

    @Test
    fun encodesSupportedMotionCommandsAndClampsPwm() {
        assertEquals("F,150", SerialProtocol.encodeMotion("F", 150))
        assertEquals("SR,0", SerialProtocol.encodeMotion("sr", -12))
        assertEquals("L,255", SerialProtocol.encodeMotion("L", 900))
        assertEquals("BACKOFF,80", SerialProtocol.encodeMotion("BACKOFF", 80))
        assertEquals("SCAN_L,80", SerialProtocol.encodeMotion("SCAN_L", 80))
        assertEquals("SCAN_R,80", SerialProtocol.encodeMotion("SCAN_R", 80))
        assertEquals("S,0", SerialProtocol.encodeMotion("S", 255))
    }

    @Test
    fun rejectsUnknownOrInjectedMotionCommands() {
        assertNull(SerialProtocol.encodeMotion("BACKWARD", 100))
        assertNull(SerialProtocol.encodeMotion("F\nS", 100))
        assertNull(SerialProtocol.encodeMotion("F,150", 100))
    }

    @Test
    fun onlyAllowsKnownTelemetryRequests() {
        assertEquals("STATUS", SerialProtocol.encodeRequest("status"))
        assertEquals("DISTANCE", SerialProtocol.encodeRequest("DISTANCE"))
        assertNull(SerialProtocol.encodeRequest("RESET"))
        assertNull(SerialProtocol.encodeRequest("STATUS\nF,255"))
    }
}

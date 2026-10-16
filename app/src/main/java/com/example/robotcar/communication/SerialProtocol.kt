package com.example.robotcar.communication

import java.util.Locale

/** Single source of truth for the ASCII protocol used by the Android app and Arduino firmware. */
object SerialProtocol {
    const val BAUD_RATE = 115200

    private val motionCommands = setOf("F", "SL", "SR", "L", "R", "REV", "BACKOFF", "SCAN_L", "SCAN_R", "S")
    private val requestCommands = setOf("DISTANCE", "STATUS")

    /** Returns COMMAND,SPEED or null for an unsupported/unsafe command token. */
    fun encodeMotion(command: String, speedPwm: Int): String? {
        val normalized = command.trim().uppercase(Locale.US)
        if (normalized !in motionCommands) return null
        if (normalized == "S") return "S,0"
        return "$normalized,${speedPwm.coerceIn(0, 255)}"
    }

    /** Telemetry requests have no speed field and must be one of the firmware's known queries. */
    fun encodeRequest(command: String): String? {
        val normalized = command.trim().uppercase(Locale.US)
        return normalized.takeIf { it in requestCommands }
    }

    /** One sonar reading carried by a firmware line. [cm] is null when [noEcho]. */
    data class UltrasonicReading(val cm: Float?, val noEcho: Boolean)

    /**
     * Extracts a sonar reading from any firmware line that carries one:
     *   DISTANCE=42.5 | DISTANCE=NO_ECHO | RESULT=F,90,D=41.2 | RESULT=SL,80,D=NO_ECHO |
     *   RESULT=STOP_OBSTACLE:27.5 | RESULT=STOP_TURN_OBSTACLE:19.0
     * Returns null for any line without a well-formed reading (garbage must age out, never read as clear).
     */
    fun parseUltrasonic(line: String): UltrasonicReading? {
        val t = line.trim()
        val raw = when {
            t.startsWith("DISTANCE=", ignoreCase = true) -> t.substringAfter('=')
            t.startsWith("RESULT=STOP_OBSTACLE:", ignoreCase = true) ||
                t.startsWith("RESULT=STOP_TURN_OBSTACLE:", ignoreCase = true) -> t.substringAfter(':')
            t.startsWith("RESULT=", ignoreCase = true) && t.contains(",D=", ignoreCase = true) ->
                t.substringAfterLast("D=")
            else -> return null
        }.trim()
        if (raw.equals("NO_ECHO", ignoreCase = true)) return UltrasonicReading(null, true)
        val cm = raw.toFloatOrNull() ?: return null
        if (!cm.isFinite() || cm <= 0f) return null
        return UltrasonicReading(cm, false)
    }
}

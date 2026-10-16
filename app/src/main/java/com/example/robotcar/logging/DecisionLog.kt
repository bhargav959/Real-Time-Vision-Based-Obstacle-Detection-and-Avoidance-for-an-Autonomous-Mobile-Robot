package com.example.robotcar.logging

import android.content.Context
import android.util.Log
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors

/**
 * Records important robot decisions (mode changes, motor-command/reason changes, confirmed traffic
 * signs, model/USB failures) to logcat (tag "RobotDecision") and to a rotating file.
 *
 * Callers are expected to log on CHANGE only, so this stays quiet during steady driving.
 *
 * File: <app external files dir>/logs/decisions.log (rotates to decisions.1.log at ~1 MB).
 * Pull it with: adb pull /sdcard/Android/data/com.example.robotcar/files/logs/decisions.log
 * or via Android Studio > Device Explorer. Logcat filter: tag:RobotDecision
 */
object DecisionLog {
    const val TAG = "RobotDecision"
    private const val MAX_FILE_BYTES = 1_000_000L

    private var file: File? = null
    private var writer: ExecutorService? = null
    private val timeFormat = SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS", Locale.US)

    @Synchronized
    fun init(context: Context) {
        if (writer != null) return
        try {
            val dir = File(context.getExternalFilesDir(null) ?: context.filesDir, "logs")
            dir.mkdirs()
            file = File(dir, "decisions.log")
            writer = Executors.newSingleThreadExecutor { r -> Thread(r, "decision-log").apply { isDaemon = true } }
        } catch (t: Throwable) {
            safeLog("log file unavailable: ${t.message}")
        }
        log("APP", "Decision log started (file=${file?.absolutePath ?: "none"})")
    }

    /** category examples: MODE, CMD, SIGN, MODEL, USB, SAFETY, APP */
    fun log(category: String, message: String) {
        val line = "[$category] $message"
        safeLog(line)
        val w = writer ?: return
        val f = file ?: return
        val stamp = synchronized(this) { timeFormat.format(Date()) }
        try {
            w.execute { append(f, "$stamp $line\n") }
        } catch (_: Throwable) {
            // Executor already shut down; logcat line above is still recorded.
        }
    }

    @Synchronized
    fun close() {
        try { writer?.shutdown() } catch (_: Throwable) {}
        writer = null
    }

    private fun append(f: File, text: String) {
        try {
            if (f.length() > MAX_FILE_BYTES) {
                val old = File(f.parentFile, "decisions.1.log")
                old.delete()
                f.renameTo(old)
            }
            f.appendText(text)
        } catch (_: Throwable) {
            // Never let logging affect robot control.
        }
    }

    private fun safeLog(line: String) {
        try {
            Log.i(TAG, line)
        } catch (_: Throwable) {
            // android.util.Log is not available in plain JVM unit tests.
        }
    }
}

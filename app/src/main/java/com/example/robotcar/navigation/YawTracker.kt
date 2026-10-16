package com.example.robotcar.navigation

import kotlin.math.abs
import kotlin.math.sqrt

/**
 * Yaw (heading change) estimate from the phone's own gyroscope. The phone rides on the robot, so
 * this is free closed-loop rotation feedback for a chassis that has no encoders or IMU.
 *
 * Mounting-independent: the gyro vector is projected onto the world "up" axis, which is the
 * direction of the gravity-sensor reading (Android's gravity/accelerometer vector points UP when
 * the device is at rest). Counter-clockwise rotation seen from above (a LEFT turn) is positive.
 *
 * Yaw is only ever used DIFFERENTIALLY (before/after a pulse), so slow gyro bias drift is harmless.
 * Pure Kotlin so it is unit-testable off-device.
 */
class YawTracker {
    private var upX = 0f
    private var upY = 0f
    private var upZ = 0f
    private var haveGravity = false
    private var lastGyroNs = Long.MIN_VALUE
    private var yawDeg = 0.0
    private var rateDegS = 0f

    /** Accumulated yaw in degrees, left/CCW positive. Unbounded (not wrapped). */
    @Synchronized fun yawDegrees(): Float = yawDeg.toFloat()

    /** Low-passed yaw rate in deg/s, left positive. */
    @Synchronized fun yawRateDegPerSec(): Float = rateDegS

    /** True once both a gravity vector and at least two gyro samples have arrived. */
    @Synchronized fun isReady(): Boolean = haveGravity && lastGyroNs != Long.MIN_VALUE

    @Synchronized fun reset() {
        yawDeg = 0.0; rateDegS = 0f; lastGyroNs = Long.MIN_VALUE
    }

    /** Feed Sensor.TYPE_GRAVITY (or a low-passed accelerometer): the vector that points up at rest. */
    @Synchronized fun onGravity(x: Float, y: Float, z: Float) {
        val n = sqrt(x * x + y * y + z * z)
        if (n < 1e-3f) return
        upX = x / n; upY = y / n; upZ = z / n
        haveGravity = true
    }

    /** Feed Sensor.TYPE_GYROSCOPE (rad/s, device axes) with its event timestamp in nanoseconds. */
    @Synchronized fun onGyro(wx: Float, wy: Float, wz: Float, timestampNs: Long) {
        if (!haveGravity) { lastGyroNs = timestampNs; return }
        val omegaUp = wx * upX + wy * upY + wz * upZ // rad/s about the vertical axis
        val rate = Math.toDegrees(omegaUp.toDouble())
        if (lastGyroNs != Long.MIN_VALUE) {
            val dt = (timestampNs - lastGyroNs) / 1e9
            // Ignore absurd gaps (sensor paused): do not integrate across them.
            if (dt > 0.0 && dt < 0.25) yawDeg += rate * dt
        }
        lastGyroNs = timestampNs
        rateDegS = rateDegS * 0.6f + rate.toFloat() * 0.4f
    }

    companion object {
        /** Absolute yaw rate considered "settled" (robot not rotating). */
        fun isSettled(rateDegS: Float, limit: Float = MotorCalibration.SCAN_SETTLED_YAW_RATE_DEG_S) = abs(rateDegS) <= limit
    }
}

/** Closed-loop rotation feedback handed to [ActivePathDiscovery]. Yaw: left/CCW positive. */
data class RotationFeedback(val yawDeg: Float, val yawRateDegPerSec: Float)

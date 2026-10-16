package com.example.robotcar.sensors

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import com.example.robotcar.navigation.YawTracker

/**
 * Feeds the phone's gyroscope + gravity sensor into a [YawTracker]. The phone rides on the robot, so
 * this is the robot's only closed-loop rotation measurement. If the sensors are missing, [available]
 * is false and everything stays open-loop exactly as in v0.5.10.
 */
class YawSensor(context: Context, private val tracker: YawTracker) : SensorEventListener {
    private val manager = context.getSystemService(Context.SENSOR_SERVICE) as? SensorManager
    private val gyro: Sensor? = manager?.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
    // TYPE_GRAVITY points UP at rest, like the accelerometer; fall back to the raw accelerometer.
    private val gravity: Sensor? = manager?.getDefaultSensor(Sensor.TYPE_GRAVITY)
        ?: manager?.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
    private val gravityIsRawAccel = gravity?.type == Sensor.TYPE_ACCELEROMETER
    private var lpX = 0f
    private var lpY = 0f
    private var lpZ = 0f
    private var lpInit = false
    @Volatile private var running = false

    val available: Boolean get() = gyro != null && gravity != null

    fun start() {
        val m = manager ?: return
        if (running || !available) return
        // Game-rate (~50 Hz) is plenty; the yaw is compared across scan steps of about 100 ms.
        m.registerListener(this, gyro, SensorManager.SENSOR_DELAY_GAME)
        m.registerListener(this, gravity, SensorManager.SENSOR_DELAY_GAME)
        tracker.reset()
        running = true
    }

    fun stop() {
        if (!running) return
        manager?.unregisterListener(this)
        running = false
        lpInit = false
    }

    override fun onSensorChanged(event: SensorEvent) {
        val v = event.values
        if (v.size < 3) return
        when (event.sensor.type) {
            Sensor.TYPE_GYROSCOPE -> tracker.onGyro(v[0], v[1], v[2], event.timestamp)
            Sensor.TYPE_GRAVITY -> tracker.onGravity(v[0], v[1], v[2])
            Sensor.TYPE_ACCELEROMETER -> if (gravityIsRawAccel) {
                // Heavy low-pass so the motors' vibration does not tilt the "up" estimate.
                if (!lpInit) { lpX = v[0]; lpY = v[1]; lpZ = v[2]; lpInit = true }
                else { lpX += 0.05f * (v[0] - lpX); lpY += 0.05f * (v[1] - lpY); lpZ += 0.05f * (v[2] - lpZ) }
                tracker.onGravity(lpX, lpY, lpZ)
            }
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit
}

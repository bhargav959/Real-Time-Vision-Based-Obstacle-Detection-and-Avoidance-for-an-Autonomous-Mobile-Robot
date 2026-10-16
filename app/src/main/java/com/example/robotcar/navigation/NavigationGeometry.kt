package com.example.robotcar.navigation

import kotlin.math.PI
import kotlin.math.atan
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.tan

/**
 * Pinhole ground projection for a phone mounted 10 cm above the ground and tilted downward.
 * Rows map to ray angles with atan (not linearly) and lateral scale uses the ray slant, so the
 * model is exact for a flat floor and an ideal pinhole. FOV values stay configurable because
 * metric accuracy still needs one calibration pass on the real phone (and the FOV pair must match
 * the analysed image orientation: swap them if the phone is mounted in portrait).
 */
class CameraGroundProjector(
    private val config: NavigationGeometryConfig
) {
    private val tiltRad = config.cameraTiltDeg * PI.toFloat() / 180f
    private val tanHalfV = tan((config.verticalFovDeg * PI.toFloat() / 360f).toDouble()).toFloat()
    private val tanHalfH = tan((config.horizontalFovDeg * PI.toFloat() / 360f).toDouble()).toFloat()

    /** Normalised pinhole ray offset (positive = below the optical axis) for an image row. */
    private fun rayN(row: Int, height: Int): Float {
        val v = row.toFloat() / (height - 1).toFloat()
        return (v - 0.5f) * 2f * tanHalfV
    }

    /** Downward ray component for a unit optical-axis step; <= 0 means the ray never hits the ground. */
    private fun downComponent(n: Float): Float =
        sin(tiltRad.toDouble()).toFloat() + n * cos(tiltRad.toDouble()).toFloat()

    fun groundDistanceCmAtRow(row: Int, height: Int): Float? {
        if (height <= 1 || row !in 0 until height) return null
        val n = rayN(row, height)
        val down = downComponent(n)
        if (down <= 1e-4f) return null
        val forward = cos(tiltRad.toDouble()).toFloat() - n * sin(tiltRad.toDouble()).toFloat()
        val d = config.cameraHeightCm * forward / down
        return if (d > 0f) d else null
    }

    fun rowForDistanceCm(distanceCm: Float, height: Int): Int? {
        if (distanceCm <= 0f || height <= 1) return null
        var bestRow = -1
        var bestError = Float.MAX_VALUE
        for (row in 0 until height) {
            val distance = groundDistanceCmAtRow(row, height) ?: continue
            val error = kotlin.math.abs(distance - distanceCm)
            if (error < bestError) {
                bestError = error
                bestRow = row
            }
        }
        return if (bestRow >= 0) bestRow else null
    }

    /**
     * Lateral ground centimetres spanned by the full image width on this row. A pinhole ray
     * through column x hits the ground at lateral = m * H / down, so the width scales with the
     * ray's slant (1/down), not with forward ground distance.
     */
    private fun lateralScaleCm(row: Int, height: Int): Float? {
        if (height <= 1 || row !in 0 until height) return null
        val down = downComponent(rayN(row, height))
        if (down <= 1e-4f) return null
        return config.cameraHeightCm / down
    }

    fun cmPerPixelAtRow(row: Int, width: Int, height: Int): Float? {
        val scale = lateralScaleCm(row, height) ?: return null
        return 2f * tanHalfH * scale / width.coerceAtLeast(1)
    }

    fun lateralCmAt(row: Int, x: Float, width: Int, height: Int): Float? {
        val scale = lateralScaleCm(row, height) ?: return null
        val m = ((x / width.toFloat()) - 0.5f) * 2f * tanHalfH
        return m * scale
    }

    fun xForLateralCm(row: Int, lateralCm: Float, width: Int, height: Int): Float? {
        val scale = lateralScaleCm(row, height) ?: return null
        if (scale <= 0f) return null
        val m = lateralCm / scale
        return (0.5f + m / (2f * tanHalfH)) * width
    }

    fun headingAngleDeg(lateralOffsetCm: Float, lookaheadCm: Float): Float {
        if (lookaheadCm <= 1f) return 0f
        return atan((lateralOffsetCm / lookaheadCm).toDouble()).toFloat() * 180f / PI.toFloat()
    }
}

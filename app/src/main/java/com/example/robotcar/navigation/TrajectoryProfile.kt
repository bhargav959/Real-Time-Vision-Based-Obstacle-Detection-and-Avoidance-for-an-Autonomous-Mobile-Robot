package com.example.robotcar.navigation

import kotlin.math.pow

/**
 * Low-peak-slope easing profile shared by the planner, virtual pose integrator and POV overlay.
 *
 * The symmetric power curve has zero lateral slope at both ends, but a lower peak derivative than
 * cubic smoothstep (1.25 versus 1.5). This spreads avoidance across more of the lookahead and
 * reduces the sharp middle section while preserving a straight entry/exit heading.
 */
object TrajectoryProfile {
    private const val EXPONENT = 1.25f

    fun lateralFraction(progress: Float): Float {
        val p = progress.coerceIn(0f, 1f)
        if (p <= 0f) return 0f
        if (p >= 1f) return 1f
        val left = p.toDouble().pow(EXPONENT.toDouble()).toFloat()
        val right = (1f - p).toDouble().pow(EXPONENT.toDouble()).toFloat()
        return left / (left + right)
    }

    /** Derivative with respect to normalized path progress. Peak slope is EXPONENT at p=0.5. */
    fun derivative(progress: Float): Float {
        val p = progress.coerceIn(0f, 1f)
        if (p <= 0f || p >= 1f) return 0f
        val left = p.toDouble().pow(EXPONENT.toDouble()).toFloat()
        val right = (1f - p).toDouble().pow(EXPONENT.toDouble()).toFloat()
        val denominator = (left + right) * (left + right)
        val numerator = EXPONENT * p.toDouble().pow((EXPONENT - 1f).toDouble()).toFloat() *
            (1f - p).toDouble().pow((EXPONENT - 1f).toDouble()).toFloat()
        return numerator / denominator
    }
}

package com.example.robotcar.navigation

/**
 * Temporal stabiliser for the semantic mask (class 0=FREE, 1=OBSTACLE, 2=UNKNOWN).
 *
 * Asymmetric on purpose, because a wrong "free" is dangerous and a wrong "obstacle" is only slow:
 *  - OBSTACLE is accepted immediately (one frame) and held until it has decayed for several frames.
 *  - FREE needs [freeConfirmFrames] consecutive free observations; until then the pixel is UNKNOWN
 *    (non-drivable). After a reset, or a long gap between frames, everything starts as UNKNOWN.
 *
 * Not thread-safe: call update() from the single inference thread. reset() may be called from any
 * thread; it takes effect on the next update().
 */
class ObstacleTracker(
    private val obstacleGain: Float = 1.0f,
    private val obstacleDecay: Float = 0.75f,
    private val obstacleThreshold: Float = 0.5f,
    private val freeConfirmFrames: Int = 2,
    private val maxFrameGapMs: Long = 1_000L
) {
    private var width = 0
    private var height = 0
    private var obstacleScore = FloatArray(0)
    private var freeStreak = ByteArray(0)
    private var output = ByteArray(0)
    private var lastUpdateMs = Long.MIN_VALUE

    @Volatile
    private var resetRequested = false

    fun reset() {
        resetRequested = true
    }

    /** Returns a buffer whose first width*height bytes are the stabilised mask (reused each call). */
    fun update(mask: ByteArray, width: Int, height: Int, nowMs: Long): ByteArray {
        val n = width * height
        if (width <= 0 || height <= 0 || mask.size < n) return mask

        val stale = lastUpdateMs != Long.MIN_VALUE && nowMs - lastUpdateMs > maxFrameGapMs
        if (resetRequested || stale || width != this.width || height != this.height) {
            this.width = width
            this.height = height
            obstacleScore = FloatArray(n)
            freeStreak = ByteArray(n)
            output = ByteArray(n)
            resetRequested = false
        }
        lastUpdateMs = nowMs

        for (i in 0 until n) {
            val cls = mask[i].toInt() and 0xFF
            var score = obstacleScore[i]
            if (cls == 1) {
                score = minOf(1f, score + obstacleGain)
                freeStreak[i] = 0
            } else {
                score *= obstacleDecay
                if (cls == 0) {
                    if (freeStreak[i] < 100) freeStreak[i] = (freeStreak[i] + 1).toByte()
                } else {
                    freeStreak[i] = 0
                }
            }
            obstacleScore[i] = score

            output[i] = when {
                score >= obstacleThreshold -> 1
                cls == 0 && freeStreak[i] >= freeConfirmFrames -> 0
                else -> 2
            }
        }
        return output
    }
}

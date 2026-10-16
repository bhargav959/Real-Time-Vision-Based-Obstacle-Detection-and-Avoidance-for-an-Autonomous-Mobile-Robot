package com.example.robotcar.video

import java.util.Random
import kotlin.math.exp
import kotlin.math.max
import kotlin.math.min

/**
 * Imperfections the ideal virtual harness lacked, so that "passes in the sim" means more about
 * "works on the floor". Every term is optional; [IDEAL] reproduces the old noise-free behaviour
 * (used by the deterministic unit tests). All randomness is seeded: runs are repeatable.
 *
 * Values in [REALISTIC] are PLAUSIBLE PLACEHOLDERS for a small L298N skid-steer chassis with a
 * phone-camera segmentation model. Replace them with measured numbers (see docs/SIM_REALITY_CALIBRATION).
 */
data class SimRealism(
    /** Camera -> decision delay in 100 ms frames (capture, preprocessing, inference, tracker). */
    val perceptionLatencyFrames: Int = 0,
    /** Per-frame probability of a false obstacle blob on free floor (glare, shadow, floor pattern). */
    val falseObstacleBlobProb: Float = 0f,
    /** Per-frame probability that a patch of a real obstacle is missed (classified free). */
    val missedObstaclePatchProb: Float = 0f,
    /** Whole-mask horizontal jitter, +-pixels (model boundary wobble, phone vibration). */
    val maskJitterPx: Int = 0,
    /** Right wheel speed relative to the left (motor/gearbox mismatch -> drift). 1.0 = identical. */
    val rightWheelGain: Float = 1f,
    /** PWM below which a wheel does not turn under load. Speed rises linearly from here to PWM 150. */
    val motorDeadbandPwm: Int = 0,
    /** First-order motor spin-up time constant; short pulses from rest achieve only part of nominal motion. */
    val motorLagTauMs: Float = 0f,
    /** Fraction of ideal wheel speed lost to slip/scrub when pivoting in place (skid-steer friction). */
    val pivotSlipLoss: Float = 0f,
    /** Probability a sonar reading is a miss (NO_ECHO) although an obstacle is in the beam. */
    val sonarDropoutProb: Float = 0f,
    /** Probability of a spurious near reading (multipath, motor noise). */
    val sonarSpuriousProb: Float = 0f,
    /** Gaussian sonar range noise sigma (cm). */
    val sonarSigmaCm: Float = 0f,
    val seed: Long = 1L
) {
    val enabled: Boolean
        get() = this != IDEAL

    companion object {
        val IDEAL = SimRealism()
        val REALISTIC = SimRealism(
            perceptionLatencyFrames = 3,
            falseObstacleBlobProb = 0.05f,
            missedObstaclePatchProb = 0.08f,
            maskJitterPx = 1,
            rightWheelGain = 0.93f,
            motorDeadbandPwm = 24, // Placeholder aligned with the current 25-PWM floor; measure on the real chassis
            motorLagTauMs = 60f,
            pivotSlipLoss = 0.25f,
            sonarDropoutProb = 0.04f,
            sonarSpuriousProb = 0.01f,
            sonarSigmaCm = 0.8f
        )
    }
}

/** Stateful applier for [SimRealism]'s perception terms (latency queue + mask corruption). */
class SimPerceptionModel(private val cfg: SimRealism) {
    private val rng = Random(cfg.seed)
    private val queue = ArrayDeque<ByteArray>()

    /** Latency reported to the controller as free-space age (ms). */
    val latencyMs: Long get() = cfg.perceptionLatencyFrames * 100L

    fun reset() { queue.clear() }

    /** Deterministic 0..1 for sonar/other draws that share this model's RNG stream. */
    fun nextFloat(): Float = rng.nextFloat()
    fun nextGaussian(): Float = rng.nextGaussian().toFloat()

    /** Returns the mask the controller will "see" this frame: delayed, jittered, with misses/false blobs. */
    fun process(truth: ByteArray, width: Int, height: Int): ByteArray {
        if (!cfg.enabled) return truth
        queue.addLast(truth.copyOf())
        while (queue.size > cfg.perceptionLatencyFrames + 1) queue.removeFirst()
        // Until the pipeline fills, use the oldest available frame (as a camera warming up would).
        val delayed = queue.first().copyOf()

        if (cfg.maskJitterPx > 0) {
            val dx = rng.nextInt(2 * cfg.maskJitterPx + 1) - cfg.maskJitterPx
            if (dx != 0) shiftHorizontally(delayed, width, height, dx)
        }
        if (cfg.missedObstaclePatchProb > 0f && rng.nextFloat() < cfg.missedObstaclePatchProb) {
            punchHole(delayed, width, height)
        }
        if (cfg.falseObstacleBlobProb > 0f && rng.nextFloat() < cfg.falseObstacleBlobProb) {
            addBlob(delayed, width, height)
        }
        return delayed
    }

    private fun shiftHorizontally(m: ByteArray, w: Int, h: Int, dx: Int) {
        val row = ByteArray(w)
        for (y in 0 until h) {
            val base = y * w
            for (x in 0 until w) {
                val sx = x - dx
                row[x] = if (sx in 0 until w) m[base + sx] else m[base + x]
            }
            System.arraycopy(row, 0, m, base, w)
        }
    }

    /** Erase an obstacle patch (8..20 px) located on a randomly chosen obstacle pixel, if any. */
    private fun punchHole(m: ByteArray, w: Int, h: Int) {
        var cx = -1; var cy = -1
        for (attempt in 0 until 40) {
            val x = rng.nextInt(w); val y = rng.nextInt(h)
            if (m[y * w + x].toInt() == 1) { cx = x; cy = y; break }
        }
        if (cx < 0) return
        val r = 4 + rng.nextInt(7)
        for (y in max(0, cy - r)..min(h - 1, cy + r)) for (x in max(0, cx - r)..min(w - 1, cx + r)) {
            if (m[y * w + x].toInt() == 1) m[y * w + x] = 0
        }
    }

    /** Add a 6..14 px obstacle blob on the lower (nearer) 60% of the image. */
    private fun addBlob(m: ByteArray, w: Int, h: Int) {
        val r = 3 + rng.nextInt(5)
        val cx = r + rng.nextInt(max(1, w - 2 * r))
        val cy = (h * 0.4f).toInt() + rng.nextInt(max(1, (h * 0.6f).toInt() - r))
        for (y in max(0, cy - r)..min(h - 1, cy + r)) for (x in max(0, cx - r)..min(w - 1, cx + r)) {
            if (m[y * w + x].toInt() == 0) m[y * w + x] = 1
        }
    }
}

/** Mean fraction of nominal speed achieved over [dtMs] when starting from rest (first-order lag, time constant tauMs). */
internal fun spinUpEfficiency(dtMs: Float, tauMs: Float): Float {
    if (tauMs <= 0f || dtMs <= 0f) return 1f
    return 1f - (tauMs / dtMs) * (1f - exp(-dtMs / tauMs))
}

package com.example.robotcar.ai

import com.example.robotcar.navigation.TrafficSign
import com.example.robotcar.vision.ModelInputPreprocessor
import kotlin.math.exp
import kotlin.math.max
import kotlin.math.min
import kotlin.math.roundToInt

/**
 * Decoder for the custom Ultralytics YOLO traffic-sign TFLite export.
 * Model signature validated during conversion: float32 [1, 3, 640, 640] -> [1, 11, 8400]
 * (4 xywh values + 7 class scores per candidate). Class order is the one stored in the checkpoint
 * metadata (names 0..6) and in assets/models/traffic_sign_labels.txt.
 */
object TrafficSignDecoder {
    /** Must stay in exactly the class order stored in the checkpoint's metadata (names 0..6). */
    val labels = listOf(
        "compulsory_right",
        "no_stopping_or_parking",
        "right_turn_prohibited",
        "right_reverse_bend",
        "side_road_right",
        "stop",
        "turn_right",
    )

    private const val INPUT_SIZE = 640f
    private const val SCORE_THRESHOLD = 0.50f
    private const val NMS_IOU = 0.45f
    private const val MAX_CANDIDATES = 300
    /** Labels the navigation logic reacts to. Anything here that is not in [labels] can never trigger. */
    val behaviorLabels = setOf("no_entry", "stop")

    /** Behavior labels the loaded model cannot produce (currently "no_entry"). Logged at model load. */
    val unsupportedBehaviorLabels: Set<String> get() = behaviorLabels - labels.toSet()

    private data class Candidate(
        val classId: Int,
        val score: Float,
        val x1: Float,
        val y1: Float,
        val x2: Float,
        val y2: Float
    )

    fun decode(
        output: TfliteModelRunner.Output,
        transform: ModelInputPreprocessor.LetterboxTransform
    ): TrafficSign? {
        val shape = output.shape
        val values = output.values
        val nc = labels.size
        val attrs = 4 + nc

        val channelFirst: Boolean
        val candidatesCount: Int
        when {
            shape.size == 3 && shape[0] == 1 && shape[1] == attrs -> {
                channelFirst = true
                candidatesCount = shape[2]
            }
            shape.size == 3 && shape[0] == 1 && shape[2] == attrs -> {
                channelFirst = false
                candidatesCount = shape[1]
            }
            shape.size == 2 && shape[0] == attrs -> {
                channelFirst = true
                candidatesCount = shape[1]
            }
            shape.size == 2 && shape[1] == attrs -> {
                channelFirst = false
                candidatesCount = shape[0]
            }
            else -> error(
                "Unsupported YOLO traffic output shape ${shape.contentToString()}; " +
                    "expected [1,$attrs,N] or [1,N,$attrs]"
            )
        }
        require(values.size == attrs * candidatesCount) {
            "Output element count ${values.size} does not match shape ${shape.contentToString()}"
        }

        val candidates = ArrayList<Candidate>()
        for (i in 0 until candidatesCount) {
            val x = value(values, 0, i, candidatesCount, channelFirst)
            val y = value(values, 1, i, candidatesCount, channelFirst)
            val w = value(values, 2, i, candidatesCount, channelFirst)
            val h = value(values, 3, i, candidatesCount, channelFirst)
            if (!x.isFinite() || !y.isFinite() || !w.isFinite() || !h.isFinite() || w <= 0f || h <= 0f) continue

            var bestClass = -1
            var bestScore = 0f
            for (c in 0 until nc) {
                val raw = value(values, 4 + c, i, candidatesCount, channelFirst)
                val score = probability(raw)
                if (score > bestScore) {
                    bestScore = score
                    bestClass = c
                }
            }
            if (bestClass < 0 || bestScore < SCORE_THRESHOLD) continue

            // Ultralytics detection exports normally use xywh in 640-space for end2end=false.
            // If a third-party converter emitted normalized coordinates, handle that too.
            val scaleToPixels = if (max(max(x, y), max(w, h)) <= 1.5f) INPUT_SIZE else 1f
            val cx = x * scaleToPixels
            val cy = y * scaleToPixels
            val bw = w * scaleToPixels
            val bh = h * scaleToPixels
            val x1 = (cx - bw * 0.5f).coerceIn(0f, INPUT_SIZE)
            val y1 = (cy - bh * 0.5f).coerceIn(0f, INPUT_SIZE)
            val x2 = (cx + bw * 0.5f).coerceIn(0f, INPUT_SIZE)
            val y2 = (cy + bh * 0.5f).coerceIn(0f, INPUT_SIZE)
            if (x2 - x1 < 2f || y2 - y1 < 2f) continue

            // Discard detections whose center lies in letterbox padding.
            val centerXInput = (x1 + x2) * 0.5f
            val centerYInput = (y1 + y2) * 0.5f
            if (centerXInput < transform.padLeft || centerXInput >= transform.padLeft + transform.resizedWidth) continue
            if (centerYInput < transform.padTop || centerYInput >= transform.padTop + transform.resizedHeight) continue

            candidates += Candidate(bestClass, bestScore, x1, y1, x2, y2)
        }

        if (candidates.isEmpty()) return null
        val top = candidates.sortedByDescending { it.score }.take(MAX_CANDIDATES)
        val kept = mutableListOf<Candidate>()
        for (candidate in top) {
            if (kept.none { it.classId == candidate.classId && iou(it, candidate) > NMS_IOU }) {
                kept += candidate
            }
        }
        // Use label names rather than raw IDs so retraining with a different class set cannot
        // silently remap a safety behavior (stop is class 5 in the current 7-class model).
        val safetyCritical = kept.filter {
            it.score >= 0.75f && labels[it.classId] in behaviorLabels
        }
        val best = (safetyCritical.maxByOrNull { it.score } ?: kept.maxByOrNull { it.score })
            ?: return null

        val rawCenterX = (best.x1 + best.x2) * 0.5f
        val contentX = ((rawCenterX - transform.padLeft) / transform.resizedWidth).coerceIn(0f, 1f)
        return TrafficSign(
            label = labels[best.classId],
            confidence = best.score,
            centerX = contentX
        )
    }

    private fun value(values: FloatArray, attr: Int, candidate: Int, count: Int, channelFirst: Boolean): Float =
        if (channelFirst) values[attr * count + candidate] else values[candidate * (4 + labels.size) + attr]

    private fun probability(raw: Float): Float =
        if (raw in 0f..1f) raw else (1f / (1f + exp(-raw)))

    private fun iou(a: Candidate, b: Candidate): Float {
        val ix1 = max(a.x1, b.x1)
        val iy1 = max(a.y1, b.y1)
        val ix2 = min(a.x2, b.x2)
        val iy2 = min(a.y2, b.y2)
        val iw = (ix2 - ix1).coerceAtLeast(0f)
        val ih = (iy2 - iy1).coerceAtLeast(0f)
        val inter = iw * ih
        val areaA = (a.x2 - a.x1) * (a.y2 - a.y1)
        val areaB = (b.x2 - b.x1) * (b.y2 - b.y1)
        return inter / (areaA + areaB - inter + 1e-6f)
    }
}

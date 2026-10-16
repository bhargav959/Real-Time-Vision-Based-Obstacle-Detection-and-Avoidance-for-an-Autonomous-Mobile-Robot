package com.example.robotcar.navigation

/** Converts a semantic class mask (0=FREE, 1=OBSTACLE, 2=UNKNOWN) into navigation scores. */
class FreeSpaceAnalyzer(
    private val roiTopFraction: Float = 0.55f,
    private val unknownPenalty: Float = 0.35f
) {
    data class Result(
        val freeLeft: Float,
        val freeCenter: Float,
        val freeRight: Float,
        val obstacleCenter: Float,
        val unknownCenter: Float
    )

    fun analyze(mask: ByteArray, width: Int, height: Int, stride: Int = width): Result {
        if (width <= 0 || height <= 0 || mask.isEmpty()) return Result(0f, 0f, 0f, 1f, 1f)
        val top = (height * roiTopFraction).toInt().coerceIn(0, height - 1)

        var leftScore = 0f
        var leftTotal = 0
        var centerScore = 0f
        var centerTotal = 0
        var rightScore = 0f
        var rightTotal = 0
        var centerObstacle = 0
        var centerUnknown = 0
        var centerCount = 0

        for (y in top until height) {
            val base = y * stride
            for (x in 0 until width) {
                val idx = base + x
                if (idx >= mask.size) continue
                val cls = mask[idx].toInt() and 0xFF
                // 255 is the BDD/Ultralytics ignore value; it must not be
                // counted as either free space or obstacle space.
                if (cls == 255) continue

                val score = when (cls) {
                    0 -> 1f
                    2 -> unknownPenalty
                    1 -> 0f
                    else -> 0f
                }
                when {
                    x < width / 3 -> {
                        leftScore += score
                        leftTotal++
                    }
                    x < 2 * width / 3 -> {
                        centerScore += score
                        centerTotal++
                        centerCount++
                        if (cls == 1) centerObstacle++
                        if (cls == 2) centerUnknown++
                    }
                    else -> {
                        rightScore += score
                        rightTotal++
                    }
                }
            }
        }

        return Result(
            freeLeft = if (leftTotal == 0) 0f else leftScore / leftTotal,
            freeCenter = if (centerTotal == 0) 0f else centerScore / centerTotal,
            freeRight = if (rightTotal == 0) 0f else rightScore / rightTotal,
            obstacleCenter = if (centerCount == 0) 1f else centerObstacle / centerCount.toFloat(),
            unknownCenter = if (centerCount == 0) 1f else centerUnknown / centerCount.toFloat()
        )
    }
}

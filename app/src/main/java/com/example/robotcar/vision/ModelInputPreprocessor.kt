package com.example.robotcar.vision

import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import kotlin.math.roundToInt

/**
 * Ultralytics-style square letterbox preprocessing with 114/255 padding and RGB/255 input.
 *
 * The object-level functions are kept for compatibility with small unit tests. The Android
 * camera path uses [Reusable] so the letterbox Bitmap, pixel scratch buffer and FloatArray are
 * reused between inferences instead of being allocated on every frame.
 */
object ModelInputPreprocessor {
    private const val PAD_VALUE = 114

    data class LetterboxTransform(
        val modelSize: Int,
        val originalWidth: Int,
        val originalHeight: Int,
        val resizedWidth: Int,
        val resizedHeight: Int,
        val padLeft: Int,
        val padTop: Int,
        val scale: Float
    )

    data class PreparedInput(
        val floats: FloatArray,
        val transform: LetterboxTransform
    )

    data class CroppedMask(
        val data: ByteArray,
        val width: Int,
        val height: Int
    )

    /** Reusable preprocessing scratch for a single model input size/layout. */
    class Reusable(private val size: Int, @Volatile var channelFirst: Boolean) : AutoCloseable {
        init {
            require(size > 0) { "Model input size must be positive" }
        }

        private val letterboxed = Bitmap.createBitmap(size, size, Bitmap.Config.ARGB_8888)
        private val canvas = Canvas(letterboxed)
        private val paint = Paint(Paint.ANTI_ALIAS_FLAG).apply { isFilterBitmap = true }
        private val pixels = IntArray(size * size)
        private val floats = FloatArray(size * size * 3)

        /**
         * Letterboxes [bitmap] into the reusable square. [rotationDegrees] (a multiple of 90, as
         * reported by CameraX) is applied while drawing, so the caller no longer needs to allocate a
         * second, rotated copy of the camera frame. The returned transform describes the UPRIGHT image.
         */
        fun prepare(bitmap: Bitmap, rotationDegrees: Int = 0): PreparedInput {
            require(bitmap.width > 0 && bitmap.height > 0) { "Bitmap must be non-empty" }
            val rotation = ((rotationDegrees % 360) + 360) % 360
            require(rotation % 90 == 0) { "Rotation must be a multiple of 90 degrees: $rotationDegrees" }
            val swap = rotation == 90 || rotation == 270
            val uprightW = if (swap) bitmap.height else bitmap.width
            val uprightH = if (swap) bitmap.width else bitmap.height

            val scale = minOf(size.toFloat() / uprightW, size.toFloat() / uprightH)
            val resizedW = (uprightW * scale).roundToInt().coerceAtLeast(1)
            val resizedH = (uprightH * scale).roundToInt().coerceAtLeast(1)
            val padLeft = (size - resizedW) / 2
            val padTop = (size - resizedH) / 2

            val transform = LetterboxTransform(
                modelSize = size,
                originalWidth = uprightW,
                originalHeight = uprightH,
                resizedWidth = resizedW,
                resizedHeight = resizedH,
                padLeft = padLeft,
                padTop = padTop,
                scale = scale
            )

            canvas.drawColor(Color.rgb(PAD_VALUE, PAD_VALUE, PAD_VALUE))
            // Draw directly into the reusable letterbox (no temporary scaled or rotated Bitmap).
            // Matrix order: scale (raw axes) -> rotate clockwise -> move to the destination centre.
            val rawW = bitmap.width.toFloat()
            val rawH = bitmap.height.toFloat()
            val sx = (if (swap) resizedH else resizedW) / rawW
            val sy = (if (swap) resizedW else resizedH) / rawH
            canvas.save()
            canvas.translate(padLeft + resizedW / 2f, padTop + resizedH / 2f)
            canvas.rotate(rotation.toFloat())
            canvas.scale(sx, sy)
            canvas.drawBitmap(bitmap, -rawW / 2f, -rawH / 2f, paint)
            canvas.restore()

            letterboxed.getPixels(pixels, 0, size, 0, 0, size, size)
            if (channelFirst) {
                val plane = pixels.size
                for (i in pixels.indices) {
                    val p = pixels[i]
                    floats[i] = Color.red(p) / 255f
                    floats[plane + i] = Color.green(p) / 255f
                    floats[2 * plane + i] = Color.blue(p) / 255f
                }
            } else {
                var j = 0
                for (p in pixels) {
                    floats[j++] = Color.red(p) / 255f
                    floats[j++] = Color.green(p) / 255f
                    floats[j++] = Color.blue(p) / 255f
                }
            }
            return PreparedInput(floats, transform)
        }

        override fun close() {
            if (!letterboxed.isRecycled) letterboxed.recycle()
        }
    }

    fun prepareNchw(bitmap: Bitmap, size: Int): PreparedInput = prepare(bitmap, size, channelFirst = true)

    fun prepareNhwc(bitmap: Bitmap, size: Int): PreparedInput = prepare(bitmap, size, channelFirst = false)

    fun toFloatNchw(bitmap: Bitmap, size: Int): FloatArray = prepareNchw(bitmap, size).floats

    fun toFloatNhwc(bitmap: Bitmap, size: Int): FloatArray = prepareNhwc(bitmap, size).floats

    /**
     * Crop a semantic mask from the padded square back to the actual image content.
     * When [scratch] is large enough it is reused for the result (so [CroppedMask.data] may be longer
     * than width*height; consumers must index with the returned width/height only).
     */
    fun cropMaskToContent(
        mask: ByteArray,
        maskWidth: Int,
        maskHeight: Int,
        transform: LetterboxTransform,
        scratch: ByteArray? = null
    ): CroppedMask {
        require(maskWidth > 0 && maskHeight > 0) { "Invalid mask dimensions" }
        require(mask.size >= maskWidth * maskHeight) {
            "Mask buffer is smaller than ${maskWidth}x$maskHeight"
        }

        val sx = maskWidth.toFloat() / transform.modelSize
        val sy = maskHeight.toFloat() / transform.modelSize
        val x0 = (transform.padLeft * sx).roundToInt().coerceIn(0, maskWidth - 1)
        val y0 = (transform.padTop * sy).roundToInt().coerceIn(0, maskHeight - 1)
        val x1 = ((transform.padLeft + transform.resizedWidth) * sx)
            .roundToInt().coerceIn(x0 + 1, maskWidth)
        val y1 = ((transform.padTop + transform.resizedHeight) * sy)
            .roundToInt().coerceIn(y0 + 1, maskHeight)
        val outWidth = x1 - x0
        val outHeight = y1 - y0
        val needed = outWidth * outHeight
        val out = if (scratch != null && scratch.size >= needed) scratch else ByteArray(needed)

        for (y in 0 until outHeight) {
            val srcBase = (y0 + y) * maskWidth + x0
            val dstBase = y * outWidth
            mask.copyInto(out, dstBase, srcBase, srcBase + outWidth)
        }
        return CroppedMask(out, outWidth, outHeight)
    }

    private fun prepare(bitmap: Bitmap, size: Int, channelFirst: Boolean): PreparedInput {
        return Reusable(size, channelFirst).use { it.prepare(bitmap) }
    }
}

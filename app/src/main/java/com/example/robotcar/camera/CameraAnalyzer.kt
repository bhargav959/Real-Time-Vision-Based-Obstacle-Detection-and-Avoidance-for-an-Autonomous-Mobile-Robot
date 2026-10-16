package com.example.robotcar.camera

import android.graphics.Bitmap
import androidx.camera.core.ImageAnalysis
import android.os.SystemClock
import androidx.camera.core.ImageProxy

class CameraAnalyzer(
    private val shouldCaptureModelFrame: () -> Boolean,
    /**
     * RAW un-rotated model bitmap or null, model-frame requested, rotationDegrees, and the
     * elapsedRealtime (ms) at which the frame reached the analyzer. Downstream ages are measured
     * from this capture time, NOT from when inference finished, so inference latency is visible.
     */
    private val onResult: (Bitmap?, Boolean, Int, Long) -> Unit
) : ImageAnalysis.Analyzer {
    override fun analyze(image: ImageProxy) {
        val capturedAtMs = SystemClock.elapsedRealtime()
        var modelBitmap: Bitmap? = null
        var modelFrameRequested = false
        val rotationDegrees = image.imageInfo.rotationDegrees
        try {
            modelFrameRequested = shouldCaptureModelFrame()
            if (modelFrameRequested) {
                modelBitmap = runCatching { image.toBitmap() }.getOrNull()
            }
        } catch (_: Throwable) {
            // Keep the camera analyzer alive and let the model path fail closed if needed.
        } finally {
            image.close()
        }
        onResult(modelBitmap, modelFrameRequested, rotationDegrees, capturedAtMs)
    }
}

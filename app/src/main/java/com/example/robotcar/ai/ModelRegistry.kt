package com.example.robotcar.ai

import android.content.Context
import android.util.Log

class ModelRegistry(private val context: Context) : AutoCloseable {
    companion object {
        const val FREE_SPACE_MODEL = "models/free_space.onnx"
        const val TRAFFIC_SIGN_MODEL = "models/traffic_sign.tflite"
        private const val TAG = "ModelRegistry"
    }

    val freeSpace = OnnxModelRunner(context, FREE_SPACE_MODEL)
    val trafficSign = TfliteModelRunner(context, TRAFFIC_SIGN_MODEL)

    fun loadAvailable(): List<String> {
        val loaded = mutableListOf<String>()
        freeSpace.load().onSuccess { info ->
            Log.i(TAG, "Loaded ${info.assetName}: input=${info.inputShape.contentToString()} outputs=${info.outputSummary}")
            loaded += info.assetName
        }.onFailure { error ->
            Log.e(TAG, "Free-space model failed to load", error)
        }
        trafficSign.load().onSuccess { info ->
            Log.i(TAG, "Loaded ${info.assetName}: input=${info.inputShape.contentToString()} output=${info.outputShapes.joinToString { it.contentToString() }}")
            loaded += info.assetName
        }.onFailure { error ->
            Log.e(TAG, "Traffic-sign model failed to load", error)
        }
        return loaded
    }

    val areRequiredModelsLoaded: Boolean
        get() = freeSpace.isLoaded && trafficSign.isLoaded

    override fun close() {
        freeSpace.close()
        trafficSign.close()
    }
}

package com.example.robotcar.ai

import android.content.Context
import org.tensorflow.lite.DataType
import org.tensorflow.lite.Interpreter
import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * LiteRT/LiteRT/TFLite Interpreter wrapper for the converted custom Ultralytics traffic-sign model.
 * Reuses the input/output buffers and output FloatArray between inferences.
 */
class TfliteModelRunner(
    private val context: Context,
    private val assetName: String,
    private val numThreads: Int = 4
) : AutoCloseable {
    private var interpreter: Interpreter? = null
    private var modelBuffer: ByteBuffer? = null
    private var inputBuffer: ByteBuffer? = null
    private var outputBuffer: ByteBuffer? = null
    private var reusableOutput: FloatArray? = null
    private var outputShape: IntArray = intArrayOf()

    /** True when the model input is NCHW [1,3,640,640]; false for NHWC [1,640,640,3]. Set by [load]. */
    @Volatile
    var inputChannelFirst: Boolean = false
        private set

    val isLoaded: Boolean
        get() = interpreter != null

    data class ModelInfo(
        val assetName: String,
        val inputShape: IntArray,
        val inputType: String,
        val outputShapes: List<IntArray>,
        val outputTypes: List<String>
    )

    data class Output(
        val shape: IntArray,
        val values: FloatArray
    )

    @Synchronized
    fun load(): Result<ModelInfo> = runCatching {
        close()

        val bytes = context.assets.open(assetName).use { it.readBytes() }
        require(bytes.isNotEmpty()) { "Traffic model asset is empty: $assetName" }
        val mapped = ByteBuffer.allocateDirect(bytes.size).order(ByteOrder.nativeOrder())
        mapped.put(bytes)
        mapped.rewind()

        val options = Interpreter.Options().apply {
            setNumThreads(numThreads.coerceAtLeast(1))
        }
        val newInterpreter = Interpreter(mapped, options)
        try {
            val inputTensor = newInterpreter.getInputTensor(0)
            require(inputTensor.dataType() == DataType.FLOAT32) {
                "Traffic model input type ${inputTensor.dataType()} is not FLOAT32"
            }
            val inShape = inputTensor.shape()
            val nhwc = inShape.contentEquals(intArrayOf(1, 640, 640, 3))
            val nchw = inShape.contentEquals(intArrayOf(1, 3, 640, 640))
            require(nhwc || nchw) {
                "Traffic model input must be [1,640,640,3] or [1,3,640,640], got ${inShape.contentToString()}"
            }
            inputChannelFirst = nchw

            require(newInterpreter.outputTensorCount == 1) {
                "Expected one traffic output tensor, got ${newInterpreter.outputTensorCount}"
            }
            val outTensor = newInterpreter.getOutputTensor(0)
            require(outTensor.dataType() == DataType.FLOAT32) {
                "Traffic model output type ${outTensor.dataType()} is not FLOAT32"
            }
            require(isSupportedOutputShape(outTensor.shape())) {
                "Unsupported traffic model output shape ${outTensor.shape().contentToString()}; " +
                    "expected 4 + ${TrafficSignDecoder.labels.size} = ${4 + TrafficSignDecoder.labels.size} attributes " +
                    "in [1,A,N], [1,N,A], [A,N], or [N,A] layout"
            }

            interpreter = newInterpreter
            modelBuffer = mapped
            outputShape = outTensor.shape().clone()
            inputBuffer = ByteBuffer.allocateDirect(640 * 640 * 3 * 4).order(ByteOrder.nativeOrder())
            outputBuffer = ByteBuffer.allocateDirect(outTensor.numElements() * 4).order(ByteOrder.nativeOrder())
            reusableOutput = FloatArray(outTensor.numElements())

            val outputShapes = (0 until newInterpreter.outputTensorCount).map {
                newInterpreter.getOutputTensor(it).shape().clone()
            }
            val outputTypes = (0 until newInterpreter.outputTensorCount).map {
                newInterpreter.getOutputTensor(it).dataType().name
            }

            ModelInfo(
                assetName = assetName,
                inputShape = inputTensor.shape().clone(),
                inputType = inputTensor.dataType().name,
                outputShapes = outputShapes,
                outputTypes = outputTypes
            )
        } catch (t: Throwable) {
            try { newInterpreter.close() } catch (_: Throwable) {}
            throw t
        }
    }

    @Synchronized
    fun runFloat(input: FloatArray): Output {
        val i = interpreter ?: error("Model not loaded: $assetName")
        require(input.size == 640 * 640 * 3) {
            "Traffic input must contain ${640 * 640 * 3} floats, got ${input.size}"
        }
        val inBuffer = inputBuffer ?: error("Input buffer not allocated")
        val outBuffer = outputBuffer ?: error("Output buffer not allocated")
        val output = reusableOutput ?: error("Output scratch not allocated")

        inBuffer.rewind()
        inBuffer.asFloatBuffer().put(input)
        inBuffer.rewind()
        outBuffer.clear()
        i.run(inBuffer, outBuffer)
        outBuffer.rewind()
        outBuffer.asFloatBuffer().get(output)
        return Output(outputShape, output)
    }

    private fun isSupportedOutputShape(shape: IntArray): Boolean {
        // YOLO detection output = 4 box coordinates + one score per trained class.
        // The packaged custom traffic-sign model has 7 classes, so attrs = 11 ([1,11,8400]).
        val attrs = 4 + TrafficSignDecoder.labels.size
        return when {
            shape.size == 3 && shape[0] == 1 && shape[1] == attrs && shape[2] > 0 -> true
            shape.size == 3 && shape[0] == 1 && shape[2] == attrs && shape[1] > 0 -> true
            shape.size == 2 && shape[0] == attrs && shape[1] > 0 -> true
            shape.size == 2 && shape[1] == attrs && shape[0] > 0 -> true
            else -> false
        }
    }

    @Synchronized
    override fun close() {
        try { interpreter?.close() } catch (_: Throwable) {}
        interpreter = null
        modelBuffer = null
        inputBuffer = null
        outputBuffer = null
        reusableOutput = null
        outputShape = intArrayOf()
    }
}

package com.example.robotcar.ai

import ai.onnxruntime.OnnxJavaType
import ai.onnxruntime.OnnxTensor
import ai.onnxruntime.OrtEnvironment
import ai.onnxruntime.OrtSession
import ai.onnxruntime.TensorInfo
import android.content.Context
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer

/**
 * ONNX Runtime wrapper for the supplied free-space YOLO26 semantic model.
 *
 * GC notes: the input goes through one reusable direct FloatBuffer (no heap copy per run), only the
 * first output is requested, and the semantic mask is decoded straight from the output tensor's
 * buffer into a caller-supplied ByteArray. The previous version materialised the output as nested
 * Java arrays and then boxed every element into a MutableList<Double>.
 */
class OnnxModelRunner(
    private val context: Context,
    private val assetName: String
) : AutoCloseable {
    private val environment = OrtEnvironment.getEnvironment()
    private var session: OrtSession? = null
    private var inputName: String? = null
    private var outputName: String? = null
    private var inputShape: LongArray? = null
    private var inputBuffer: FloatBuffer? = null

    val isLoaded: Boolean
        get() = session != null

    data class ModelInfo(
        val assetName: String,
        val inputName: String,
        val inputShape: LongArray,
        val outputSummary: String
    )

    @Synchronized
    fun load(): Result<ModelInfo> = runCatching {
        val bytes = context.assets.open(assetName).use { it.readBytes() }
        val options = OrtSession.SessionOptions()
        options.use {
            it.setIntraOpNumThreads(Runtime.getRuntime().availableProcessors().coerceIn(2, 4))
            it.setOptimizationLevel(OrtSession.SessionOptions.OptLevel.ALL_OPT)
            session?.close()
            session = environment.createSession(bytes, it)
        }

        val s = requireNotNull(session)
        val firstInput = s.inputInfo.entries.firstOrNull()
            ?: error("Model has no inputs")
        inputName = firstInput.key
        val tensorInfo = firstInput.value.info as? TensorInfo
            ?: error("Input '${firstInput.key}' is not a tensor")
        inputShape = tensorInfo.shape
        require(tensorInfo.shape.contentEquals(longArrayOf(1L, 3L, 256L, 256L))) {
            "Free-space model input must be [1,3,256,256], got ${tensorInfo.shape.contentToString()}"
        }
        outputName = s.outputInfo.keys.firstOrNull() ?: error("Model has no outputs")
        inputBuffer = ByteBuffer
            .allocateDirect(256 * 256 * 3 * 4)
            .order(ByteOrder.nativeOrder())
            .asFloatBuffer()

        val outputSummary = s.outputInfo.entries.joinToString(" | ") { (name, node) ->
            val ti = node.info as? TensorInfo
            val shape = ti?.shape?.joinToString(prefix = "[", postfix = "]") ?: "unknown"
            "$name:$shape/${ti?.type ?: "unknown"}"
        }

        ModelInfo(assetName, firstInput.key, tensorInfo.shape, outputSummary)
    }

    /**
     * Runs the model and writes the per-pixel class ids (0=free, 1=obstacle, 2=unknown, 255=ignore)
     * for a [width] x [height] mask into [mask]. Returns nothing; throws on any shape/type problem so
     * the caller can drop to SAFE.
     */
    @Synchronized
    fun runSemantic(floats: FloatArray, width: Int, height: Int, mask: ByteArray) {
        val s = session ?: error("Model not loaded: $assetName")
        val name = inputName ?: error("Input name not initialized")
        val outName = outputName ?: error("Output name not initialized")
        val shape = inputShape ?: error("Input shape not initialized")
        val buffer = inputBuffer ?: error("Input buffer not initialized")
        val expected = shape.fold(1L) { acc, v ->
            require(v > 0) { "Dynamic input shape is not supported: ${shape.contentToString()}" }
            acc * v
        }.toInt()
        require(floats.size == expected) {
            "Input data size ${floats.size} does not match shape ${shape.contentToString()}"
        }

        buffer.clear()
        buffer.put(floats)
        buffer.rewind()

        // A fresh tensor per run (cheap: it wraps the direct buffer) keeps correctness independent of
        // whether a given ORT build copies or shares the buffer memory.
        OnnxTensor.createTensor(environment, buffer, shape).use { input ->
            s.run(mapOf(name to input), setOf(outName)).use { result ->
                val tensor = result.iterator().next().value as? OnnxTensor
                    ?: error("Free-space output '$outName' is not a tensor")
                SemanticMaskDecoder.decode(tensor, width, height, mask)
            }
        }
    }

    @Synchronized
    override fun close() {
        session?.close()
        session = null
        inputBuffer = null
    }
}

/** Converts the ArgMax (or equivalent 3-class score tensor) into 0..2/255 labels without boxing. */
object SemanticMaskDecoder {
    private const val CLASS_COUNT = 3

    fun decode(tensor: OnnxTensor, width: Int, height: Int, out: ByteArray) {
        require(width > 0 && height > 0) { "Invalid mask dimensions" }
        val pixels = width * height
        require(out.size >= pixels) { "Mask buffer ${out.size} is smaller than $pixels" }

        val info = tensor.info
        val shape = info.shape
        val total = info.numElements.toInt()

        when {
            total == pixels -> decodeLabels(tensor, info.type, pixels, out)
            total == pixels * CLASS_COUNT -> {
                require(info.type == OnnxJavaType.FLOAT) {
                    "Score output must be FLOAT, got ${info.type}"
                }
                decodeScores(tensor.floatBuffer, shape, pixels, out)
            }
            else -> error(
                "Unexpected semantic output shape=${shape.contentToString()} values=$total; " +
                    "expected $pixels or ${pixels * CLASS_COUNT}"
            )
        }
    }

    private fun decodeLabels(tensor: OnnxTensor, type: OnnxJavaType, pixels: Int, out: ByteArray) {
        when (type) {
            OnnxJavaType.INT64 -> {
                val b = tensor.longBuffer
                for (i in 0 until pixels) out[i] = classId(b.get(i))
            }
            OnnxJavaType.INT32 -> {
                val b = tensor.intBuffer
                for (i in 0 until pixels) out[i] = classId(b.get(i).toLong())
            }
            OnnxJavaType.INT16 -> {
                val b = tensor.shortBuffer
                for (i in 0 until pixels) out[i] = classId(b.get(i).toLong())
            }
            OnnxJavaType.INT8, OnnxJavaType.UINT8 -> {
                val b = tensor.byteBuffer
                // Signed read of 255 yields -1 for INT8 ignore labels, so normalise to 0..255 first.
                for (i in 0 until pixels) out[i] = classId((b.get(i).toInt() and 0xFF).toLong())
            }
            OnnxJavaType.FLOAT -> {
                val b = tensor.floatBuffer
                for (i in 0 until pixels) {
                    val v = b.get(i)
                    out[i] = if (v.isFinite()) classId(v.toLong()) else 255.toByte()
                }
            }
            OnnxJavaType.DOUBLE -> {
                val b = tensor.doubleBuffer
                for (i in 0 until pixels) {
                    val v = b.get(i)
                    out[i] = if (v.isFinite()) classId(v.toLong()) else 255.toByte()
                }
            }
            else -> error("Unsupported semantic label output type: $type")
        }
    }

    private fun decodeScores(b: FloatBuffer, shape: LongArray, pixels: Int, out: ByteArray) {
        val channelFirst = shape.size >= 3 && shape[shape.size - 3] == CLASS_COUNT.toLong()
        for (i in 0 until pixels) {
            var bestClass = 0
            var bestScore = Float.NEGATIVE_INFINITY
            for (c in 0 until CLASS_COUNT) {
                val idx = if (channelFirst) c * pixels + i else i * CLASS_COUNT + c
                val score = b.get(idx)
                if (score > bestScore) {
                    bestScore = score
                    bestClass = c
                }
            }
            out[i] = bestClass.toByte()
        }
    }

    private fun classId(value: Long): Byte = when (value) {
        0L, 1L, 2L -> value.toByte()
        else -> 255.toByte() // includes the 255 ignore label and anything unexpected
    }
}

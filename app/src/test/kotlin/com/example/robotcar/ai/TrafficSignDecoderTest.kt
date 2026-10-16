package com.example.robotcar.ai

import com.example.robotcar.vision.ModelInputPreprocessor
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import java.io.File

class TrafficSignDecoderTest {
    private val transform = ModelInputPreprocessor.LetterboxTransform(
        modelSize = 640, originalWidth = 640, originalHeight = 640,
        resizedWidth = 640, resizedHeight = 640, padLeft = 0, padTop = 0, scale = 1f
    )
    private val nc = TrafficSignDecoder.labels.size
    private val attrs = 4 + nc

    /** Builds a channel-first [1, attrs, n] tensor with one detection at candidate [at]. */
    private fun tensor(n: Int, at: Int, cx: Float, cy: Float, w: Float, h: Float, classId: Int, score: Float): TfliteModelRunner.Output {
        val v = FloatArray(attrs * n)
        v[0 * n + at] = cx; v[1 * n + at] = cy; v[2 * n + at] = w; v[3 * n + at] = h
        v[(4 + classId) * n + at] = score
        return TfliteModelRunner.Output(intArrayOf(1, attrs, n), v)
    }

    @Test fun modelHasSevenClassesAndTheRealOutputShape() {
        assertEquals(7, nc)
        assertEquals(11, attrs) // matches the packaged tflite output [1, 11, 8400]
    }

    @Test fun everyClassIdDecodesToItsLabelOnTheRealOutputShape() {
        for (classId in TrafficSignDecoder.labels.indices) {
            val decoded = TrafficSignDecoder.decode(tensor(8400, 4321, 320f, 320f, 160f, 160f, classId, 0.95f), transform)
            assertNotNull("class $classId should decode", decoded)
            assertEquals(TrafficSignDecoder.labels[classId], decoded!!.label)
        }
    }

    @Test fun normalizedBoxesFromTheTfliteExportAreHandled() {
        // The packaged model normalises xywh by 1/640 (Ultralytics _NormalizeCoords).
        val decoded = TrafficSignDecoder.decode(tensor(8400, 7, 0.25f, 0.5f, 0.2f, 0.2f, 5, 0.9f), transform)
        assertEquals("stop", decoded!!.label)
        assertEquals(0.25f, decoded.centerX, 0.01f)
    }

    @Test fun classOrderMatchesCheckpointMetadataAndStopIsClassFive() {
        assertEquals(
            listOf("compulsory_right", "no_stopping_or_parking", "right_turn_prohibited",
                "right_reverse_bend", "side_road_right", "stop", "turn_right"),
            TrafficSignDecoder.labels
        )
        assertEquals(5, TrafficSignDecoder.labels.indexOf("stop"))
        assertEquals(6, TrafficSignDecoder.labels.indexOf("turn_right"))
    }

    @Test fun labelAssetMatchesDecoderList() {
        val f = listOf("src/main/assets/models/traffic_sign_labels.txt", "app/src/main/assets/models/traffic_sign_labels.txt")
            .map(::File).firstOrNull { it.exists() } ?: return // not runnable from this working dir: skip
        assertEquals(TrafficSignDecoder.labels, f.readLines().map { it.trim() }.filter { it.isNotEmpty() })
    }

    @Test fun lowConfidenceAndOldClassCountShapesAreRejected() {
        assertNull(TrafficSignDecoder.decode(tensor(8400, 9, 320f, 320f, 160f, 160f, 5, 0.3f), transform))
        val old = TfliteModelRunner.Output(intArrayOf(1, 33, 4), FloatArray(33 * 4))
        val e = runCatching { TrafficSignDecoder.decode(old, transform) }.exceptionOrNull()
        assertTrue("a 29-class tensor must be refused, not mis-decoded", e != null)
    }

    @Test fun unsupportedBehaviorLabelsAreReportedSoDeadLogicIsVisible() {
        assertTrue(TrafficSignDecoder.unsupportedBehaviorLabels.contains("no_entry"))
        assertTrue(!TrafficSignDecoder.unsupportedBehaviorLabels.contains("stop"))
    }
}

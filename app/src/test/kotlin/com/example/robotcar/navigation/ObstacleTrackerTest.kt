package com.example.robotcar.navigation

import org.junit.Assert.assertEquals
import org.junit.Test

class ObstacleTrackerTest {
    private val w = 4
    private val h = 4
    private fun frame(cls: Int) = ByteArray(w * h) { cls.toByte() }

    @Test
    fun firstFreeFrameIsStillUnknownThenFree() {
        val t = ObstacleTracker()
        assertEquals(2, t.update(frame(0), w, h, 0L)[0].toInt())
        assertEquals(0, t.update(frame(0), w, h, 33L)[0].toInt())
    }

    @Test
    fun obstacleAcceptedImmediately() {
        val t = ObstacleTracker()
        t.update(frame(0), w, h, 0L); t.update(frame(0), w, h, 33L)
        assertEquals(1, t.update(frame(1), w, h, 66L)[0].toInt())
    }

    @Test
    fun obstacleHeldBrieflyThenFreeNeedsConfirmation() {
        val t = ObstacleTracker()
        t.update(frame(0), w, h, 0L); t.update(frame(0), w, h, 33L)
        t.update(frame(1), w, h, 66L)
        // single-frame dropout: still obstacle (score 1.0 -> 0.75)
        assertEquals(1, t.update(frame(0), w, h, 99L)[0].toInt())
        assertEquals(1, t.update(frame(0), w, h, 132L)[0].toInt())  // 0.56
        // 0.42 < 0.5 -> not obstacle; free streak is already 3 -> FREE
        assertEquals(0, t.update(frame(0), w, h, 165L)[0].toInt())
    }

    @Test
    fun flickerOfFreeNeverBecomesFree() {
        val t = ObstacleTracker()
        var now = 0L
        repeat(10) { i ->
            val out = t.update(frame(if (i % 2 == 0) 0 else 2), w, h, now)
            assertEquals(2, out[0].toInt())
            now += 33
        }
    }

    @Test
    fun resetAndLongGapRestartAsUnknown() {
        val t = ObstacleTracker()
        t.update(frame(0), w, h, 0L); t.update(frame(0), w, h, 33L)
        t.reset()
        assertEquals(2, t.update(frame(0), w, h, 66L)[0].toInt())
        t.update(frame(0), w, h, 99L)
        assertEquals(2, t.update(frame(0), w, h, 5_000L)[0].toInt())
    }
}

package com.example.robotcar.video

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class ClosedLoopSimTest {
    private fun sweep(realismFor: (Long) -> SimRealism, seeds: Int): Map<Int, List<ClosedLoopHarness.Result>> {
        val out = HashMap<Int, MutableList<ClosedLoopHarness.Result>>()
        for (seed in 1L..seeds) {
            for (seg in 0..8) {
                out.getOrPut(seg) { mutableListOf() } += ClosedLoopHarness(realismFor(seed)).runSegment(seg)
            }
        }
        return out
    }

    private fun report(title: String, r: Map<Int, List<ClosedLoopHarness.Result>>) {
        println("== $title")
        for ((seg, list) in r.toSortedMap()) {
            val col = list.count { it.collisions > 0 }
            val prog = list.map { it.forwardCm }.average()
            val stuck = list.count { !it.moved && seg in 0..6 }
            println("seg $seg: collision runs ${col}/${list.size}  mean fwd ${"%.1f".format(prog)} cm  never-moved $stuck")
        }
    }

    @Test
    fun idealProfileHasNoCollisions() {
        val r = sweep({ SimRealism.IDEAL }, 1)
        report("IDEAL", r)
        assertEquals(0, r.values.flatten().sumOf { it.collisions })
    }

    @Test
    fun realisticProfileMonteCarlo() {
        val r = sweep({ s -> SimRealism.REALISTIC.copy(seed = s) }, 25)
        report("REALISTIC x25 seeds", r)
        val total = r.values.flatten()
        val colRuns = total.count { it.collisions > 0 }
        println("TOTAL collision runs: $colRuns / ${total.size}")
        assertTrue("collision runs $colRuns/${total.size}", colRuns == 0)
    }

    /** Not an assertion: shows how much margin the controller has when the real chassis is faster/laggier than assumed. */
    @Test
    fun sensitivityPrint() {
        data class Case(val name: String, val speed: Float, val r: SimRealism)
        val base = SimRealism.REALISTIC
        val cases = listOf(
            Case("speed x1.0 lat 3", 18f, base),
            Case("speed x1.5 lat 3", 27f, base),
            Case("speed x2.0 lat 3", 36f, base),
            Case("speed x1.5 lat 5", 27f, base.copy(perceptionLatencyFrames = 5)),
            Case("speed x2.0 lat 5", 36f, base.copy(perceptionLatencyFrames = 5)),
            Case("speed x2.0 lat 2 sonar drop 15%", 36f, base.copy(perceptionLatencyFrames = 2, sonarDropoutProb = 0.15f))
        )
        for (c in cases) {
            var col = 0; var n = 0
            for (seed in 1L..10L) for (seg in 0..8) {
                val res = ClosedLoopHarness(c.r.copy(seed = seed), nominalSpeedCmS = c.speed).runSegment(seg)
                if (res.collisions > 0) col++; n++
            }
            println("SENS ${c.name}: collision runs $col/$n")
        }
    }
}

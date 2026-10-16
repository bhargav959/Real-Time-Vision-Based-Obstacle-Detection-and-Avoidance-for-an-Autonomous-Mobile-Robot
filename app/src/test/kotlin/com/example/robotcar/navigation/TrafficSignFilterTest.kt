package com.example.robotcar.navigation

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Test

class TrafficSignFilterTest {
    @Test
    fun signRequiresThreeMatchingFrames() {
        val filter = TrafficSignFilter(requiredFrames = 3)
        val sign = TrafficSign("stop", 0.9f)
        assertNull(filter.update(sign))
        assertNull(filter.update(sign))
        assertEquals("stop", filter.update(sign)?.label)
    }

    @Test
    fun missingFrameResetsConfirmation() {
        val filter = TrafficSignFilter(requiredFrames = 3)
        val sign = TrafficSign("stop", 0.9f)
        filter.update(sign)
        filter.update(sign)
        assertNull(filter.update(null))
        assertNull(filter.update(sign))
    }
}

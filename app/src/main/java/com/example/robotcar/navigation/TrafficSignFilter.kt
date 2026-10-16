package com.example.robotcar.navigation

/** Requires repeated agreement before exposing a sign to the behavior controller. */
class TrafficSignFilter(private val requiredFrames: Int = 3) {
    init {
        require(requiredFrames >= 1) { "requiredFrames must be >= 1" }
    }

    private var lastLabel: String? = null
    private var count = 0
    private var latest: TrafficSign? = null

    fun reset() {
        lastLabel = null
        count = 0
        latest = null
    }

    fun update(sign: TrafficSign?): TrafficSign? {
        if (sign == null || sign.confidence < 0.50f) {
            count = 0
            lastLabel = null
            latest = null
            return null
        }

        if (sign.label == lastLabel) {
            count++
        } else {
            lastLabel = sign.label
            count = 1
        }
        latest = sign
        return if (count >= requiredFrames) latest else null
    }
}

package com.example.robotcar.navigation

import kotlin.math.abs
import kotlin.math.ceil
import kotlin.math.floor
import kotlin.math.max
import kotlin.math.min

/**
 * Geometry-first local obstacle planner.
 *
 * The planner does not follow a visual lane. It treats the 10 cm side margin as a preferred
 * vehicle-centered corridor and searches for a collision-free local path. The path starts near
 * the current vehicle center and curves toward a selected lateral target at lookahead distance.
 *
 * Gap rule: usable FREE gap must meet config.requiredGapWidthCm. In the live/virtual app instance,
 * the 15 cm chassis width plus a 2 cm margin on each side produces a 19 cm effective requirement.
 * The separate 10 cm preferred boundary margin is not added to the passage-gap test.
 *
 * Unknown pixels are not free for collision geometry.
 *
 * Performance note: free intervals are precomputed once per planning row and reused across all
 * lateral candidates instead of rescanning the full row for every candidate.
 */
class ObstaclePlanner(
    private val config: NavigationGeometryConfig = NavigationGeometryConfig(),
    private val projector: CameraGroundProjector = CameraGroundProjector(config),
    private val occupancyThreshold: Int = 1,
    private val freeClass: Int = 0,
    private val lateralSamples: Int = 61
) {
    private data class FreeRun(val start: Int, val end: Int)

    private data class RowGeometry(
        val row: Int,
        val distanceCm: Float,
        val cmPerPixel: Float,
        val freeRuns: List<FreeRun>,
        val obstaclePixels: Int
    )

    private data class CandidateEval(
        val lateral: Float,
        val minimumGap: Float,
        val feasible: Boolean
    )

    // Retain both offset and side. Offset continuity prevents sudden steering changes; the side
    // lock prevents left/right flicker when segmentation is noisy and both routes remain feasible.
    private var previousLateralCm: Float? = null
    private var preferredAvoidanceSide: Int = 0 // -1 = left, +1 = right, 0 = uncommitted
    private var centerClearFrames: Int = 0

    private companion object {
        const val CENTER_CLEAR_FRAMES_TO_RELEASE_SIDE = 4
        const val SIDE_CHANGE_CLEARANCE_ADVANTAGE_CM = 5f
    }

    fun reset() {
        previousLateralCm = null
        preferredAvoidanceSide = 0
        centerClearFrames = 0
    }

    fun analyze(mask: ByteArray, width: Int, height: Int): ObstaclePlan {
        if (width <= 0 || height <= 0 || mask.size < width * height) return blockedPlan()

        val targetDistance = config.targetLookaheadCm.coerceIn(
            config.nearPlanningDistanceCm + 1f,
            config.maxPlanningDistanceCm
        )
        val rows = buildRowGeometry(mask, width, height, targetDistance)
        if (rows.isEmpty()) return blockedPlan()

        // Only explicit obstacle class pixels count as obstacle evidence. UNKNOWN is still not
        // drivable, but unknown pixels alone are not reported as a confirmed physical obstacle.
        val obstaclePixels = rows.sumOf { it.obstaclePixels }
        val nearestObstacle = nearestObstacleDistance(mask, width, height)
        val previousLateral = previousLateralCm
        val centerEval = evaluateCandidate(rows, width, height, 0f, targetDistance)

        // Release the preferred side only after four consecutive frames have a verified centre
        // corridor. A single flickering FREE frame must not reverse the steering decision.
        centerClearFrames = if (centerEval.feasible) centerClearFrames + 1 else 0
        // Do not release the side while the robot is still unwinding a lateral trajectory back
        // toward centre. This avoids treating the first few clear frames as permission to reverse.
        if (centerClearFrames >= CENTER_CLEAR_FRAMES_TO_RELEASE_SIDE &&
            (previousLateral == null || abs(previousLateral) <= 3.5f)) {
            preferredAvoidanceSide = 0
        }
        val sideLock = preferredAvoidanceSide

        // Search each side independently. This makes the choice explicit: if the centre is blocked,
        // the planner can retain a previously selected feasible side, otherwise it compares actual
        // geometric clearance and path cost instead of relying on loop-order ties alone.
        val nominalLeft = findBestCandidate(
            rows, width, height, targetDistance, config.nominalCenterOffsetCm, previousLateral, side = -1
        )
        val nominalRight = findBestCandidate(
            rows, width, height, targetDistance, config.nominalCenterOffsetCm, previousLateral, side = 1
        )
        val expandedLeft = findBestCandidate(
            rows, width, height, targetDistance, config.expandedCenterOffsetCm, previousLateral, side = -1
        )
        val expandedRight = findBestCandidate(
            rows, width, height, targetDistance, config.expandedCenterOffsetCm, previousLateral, side = 1
        )

        val nominal = chooseBetweenSides(nominalLeft, nominalRight, previousLateral, sideLock)
        val expanded = chooseBetweenSides(expandedLeft, expandedRight, previousLateral, sideLock)
        val preferredNominal = candidateForSide(sideLock, nominalLeft, nominalRight)
        val preferredExpanded = candidateForSide(sideLock, expandedLeft, expandedRight)

        // Safety-first selection: centre path wins when verified. When blocked, keep the committed
        // side if it still has a feasible path; otherwise use another verified side or stop.
        // A nominal path on the opposite side does NOT override a feasible expanded path on the
        // committed side, because that would cause avoidable left/right oscillation.
        val selected = when {
            centerEval.feasible && previousLateral != null && abs(previousLateral) > 4f -> {
                val releaseTarget = approach(previousLateral, 0f, maxStepCm = 4f)
                val releaseEval = evaluateCandidate(rows, width, height, releaseTarget, targetDistance)
                if (releaseEval.feasible) releaseEval else centerEval
            }
            centerEval.feasible -> centerEval
            preferredNominal.feasible -> preferredNominal
            preferredExpanded.feasible -> preferredExpanded
            nominal.feasible -> nominal
            expanded.feasible -> expanded
            else -> CandidateEval(0f, 0f, false)
        }

        if (selected.feasible) {
            previousLateralCm = selected.lateral
            if (!centerEval.feasible && abs(selected.lateral) > 0.5f) {
                preferredAvoidanceSide = if (selected.lateral < 0f) -1 else 1
                centerClearFrames = 0
            }
        }

        val obstacleDetected = obstaclePixels >= max(6, rows.size) ||
            (nearestObstacle != null && nearestObstacle <= targetDistance)
        val crosses = abs(selected.lateral) > config.nominalCenterOffsetCm + 0.5f
        // If centre is blocked, even a small-offset feasible trajectory is still an avoidance path.
        // The controller must follow its left/right sign rather than accidentally issuing FORWARD.
        val pathType = when {
            !selected.feasible -> PathType.STOP
            !centerEval.feasible || abs(selected.lateral) >= 3.5f -> PathType.AVOIDANCE_CURVE
            else -> PathType.STRAIGHT
        }
        val boundary = when {
            !selected.feasible -> BoundaryState.BLOCKED
            crosses -> BoundaryState.CROSSING
            else -> BoundaryState.NOMINAL
        }
        val heading = projector.headingAngleDeg(selected.lateral, targetDistance)
        val candidate = if (selected.feasible) {
            PathCandidate(
                lateralOffsetCm = selected.lateral,
                targetDistanceCm = targetDistance,
                minimumGapWidthCm = selected.minimumGap,
                headingAngleDeg = heading,
                crossesBoundary = crosses,
                feasible = true
            )
        } else null

        return ObstaclePlan(
            obstacleDetected = obstacleDetected,
            obstacleCountEstimate = estimateObstacleCount(rows),
            nearestObstacleCm = nearestObstacle,
            centerPathFeasible = centerEval.feasible,
            nominalPathFeasible = nominalLeft.feasible || nominalRight.feasible,
            expandedPathFeasible = expandedLeft.feasible || expandedRight.feasible,
            bestPath = candidate,
            centerGapWidthCm = centerEval.minimumGap,
            requiredGapWidthCm = config.requiredGapWidthCm,
            nominalBoundaryHalfWidthCm = config.nominalBoundaryHalfWidthCm,
            expandedBoundaryHalfWidthCm = config.expandedBoundaryHalfWidthCm,
            nominalCenterOffsetCm = config.nominalCenterOffsetCm,
            expandedCenterOffsetCm = config.expandedCenterOffsetCm,
            boundaryState = boundary,
            pathType = pathType
        )
    }

    private fun candidateForSide(side: Int, left: CandidateEval, right: CandidateEval): CandidateEval = when {
        side < 0 -> left
        side > 0 -> right
        else -> CandidateEval(0f, 0f, false)
    }

    private fun chooseBetweenSides(
        left: CandidateEval,
        right: CandidateEval,
        previousLateralCm: Float?,
        preferredSide: Int
    ): CandidateEval {
        if (preferredSide < 0 && left.feasible) return left
        if (preferredSide > 0 && right.feasible) return right
        if (!left.feasible) return right
        if (!right.feasible) return left

        // When no side has been committed, favour a meaningfully wider verified gap. For similar
        // clearances, choose the lower-cost curve; exact ties deterministically choose left.
        if (left.minimumGap.isFinite() && right.minimumGap.isFinite()) {
            val gapDifference = left.minimumGap - right.minimumGap
            if (abs(gapDifference) >= SIDE_CHANGE_CLEARANCE_ADVANTAGE_CM) {
                return if (gapDifference > 0f) left else right
            }
        }
        val leftCost = candidateCost(left, previousLateralCm)
        val rightCost = candidateCost(right, previousLateralCm)
        return if (leftCost <= rightCost) left else right
    }

    private fun candidateCost(candidate: CandidateEval, previousLateralCm: Float?): Float {
        if (!candidate.feasible) return Float.MAX_VALUE
        val heading = abs(projector.headingAngleDeg(candidate.lateral, config.targetLookaheadCm))
        val continuityPenalty = previousLateralCm?.let {
            abs(candidate.lateral - it) * 0.85f
        } ?: 0f
        val clearanceReward = if (candidate.minimumGap.isFinite()) {
            (candidate.minimumGap - config.requiredGapWidthCm).coerceIn(0f, 20f) * 0.20f
        } else 0f
        return abs(candidate.lateral) * 1.30f + heading * 0.35f + continuityPenalty - clearanceReward
    }

    private fun buildRowGeometry(
        mask: ByteArray,
        width: Int,
        height: Int,
        targetDistanceCm: Float
    ): List<RowGeometry> {
        val rows = ArrayList<RowGeometry>()
        var distance = config.nearPlanningDistanceCm
        while (distance <= targetDistanceCm + 0.1f) {
            // Planner distances are measured from the robot's pivot centre; the projector measures
            // from the camera, which sits cameraForwardOffsetCm ahead of it.
            val row = projector.rowForDistanceCm(distance - config.cameraForwardOffsetCm, height)
            if (row != null && rows.lastOrNull()?.row != row) {
                val cmPerPixel = projector.cmPerPixelAtRow(row, width, height)
                val runs = if (cmPerPixel != null) freeRuns(mask, width, row) else emptyList()
                if (cmPerPixel != null) {
                    rows += RowGeometry(
                        row,
                        (projector.groundDistanceCmAtRow(row, height)?.plus(config.cameraForwardOffsetCm)) ?: distance,
                        cmPerPixel,
                        runs,
                        obstaclePixels(mask, width, row)
                    )
                }
            }
            distance += config.planningStepCm
        }
        if (rows.isEmpty()) {
            projector.rowForDistanceCm(targetDistanceCm - config.cameraForwardOffsetCm, height)?.let { row ->
                val cmPerPixel = projector.cmPerPixelAtRow(row, width, height)
                if (cmPerPixel != null) {
                    rows += RowGeometry(
                        row,
                        (projector.groundDistanceCmAtRow(row, height)?.plus(config.cameraForwardOffsetCm)) ?: targetDistanceCm,
                        cmPerPixel,
                        freeRuns(mask, width, row),
                        obstaclePixels(mask, width, row)
                    )
                }
            }
        }
        return rows
    }

    private fun obstaclePixels(mask: ByteArray, width: Int, row: Int): Int {
        val base = row * width
        var count = 0
        for (x in 0 until width) {
            if ((mask[base + x].toInt() and 0xFF) == occupancyThreshold) count++
        }
        return count
    }

    private fun freeRuns(mask: ByteArray, width: Int, row: Int): List<FreeRun> {
        val runs = ArrayList<FreeRun>()
        var start = -1
        val base = row * width
        for (x in 0 until width) {
            val free = (mask[base + x].toInt() and 0xFF) == freeClass
            if (free && start < 0) {
                start = x
            } else if (!free && start >= 0) {
                runs += FreeRun(start, x - 1)
                start = -1
            }
        }
        if (start >= 0) runs += FreeRun(start, width - 1)
        return runs
    }

    private fun findBestCandidate(
        rows: List<RowGeometry>,
        width: Int,
        height: Int,
        targetDistanceCm: Float,
        maxCenterOffsetCm: Float,
        previousLateralCm: Float?,
        side: Int
    ): CandidateEval {
        var best = CandidateEval(0f, 0f, false)
        var bestCost = Float.MAX_VALUE

        for (i in 0 until lateralSamples) {
            val normalized = if (lateralSamples == 1) 0f else i.toFloat() / (lateralSamples - 1)
            val targetLateral = (normalized * 2f - 1f) * maxCenterOffsetCm
            // Lateral sign follows image/ground coordinates: negative is LEFT, positive is RIGHT.
            if (side < 0 && targetLateral >= -0.01f) continue
            if (side > 0 && targetLateral <= 0.01f) continue
            val eval = evaluateCandidate(rows, width, height, targetLateral, targetDistanceCm)
            if (!eval.feasible) continue

            val heading = abs(projector.headingAngleDeg(targetLateral, targetDistanceCm))
            // Clearance is a safety constraint first, then only a small preference. The previous
            // inverse-clearance score over-rewarded very wide gaps and selected unnecessarily large
            // lateral excursions even when a milder curve was already feasible.
            val clearanceDeficit = if (eval.minimumGap.isFinite()) {
                (config.requiredGapWidthCm + 10f - eval.minimumGap).coerceIn(0f, 10f)
            } else 0f
            val clearancePenalty = clearanceDeficit * 0.25f
            val lateralCost = abs(targetLateral) * 1.30f
            val headingCost = heading * 0.35f
            val continuityPenalty = previousLateralCm?.let {
                abs(targetLateral - it) * 0.85f
            } ?: 0f
            val clearanceReward = if (eval.minimumGap.isFinite()) {
                (eval.minimumGap - config.requiredGapWidthCm).coerceIn(0f, 20f) * 0.20f
            } else 0f
            val cost = lateralCost + headingCost + clearancePenalty + continuityPenalty - clearanceReward
            if (cost < bestCost) {
                bestCost = cost
                best = eval
            }
        }
        return best
    }

    /**
     * Evaluate the actual smooth local trajectory through every sampled depth row.
     *
     * Per row: the (width + clearance) cross-section must lie inside one free run. In addition the
     * four corners of the robot rectangle, yawed to the path's local heading, are projected to
     * the depth row they actually reach, so the robot LENGTH is checked too: a long body swinging
     * through a bend cannot clip an obstacle that a width-only check would miss.
     */
    private fun evaluateCandidate(
        rows: List<RowGeometry>,
        width: Int,
        height: Int,
        targetLateralCm: Float,
        targetDistanceCm: Float
    ): CandidateEval {
        val required = config.requiredGapWidthCm
        // The trajectory begins at the vehicle origin (distance zero), not at the first visible
        // ground pixel. Rows below the camera horizon are simply not checked, but their path
        // progress must not be renormalised to zero. This keeps the curve slope consistent with
        // the reported bearing angle and avoids exaggerated turns near the camera.
        val span = targetDistanceCm.coerceAtLeast(1f)
        var minimumGap = Float.POSITIVE_INFINITY
        for (row in rows) {
            val progress = (row.distanceCm / span).coerceIn(0f, 1f)
            val smooth = TrajectoryProfile.lateralFraction(progress)
            val centerLateral = targetLateralCm * smooth
            // d(lateral)/d(distance) of the shared trajectory from the vehicle origin.
            val slope = targetLateralCm * TrajectoryProfile.derivative(progress) / span
            val yawRad = kotlin.math.atan(slope.toDouble()).toFloat()

            val centerX = projector.xForLateralCm(row.row, centerLateral, width, height) ?: return CandidateEval(targetLateralCm, 0f, false)
            val run = row.freeRuns.firstOrNull { centerX >= it.start && centerX <= it.end }
                ?: return CandidateEval(targetLateralCm, 0f, false)

            val gapPx = (run.end - run.start + 1).toFloat()
            val measuredGap = gapPx * row.cmPerPixel
            val gap = if (measuredGap < required &&
                measuredGap + config.geometryRasterToleranceCm >= required) {
                required
            } else measuredGap

            if (gap + config.geometryRasterToleranceCm < required) {
                return CandidateEval(targetLateralCm, min(minimumGap, gap), false)
            }
            minimumGap = min(minimumGap, gap)

            if (!robotFootprintFits(row, centerX, width) ||
                !cornersFit(rows, width, height, row.distanceCm, centerLateral, yawRad)) {
                return CandidateEval(targetLateralCm, minimumGap, false)
            }
        }
        return CandidateEval(targetLateralCm, minimumGap, true)
    }

    private fun robotFootprintFits(row: RowGeometry, centerX: Float, width: Int): Boolean {
        val halfWidthPixels = (config.effectiveRobotWidthCm * 0.5f) / row.cmPerPixel
        val leftX = centerX - halfWidthPixels
        val rightX = centerX + halfWidthPixels
        val minX = ceil(leftX - 0.5f).toInt()
        val maxX = floor(rightX - 0.5f).toInt()
        if (minX < 0 || maxX >= width || minX > maxX) return false
        val run = row.freeRuns.firstOrNull { minX >= it.start && maxX <= it.end } ?: return false
        return minX >= run.start && maxX <= run.end
    }

    /**
     * Robot rectangle corners (footprint centred at distance [dc], lateral [c], heading [yawRad],
     * +yaw = toward +lateral) checked against the free runs of the planned row nearest to where
     * each corner lands. Corners outside the planned depth range are not observable and skipped
     * (they are covered by the ultrasonic backstop / earlier rows).
     */
    private fun cornersFit(
        rows: List<RowGeometry>,
        width: Int,
        height: Int,
        dc: Float,
        c: Float,
        yawRad: Float
    ): Boolean {
        val first = rows.first().distanceCm
        val last = rows.last().distanceCm
        val hw = config.effectiveRobotWidthCm * 0.5f
        val hl = config.robotLengthCm * 0.5f
        val sinY = kotlin.math.sin(yawRad)
        val cosY = kotlin.math.cos(yawRad)
        val tol = config.geometryRasterToleranceCm
        for (sy in floatArrayOf(-hl, hl)) {
            for (sx in floatArrayOf(-hw, hw)) {
                val dist = dc + sy * cosY - sx * sinY
                if (dist < first || dist > last) continue
                // Pull the corner toward the centreline by the raster tolerance (pixel quantisation).
                val lat = c + sy * sinY + sx * cosY - (if (sx > 0f) tol else -tol)
                var nearest = rows[0]
                var best = Float.MAX_VALUE
                for (r in rows) {
                    val e = abs(r.distanceCm - dist)
                    if (e < best) { best = e; nearest = r }
                }
                val x = projector.xForLateralCm(nearest.row, lat, width, height) ?: return false
                val xi = floor(x).toInt()
                if (xi < 0 || xi >= width) return false
                if (nearest.freeRuns.none { xi >= it.start && xi <= it.end }) return false
            }
        }
        return true
    }

    /**
     * Nearest obstacle INSIDE the expanded corridor (obstacles beside it do not slow the robot),
     * ignoring isolated pixels (needs config.minObstaclePixelsPerRow on the row).
     */
    private fun nearestObstacleDistance(mask: ByteArray, width: Int, height: Int): Float? {
        val limitCm = config.expandedBoundaryHalfWidthCm
        for (row in height - 1 downTo 0) {
            val distance = (projector.groundDistanceCmAtRow(row, height) ?: continue) + config.cameraForwardOffsetCm
            if (distance > config.maxPlanningDistanceCm) continue
            val xl = projector.xForLateralCm(row, -limitCm, width, height) ?: continue
            val xr = projector.xForLateralCm(row, limitCm, width, height) ?: continue
            val from = floor(xl).toInt().coerceIn(0, width - 1)
            val to = ceil(xr).toInt().coerceIn(0, width - 1)
            val base = row * width
            var count = 0
            for (x in from..to) {
                if ((mask[base + x].toInt() and 0xFF) == occupancyThreshold) {
                    count++
                    if (count >= config.minObstaclePixelsPerRow) return distance
                }
            }
        }
        return null
    }

    private fun estimateObstacleCount(rows: List<RowGeometry>): Int {
        if (rows.isEmpty()) return 0
        val sampleRows = rows.filterIndexed { index, _ -> index % max(1, rows.size / 3) == 0 }
        if (sampleRows.isEmpty()) return 0
        var total = 0
        for (row in sampleRows) {
            val runs = row.freeRuns
            // Each transition from FREE to non-FREE after a free interval is an approximate obstacle.
            total += if (runs.isEmpty()) 1 else max(0, runs.size - 1)
        }
        return (total / sampleRows.size).coerceAtLeast(0)
    }

    private fun approach(current: Float, target: Float, maxStepCm: Float): Float {
        val delta = target - current
        return if (abs(delta) <= maxStepCm) target else current + kotlin.math.sign(delta) * maxStepCm
    }

    private fun blockedPlan(): ObstaclePlan = ObstaclePlan(
        obstacleDetected = true,
        obstacleCountEstimate = 0,
        nearestObstacleCm = 0f,
        centerPathFeasible = false,
        nominalPathFeasible = false,
        expandedPathFeasible = false,
        bestPath = null,
        centerGapWidthCm = 0f,
        requiredGapWidthCm = config.requiredGapWidthCm,
        nominalBoundaryHalfWidthCm = config.nominalBoundaryHalfWidthCm,
        expandedBoundaryHalfWidthCm = config.expandedBoundaryHalfWidthCm,
        nominalCenterOffsetCm = config.nominalCenterOffsetCm,
        expandedCenterOffsetCm = config.expandedCenterOffsetCm,
        boundaryState = BoundaryState.BLOCKED,
        pathType = PathType.STOP
    )
}

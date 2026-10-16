package com.example.robotcar.navigation

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class ObstaclePlannerTest {
    private val config = NavigationGeometryConfig(
        robotWidthCm = 15f,
        robotLengthCm = 25.5f,
        nominalBoundaryMarginCm = 10f,
        temporaryBoundaryExtensionCm = 20f,
        cameraHeightCm = 10f,
        cameraTiltDeg = 7.5f,
        targetLookaheadCm = 65f
    )
    private val projector = CameraGroundProjector(config)
    private val planner = ObstaclePlanner(config)

    private fun groundMask(vararg boxes: Box): ByteArray = ByteArray(256 * 256) { i ->
        val y = i / 256
        val x = i % 256
        val distance = projector.groundDistanceCmAtRow(y, 256)
        if (distance == null) 0 else {
            val lateral = projector.lateralCmAt(y, x + 0.5f, 256, 256) ?: 0f
            if (boxes.any { b -> distance in b.near..b.far && lateral in b.left..b.right }) 1 else 0
        }
    }

    private data class Box(val left: Float, val right: Float, val near: Float, val far: Float)

    @Test
    fun openScenePrefersStraight() {
        val p = planner.analyze(ByteArray(256 * 256), 256, 256)
        assertTrue(p.centerPathFeasible)
        assertEquals(0f, p.bestPath?.lateralOffsetCm)
        assertFalse(p.boundaryState == BoundaryState.CROSSING)
        assertEquals(PathType.STRAIGHT, p.pathType)
    }

    @Test
    fun exact15CmGapIsPassableWithoutBoundaryMargin() {
        val p = planner.analyze(
            groundMask(
                Box(-12f, -7.5f, 50f, 85f),
                Box(7.5f, 12f, 50f, 85f)
            ),
            256, 256
        )
        assertTrue(p.obstacleDetected)
        assertTrue(p.centerPathFeasible)
        assertTrue(p.centerGapWidthCm >= 15f)
        assertFalse(p.boundaryState == BoundaryState.CROSSING)
    }

    @Test
    fun nearNarrowGapIsRejectedWhenTheFullBodyTrajectoryCannotBeVerified() {
        // Although the visible opening between the two blocks looks usable, the robot's 25.5 cm
        // body and the current 65 cm trajectory do not fit a verified bend around these near
        // obstacles. The conservative response is STOP + active scan, not a guessed side path.
        val p = planner.analyze(
            groundMask(
                Box(-11f, -6f, 50f, 85f),
                Box(6f, 11f, 50f, 85f)
            ),
            256, 256
        )
        assertTrue(p.obstacleDetected)
        assertFalse(p.centerPathFeasible)
        assertFalse(p.nominalPathFeasible)
        assertFalse(p.expandedPathFeasible)
        assertEquals(BoundaryState.BLOCKED, p.boundaryState)
        assertTrue(p.bestPath == null)
    }

    @Test
    fun chosenAvoidanceSideDoesNotFlipOnOneFrameClearanceChange() {
        val planner = ObstaclePlanner(config)
        val baseObstacle = Box(-3f, 3f, 50f, 65f)
        val first = planner.analyze(groundMask(baseObstacle), 256, 256)
        assertTrue((first.bestPath?.lateralOffsetCm ?: 0f) > 0f) // first route is to the right

        // A new edge obstacle makes the left side look better to a fresh planner, but the right
        // trajectory is still verified and should be retained to avoid a flickering steering flip.
        val changed = groundMask(baseObstacle, Box(22f, 24f, 48f, 68f))
        val sequential = planner.analyze(changed, 256, 256)
        val fresh = ObstaclePlanner(config).analyze(changed, 256, 256)
        assertTrue((sequential.bestPath?.lateralOffsetCm ?: 0f) > 0f)
        assertTrue((fresh.bestPath?.lateralOffsetCm ?: 0f) < 0f)
    }

    @Test
    fun unknownRegionIsNotTreatedAsPassableGap() {
        val unknown = ByteArray(256 * 256) { 2 }
        val p = planner.analyze(unknown, 256, 256)
        assertFalse(p.centerPathFeasible)
        assertFalse(p.expandedPathFeasible)
        assertEquals(PathType.STOP, p.pathType)
    }

    @Test
    fun noRouteStops() {
        val p = planner.analyze(groundMask(Box(-20f, 20f, 50f, 85f)), 256, 256)
        assertTrue(p.obstacleDetected)
        assertFalse(p.expandedPathFeasible)
        assertEquals(BoundaryState.BLOCKED, p.boundaryState)
        assertTrue(p.bestPath == null)
    }

    @Test
    fun clearanceMarginRejectsZeroClearanceGapButAcceptsWiderOne() {
        val margin = ObstaclePlanner(config.copy(gapClearanceMarginCm = 2f), projector)
        val exact = margin.analyze(
            groundMask(Box(-12f, -7.5f, 50f, 85f), Box(7.5f, 12f, 50f, 85f)), 256, 256
        )
        assertFalse(exact.centerPathFeasible)           // 15 cm gap, needs 19 cm
        assertEquals(19f, exact.requiredGapWidthCm)
        val wide = margin.analyze(
            groundMask(Box(-14f, -10.5f, 50f, 85f), Box(10.5f, 14f, 50f, 85f)), 256, 256
        )
        assertTrue(wide.centerPathFeasible)             // 21 cm gap
    }

    @Test
    fun nearestObstacleIgnoresObjectsBesideTheCorridor() {
        val beside = planner.analyze(groundMask(Box(60f, 80f, 30f, 40f)), 256, 256)
        assertTrue(beside.nearestObstacleCm == null)
        val inside = planner.analyze(groundMask(Box(-5f, 5f, 30f, 40f)), 256, 256)
        assertTrue((inside.nearestObstacleCm ?: 999f) < 45f)
    }

    @Test
    fun nearestObstacleIgnoresSingleNoisePixel() {
        val mask = ByteArray(256 * 256)
        mask[200 * 256 + 128] = 1
        assertTrue(planner.analyze(mask, 256, 256).nearestObstacleCm == null)
    }

    @Test
    fun projectionRoundTripsLateralPosition() {
        for (row in listOf(150, 200, 250)) {
            val x = projector.xForLateralCm(row, 12f, 256, 256)!!
            assertEquals(12f, projector.lateralCmAt(row, x, 256, 256)!!, 0.01f)
        }
    }

    @Test
    fun groundDistanceDecreasesDownTheImage() {
        val near = projector.groundDistanceCmAtRow(255, 256)!!
        val far = projector.groundDistanceCmAtRow(150, 256)!!
        assertTrue(near < far)
        assertTrue(near in 10f..25f)
    }
}

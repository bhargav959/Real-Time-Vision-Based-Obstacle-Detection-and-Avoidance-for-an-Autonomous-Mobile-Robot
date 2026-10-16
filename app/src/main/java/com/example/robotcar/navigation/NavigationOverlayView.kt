package com.example.robotcar.navigation

import android.content.Context
import android.graphics.Canvas
import android.graphics.Paint
import android.graphics.Path
import android.graphics.DashPathEffect
import android.util.AttributeSet
import android.view.View
import kotlin.math.PI
import kotlin.math.atan
import kotlin.math.tan

/** Visualizes the same physical corridor and smooth local trajectory used by the planner. */
class NavigationOverlayView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null
) : View(context, attrs) {
    private val config = NavigationGeometryConfig()
    private var state: ObstaclePlan? = null
    private var targetLateralCm = 0f
    private var scanCommand: MotorCommand? = null
    private var pathAuthorized = true

    private val nominalPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 4f
        color = 0xFFFFFFFF.toInt()
    }
    private val expandedPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 3f
        color = 0xFFFFC107.toInt()
    }
    private val pathPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 7f
        color = 0xFF00E676.toInt()
    }
    private val scanPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 6f
        color = 0xFFFFB300.toInt()
        pathEffect = DashPathEffect(floatArrayOf(14f, 9f), 0f)
    }
    private val labelPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
        textSize = 15f * resources.displayMetrics.scaledDensity
        typeface = android.graphics.Typeface.create("sans-serif-medium", android.graphics.Typeface.BOLD)
        setShadowLayer(3f, 1f, 1f, 0xCC000000.toInt())
    }

    /**
     * A green trajectory means the planner has a feasible path candidate. During active scanning,
     * the amber dashed cue shows only the direction being probed; it is deliberately not green
     * because a scan direction has not yet been verified as drivable.
     */
    fun update(
        plan: ObstaclePlan?,
        lateralTargetCm: Float = 0f,
        scanCommand: MotorCommand? = null,
        pathAuthorized: Boolean = true
    ) {
        state = plan
        targetLateralCm = lateralTargetCm
        this.scanCommand = scanCommand
        this.pathAuthorized = pathAuthorized
        invalidate()
    }

    fun clear() {
        state = null
        targetLateralCm = 0f
        scanCommand = null
        pathAuthorized = false
        invalidate()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        val plan = state
        if (plan != null) {
            drawBoundary(canvas, plan.nominalBoundaryHalfWidthCm, nominalPaint)
            drawBoundary(canvas, plan.expandedBoundaryHalfWidthCm, expandedPaint)
        }

        when (scanCommand) {
            MotorCommand.PIVOT_LEFT -> {
                drawTrajectory(canvas, -18f, scanPaint)
                drawLabel(canvas, "SCANNING LEFT • PATH NOT VERIFIED", scanPaint.color)
            }
            MotorCommand.PIVOT_RIGHT -> {
                drawTrajectory(canvas, 18f, scanPaint)
                drawLabel(canvas, "SCANNING RIGHT • PATH NOT VERIFIED", scanPaint.color)
            }
            MotorCommand.BACKOFF -> drawLabel(canvas, "REVERSE BACKOFF • REAR NOT SENSED", scanPaint.color)
            MotorCommand.STOP -> drawLabel(canvas, "SCAN WAITING • NO VERIFIED PATH", 0xFFFF5252.toInt())
            else -> {
                if (pathAuthorized && plan?.bestPath?.feasible == true) {
                    drawTrajectory(canvas, targetLateralCm, pathPaint)
                } else {
                    val message = when {
                        !pathAuthorized -> "SAFETY GATE ACTIVE • DRIVE INHIBITED"
                        plan == null -> "NO PERCEPTION DATA"
                        else -> "NO VERIFIED PATH"
                    }
                    drawLabel(canvas, message, 0xFFFF5252.toInt())
                }
            }
        }
    }

    private fun drawLabel(canvas: Canvas, text: String, color: Int) {
        labelPaint.color = color
        canvas.drawText(text, 12f, 24f * resources.displayMetrics.scaledDensity, labelPaint)
    }

    private fun drawBoundary(canvas: Canvas, halfWidthCm: Float, paint: Paint) {
        val nearDistance = 20f
        val farDistance = config.maxPlanningDistanceCm
        val nearY = yForDistance(nearDistance)
        val farY = yForDistance(farDistance)
        canvas.drawLine(xFor(nearDistance, -halfWidthCm), nearY, xFor(farDistance, -halfWidthCm), farY, paint)
        canvas.drawLine(xFor(nearDistance, halfWidthCm), nearY, xFor(farDistance, halfWidthCm), farY, paint)
    }

    private fun drawTrajectory(canvas: Canvas, targetLateralCm: Float, paint: Paint) {
        val startDistance = config.nearPlanningDistanceCm
        val targetDistance = config.targetLookaheadCm
        val path = Path()
        var first = true
        var distance = startDistance
        while (distance <= targetDistance + 0.1f) {
            val progress = (distance / targetDistance).coerceIn(0f, 1f)
            val lateral = targetLateralCm * TrajectoryProfile.lateralFraction(progress)
            val x = xFor(distance, lateral)
            val y = yForDistance(distance)
            if (first) {
                path.moveTo(x, y)
                first = false
            } else {
                path.lineTo(x, y)
            }
            distance += 4f
        }
        canvas.drawPath(path, paint)
    }

    private fun yForDistance(distanceCm: Float): Float {
        if (distanceCm <= 0f) return height.toFloat()
        val downAngleDeg = Math.toDegrees(atan((config.cameraHeightCm / distanceCm).toDouble())).toFloat()
        val normalized = 0.5f + (downAngleDeg - config.cameraTiltDeg) / config.verticalFovDeg
        return (normalized * height).coerceIn(0f, height.toFloat())
    }

    private fun xFor(distanceCm: Float, lateralCm: Float): Float {
        val fov = config.horizontalFovDeg * PI.toFloat() / 180f
        val visibleWidthCm = 2f * distanceCm * tan((fov / 2f).toDouble()).toFloat()
        return ((0.5f + lateralCm / visibleWidthCm) * width)
            .coerceIn(-width.toFloat(), 2f * width.toFloat())
    }
}

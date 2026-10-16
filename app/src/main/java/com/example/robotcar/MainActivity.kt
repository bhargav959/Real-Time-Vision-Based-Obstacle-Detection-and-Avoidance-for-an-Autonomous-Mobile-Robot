package com.example.robotcar

import android.Manifest
import android.content.pm.PackageManager
import android.graphics.Bitmap
import android.os.Bundle
import android.os.SystemClock
import android.util.Size
import android.view.View
import android.widget.Button
import android.widget.ImageView
import android.widget.TextView
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.camera.core.CameraSelector
import androidx.camera.core.ImageAnalysis
import androidx.camera.core.Preview
import androidx.camera.core.resolutionselector.ResolutionSelector
import androidx.camera.core.resolutionselector.ResolutionStrategy
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.camera.view.PreviewView
import androidx.core.content.ContextCompat
import com.example.robotcar.ai.ModelRegistry
import com.example.robotcar.ai.TrafficSignDecoder
import com.example.robotcar.camera.CameraAnalyzer
import com.example.robotcar.communication.SerialProtocol
import com.example.robotcar.communication.UsbSerialManager
import com.example.robotcar.logging.DecisionLog
import com.example.robotcar.navigation.CameraGroundProjector
import com.example.robotcar.navigation.ActivePathDiscovery
import com.example.robotcar.navigation.SafetyConstants
import com.example.robotcar.navigation.BoundaryState
import com.example.robotcar.navigation.FreeSpaceAnalyzer
import com.example.robotcar.navigation.MotorCommand
import com.example.robotcar.navigation.MotorCalibration
import com.example.robotcar.navigation.NavigationGeometryConfig
import com.example.robotcar.navigation.NavigationOverlayView
import com.example.robotcar.navigation.LiveDecisionStabilizer
import com.example.robotcar.navigation.NavigationController
import com.example.robotcar.navigation.ObstaclePlan
import com.example.robotcar.navigation.ObstaclePlanner
import com.example.robotcar.navigation.ObstacleTracker
import com.example.robotcar.navigation.PerceptionSnapshot
import com.example.robotcar.navigation.PathType
import com.example.robotcar.navigation.RobotMode
import com.example.robotcar.navigation.RotationFeedback
import com.example.robotcar.navigation.YawTracker
import com.example.robotcar.sensors.YawSensor
import com.example.robotcar.navigation.SpeedController
import com.example.robotcar.navigation.TrafficSignFilter
import com.example.robotcar.vision.ModelInputPreprocessor
import com.example.robotcar.video.SimRealism
import com.example.robotcar.video.VideoTestRunner
import com.example.robotcar.video.VirtualScenarioProvider
import java.util.Locale
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledExecutorService
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean

class MainActivity : AppCompatActivity() {
    private lateinit var previewView: PreviewView
    private lateinit var videoImageView: ImageView
    private lateinit var navigationOverlay: NavigationOverlayView
    private lateinit var statusText: TextView
    private lateinit var telemetryText: TextView
    private lateinit var connectUsbButton: Button
    private lateinit var armButton: Button
    private lateinit var stopButton: Button
    private lateinit var videoTestButton: Button
    private lateinit var videoPauseButton: Button
    private lateinit var videoStepButton: Button
    private lateinit var videoResetButton: Button

    private lateinit var cameraExecutor: ExecutorService
    private lateinit var modelExecutor: ExecutorService
    private lateinit var ioExecutor: ExecutorService
    private lateinit var controlExecutor: ScheduledExecutorService
    private lateinit var usb: UsbSerialManager
    private lateinit var models: ModelRegistry

    private var cameraProvider: ProcessCameraProvider? = null
    private var videoTestRunner: VideoTestRunner? = null

    private val geometryConfig = NavigationGeometryConfig(
        robotWidthCm = 15f,
        robotLengthCm = 25.5f,
        nominalBoundaryMarginCm = 10f,
        temporaryBoundaryExtensionCm = 20f,
        cameraHeightCm = 10f,
        cameraTiltDeg = 7.5f,
        targetLookaheadCm = 75f,
        // MEASURE on the chassis (see docs/SIM_REALITY_CALIBRATION_v0.5.11.md):
        //  cameraForwardOffsetCm = how far the phone camera is AHEAD (+) / BEHIND (-) of the pivot centre
        //  sonarForwardOffsetCm  = how far the HC-SR04 face is ahead of the pivot centre
        cameraForwardOffsetCm = 0f,
        sonarForwardOffsetCm = 12.75f
    )
    private val projector = CameraGroundProjector(geometryConfig)
    /** One conservative gap margin is shared by live and virtual planning: a 15 cm body needs 19 cm. */
    private val physicalGeometryConfig = geometryConfig.copy(gapClearanceMarginCm = LIVE_CLEARANCE_MARGIN_CM)
    private val livePlanner = ObstaclePlanner(physicalGeometryConfig, projector)
    private val virtualPlanner = ObstaclePlanner(physicalGeometryConfig, projector)
    private val obstacleTracker = ObstacleTracker()
    // Keep the same temporal occupancy rules in virtual and live mode; only the source of the mask differs.
    private val virtualObstacleTracker = ObstacleTracker()
    // The virtual harness uses the same immediate-STOP/direction-confirmation policy as live mode.
    private val virtualDecisionStabilizer = LiveDecisionStabilizer()
    private val activePathDiscovery = ActivePathDiscovery()
    private val liveDecisionStabilizer = LiveDecisionStabilizer()
    private val virtualPathDiscovery = ActivePathDiscovery(allowBlindBackoff = true)
    // If firmware independently vetoes a scan pivot, abort the current scan rather than assuming
    // the chassis actually rotated. Telemetry is read off the control thread.
    private val arduinoTurnRejected = AtomicBoolean(false)
    @Volatile private var activeDiscoveryState = "IDLE"
    @Volatile private var virtualDiscoveryState = "IDLE"
    private var lastVirtualSegment = -1
    private var virtualBackoffStartedAtMs = Long.MIN_VALUE
    private val speedController = SpeedController()
    private val controller = NavigationController(speedController = speedController)
    // Independent state for offline tests: stop latches / steering hysteresis must never be shared
    // with the live 10 Hz control loop, even during VIDEO TEST start/stop races.
    private val virtualController = NavigationController(speedController = SpeedController())
    private val freeSpaceAnalyzer = FreeSpaceAnalyzer()
    private val trafficSignFilter = TrafficSignFilter(requiredFrames = 3)
    private val freeSpacePreprocessor = ModelInputPreprocessor.Reusable(256, channelFirst = true)
    private val trafficSignPreprocessor = ModelInputPreprocessor.Reusable(640, channelFirst = false)
    private val freeSpaceMask = ByteArray(256 * 256)
    private val croppedMaskScratch = ByteArray(256 * 256)
    private val modelBusy = AtomicBoolean(false)
    private val snapshotLock = Any()
    // The virtual harness now injects the imperfections that the live robot has (perception latency,
    // missed/false obstacles, motor deadband/lag/mismatch, sonar dropouts). Use SimRealism.IDEAL to
    // get the old noise-free behaviour. Seeded: every run is repeatable.
    private val virtualScenario = VirtualScenarioProvider(
        physicalGeometryConfig, projector, realism = SimRealism.REALISTIC
    )
    private val yawTracker = YawTracker()
    private var yawSensor: YawSensor? = null
    private var virtualLastHeadingDeg = 0f

    @Volatile private var latestSnapshot = PerceptionSnapshot()
    @Volatile private var lastFrameAt = 0L
    @Volatile private var lastCommand = MotorCommand.STOP
    @Volatile private var lastSpeedPwm = 0
    @Volatile private var lastTelemetryUiAt = 0L
    @Volatile private var lastDistanceRequestAt = 0L
    @Volatile private var lastUltrasonicAt = 0L
    @Volatile private var lastStatusRequestAt = 0L
    /** usb.connectionId for which the Arduino has announced ROBOT_READY / STATUS=OK. */
    @Volatile private var readyConnectionId = -1
    /** Latched after an Arduino safety/error report until fresh ultrasonic clearance is confirmed. */
    @Volatile private var arduinoSafetyHold = false
    @Volatile private var arduinoSafetyHoldSince = 0L
    @Volatile private var mode = RobotMode.SAFE
    @Volatile private var modelStatus = "Models loading..."
    @Volatile private var virtualTestActive = false
    @Volatile private var virtualFrameIndex = 0
    @Volatile private var virtualTimeUs = 0L
    private var displayedVideoBitmap: Bitmap? = null

    private var lastLoggedCommand: MotorCommand? = null
    private var lastLoggedReason: String? = null
    private var lastLoggedSign: String? = null
    private var freeSpaceFailing = false
    private var trafficFailing = false
    private var lastStatusShown: String? = null
    private var lastButtonKey = 0

    @Volatile private var nextFreeInferenceAt = 0L
    @Volatile private var nextTrafficInferenceAt = 0L
    @Volatile private var lastFreeSpaceAt = 0L
    @Volatile private var lastTrafficSignAt = 0L
    @Volatile private var lastTrafficCandidateAt = 0L

    companion object {
        private const val FREE_INFERENCE_INTERVAL_MS = 140L
        private const val TRAFFIC_INFERENCE_INTERVAL_MS = 400L
        /** Keep a confirmed sign visible/usable through brief misses; controller freshness is 1.2 s. */
        private const val TRAFFIC_SIGN_DISPLAY_HOLD_MS = 1_200L
        // v0.5.11: the idle poll is ~5 Hz (acks of accepted motion commands carry D= at 10 Hz while moving).
        private const val DISTANCE_POLL_INTERVAL_MS = 150L
        /** Camera exposure -> analyzer hand-off; added to the analyzer-entry timestamp to estimate true capture time. */
        private const val CAMERA_PIPELINE_LATENCY_MS = 40L
        private const val STATUS_POLL_INTERVAL_MS = 500L
        /** Extra clearance per side for live driving (cm). 0 would allow zero-clearance gaps. */
        private const val LIVE_CLEARANCE_MARGIN_CM = 2f
        private const val MIN_PIVOT_CLEARANCE_CM = SafetyConstants.APP_PIVOT_CLEARANCE_CM
        // Live scanner uses the Arduino pivot cutoff + 4 cm, rather than the old 35 cm gate.
        // The 30 cm threshold still blocks translating commands; only verified in-place pivots pass.
        private const val MIN_SCAN_ULTRASONIC_CM = SafetyConstants.APP_TURN_MIN_DISTANCE_CM
    }

    private val cameraPermission = registerForActivityResult(
        ActivityResultContracts.RequestPermission()
    ) { granted ->
        if (granted) startCamera()
        else {
            markStatusDirty()
            statusText.text = "Camera permission denied - robot remains SAFE."
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
        DecisionLog.init(applicationContext)

        previewView = findViewById(R.id.previewView)
        videoImageView = findViewById(R.id.videoImageView)
        navigationOverlay = findViewById(R.id.navigationOverlay)
        statusText = findViewById(R.id.statusText)
        telemetryText = findViewById(R.id.telemetryText)
        connectUsbButton = findViewById(R.id.connectUsbButton)
        armButton = findViewById(R.id.armButton)
        stopButton = findViewById(R.id.stopButton)
        videoTestButton = findViewById(R.id.videoTestButton)
        videoPauseButton = findViewById(R.id.videoPauseButton)
        videoStepButton = findViewById(R.id.videoStepButton)
        videoResetButton = findViewById(R.id.videoResetButton)

        cameraExecutor = Executors.newSingleThreadExecutor()
        modelExecutor = Executors.newSingleThreadExecutor()
        ioExecutor = Executors.newFixedThreadPool(2)
        controlExecutor = Executors.newSingleThreadScheduledExecutor()
        usb = UsbSerialManager(this)
        models = ModelRegistry(this)
        usb.onTelemetryLine = ::handleUsbTelemetry

        connectUsbButton.setOnClickListener {
            markStatusDirty()
            statusText.text = "Connecting to USB serial..."
            ioExecutor.execute {
                val started = usb.connectFirstSupported()
                DecisionLog.log("USB", if (started) "connected" else "connect attempt failed / permission pending")
                runOnUiThread {
                    markStatusDirty()
                    statusText.text = if (started) {
                        "USB serial connected. Robot is SAFE until ARM is pressed."
                    } else {
                        "USB serial not connected; check OTG cable, device, and permission."
                    }
                    updateButtons()
                }
            }
        }

        armButton.setOnClickListener {
            when {
                arduinoSafetyHold -> {
                    markStatusDirty(); statusText.text = "Arduino safety interlock active. Wait for a fresh safe distance reading before ARM."
                    setMode(RobotMode.SAFE, "ARM refused: Arduino safety interlock")
                }
                !usb.connected -> {
                    markStatusDirty(); statusText.text = "Connect USB before arming."
                    setMode(RobotMode.SAFE, "ARM refused: USB not connected")
                }
                readyConnectionId != usb.connectionId && mode != RobotMode.AUTONOMOUS -> {
                    markStatusDirty(); statusText.text = "Waiting for Arduino ROBOT_READY (it resets when USB opens)."
                    setMode(RobotMode.SAFE, "ARM refused: Arduino not ready")
                }
                !models.areRequiredModelsLoaded -> {
                    markStatusDirty(); statusText.text = "Both supplied AI models must be loaded before autonomous control."
                    setMode(RobotMode.SAFE, "ARM refused: AI models not loaded")
                }
                else -> {
                    if (mode == RobotMode.AUTONOMOUS) setMode(RobotMode.SAFE, "user pressed DISARM")
                    else setMode(RobotMode.AUTONOMOUS, "user pressed ARM")
                }
            }
            updateButtons()
        }

        stopButton.setOnClickListener {
            val wasAlreadyStopped = mode == RobotMode.STOPPED
            setMode(RobotMode.STOPPED, "user pressed STOP")
            // setMode sends STOP on transitions; resend only if already in STOPPED mode.
            if (wasAlreadyStopped && usb.connected) usb.sendStop()
            markStatusDirty(); statusText.text = "STOP requested. Press ARM to resume autonomous control."
            updateButtons()
        }

        videoTestButton.setOnClickListener {
            if (virtualTestActive) {
                stopVirtualVideoTest()
                markStatusDirty(); statusText.text = "Returning to LIVE CAMERA..."
                startCamera()
                updateButtons()
            } else startVirtualVideoTest()
        }

        videoPauseButton.setOnClickListener {
            val runner = videoTestRunner ?: return@setOnClickListener
            if (runner.isPaused()) {
                runner.start()
                videoPauseButton.text = "PAUSE"
                markStatusDirty(); statusText.text = "VIDEO TEST running. Robot output is disabled."
            } else {
                runner.pause()
                videoPauseButton.text = "PLAY"
                markStatusDirty(); statusText.text = "VIDEO TEST paused. Robot output is disabled."
            }
        }

        videoStepButton.setOnClickListener {
            if (!virtualTestActive) startVirtualVideoTest(autoPlay = false)
            videoTestRunner?.step()
            videoPauseButton.text = "PLAY"
        }

        videoResetButton.setOnClickListener { resetVirtualVideoTest() }

        updateButtons()
        markStatusDirty(); statusText.text = "Starting camera and loading obstacle-navigation models..."

        ioExecutor.execute {
            val loaded = models.loadAvailable()
            modelStatus = when {
                models.areRequiredModelsLoaded -> "Both supplied models loaded"
                loaded.isEmpty() -> "No AI models loaded"
                else -> "Only ${loaded.size}/2 required AI models loaded"
            }
            nextFreeInferenceAt = 0L
            nextTrafficInferenceAt = 0L
            DecisionLog.log("MODEL", "$modelStatus (loaded=${loaded.joinToString()})")
            TrafficSignDecoder.unsupportedBehaviorLabels.takeIf { it.isNotEmpty() }?.let {
                DecisionLog.log("MODEL", "WARNING: traffic model has no class for ${it.joinToString()}; that sign behavior can never trigger")
            }
            runOnUiThread {
                markStatusDirty()
                statusText.text = if (models.areRequiredModelsLoaded) {
                    "AI ready: obstacle/free-space + traffic sign. SAFE until ARM."
                } else {
                    "$modelStatus - robot remains SAFE."
                }
                updateButtons()
            }
        }

        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED) {
            startCamera()
        } else cameraPermission.launch(Manifest.permission.CAMERA)

        controlExecutor.scheduleAtFixedRate({ tickControlLoop() }, 0L, 100L, TimeUnit.MILLISECONDS)
    }

    private fun startCamera() {
        val providerFuture = ProcessCameraProvider.getInstance(this)
        providerFuture.addListener({
            try {
                val provider = providerFuture.get()
                cameraProvider = provider
                val preview = Preview.Builder().build().also {
                    it.surfaceProvider = previewView.surfaceProvider
                }

                val analysisResolution = ResolutionSelector.Builder()
                    .setResolutionStrategy(
                        ResolutionStrategy(
                            Size(640, 480),
                            ResolutionStrategy.FALLBACK_RULE_CLOSEST_HIGHER_THEN_LOWER
                        )
                    )
                    .build()

                val analysis = ImageAnalysis.Builder()
                    .setResolutionSelector(analysisResolution)
                    .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                    .setOutputImageFormat(ImageAnalysis.OUTPUT_IMAGE_FORMAT_YUV_420_888)
                    .build()

                analysis.setAnalyzer(
                    cameraExecutor,
                    CameraAnalyzer(
                        shouldCaptureModelFrame = ::shouldCaptureModelFrame
                    ) { modelBitmap, modelFrameRequested, rotationDegrees, capturedAtMs ->
                        lastFrameAt = SystemClock.elapsedRealtime()
                        if (modelFrameRequested) {
                            if (modelBitmap != null) {
                                modelExecutor.execute {
                                    try { runDueModels(modelBitmap, rotationDegrees, capturedAtMs) }
                                    finally {
                                        modelBusy.set(false)
                                        modelBitmap.recycleSafely()
                                    }
                                }
                            } else modelBusy.set(false)
                        }
                    }
                )

                provider.unbindAll()
                provider.bindToLifecycle(this, CameraSelector.DEFAULT_BACK_CAMERA, preview, analysis)
                markStatusDirty(); statusText.text = "Camera ready. Obstacle navigation active; robot remains SAFE."
            } catch (t: Throwable) {
                markStatusDirty(); statusText.text = "Camera start failed: ${t.message}"
            }
        }, ContextCompat.getMainExecutor(this))
    }

    private fun startVirtualVideoTest(autoPlay: Boolean = true) {
        if (!models.areRequiredModelsLoaded) {
            markStatusDirty(); statusText.text = "Load both supplied AI models before starting the virtual test."
            return
        }

        if (!virtualTestActive) {
            cameraProvider?.unbindAll()
            previewView.visibility = View.GONE
            videoImageView.visibility = View.VISIBLE
            navigationOverlay.visibility = View.GONE // The POV renderer draws metric boundaries/path into the image itself.
            virtualScenario.reset()
            virtualPlanner.reset()
            virtualObstacleTracker.reset()
            virtualDecisionStabilizer.reset()
            virtualPathDiscovery.reset()
            virtualDiscoveryState = "IDLE"
            lastVirtualSegment = -1
            val wasAutonomousBeforeVirtualTest = mode == RobotMode.AUTONOMOUS
            setMode(RobotMode.SAFE, "virtual test started; physical robot output disabled")
            if (!wasAutonomousBeforeVirtualTest && usb.connected) usb.sendStop()
            latestSnapshot = PerceptionSnapshot()
            controller.reset()
            virtualController.reset()
            obstacleTracker.reset()
            trafficSignFilter.reset()
            nextFreeInferenceAt = 0L
            nextTrafficInferenceAt = 0L
            lastFreeSpaceAt = 0L
            lastTrafficSignAt = 0L
            lastTrafficCandidateAt = 0L
            lastCommand = MotorCommand.STOP
            lastSpeedPwm = 0
            virtualFrameIndex = 0
            virtualTimeUs = 0L
            virtualTestActive = true

            videoTestRunner?.release()
            videoTestRunner = VideoTestRunner(
                context = this,
                resourceId = R.raw.robot_virtual_test,
                onFrame = { bitmap, frameIndex, timeUs ->
                    modelExecutor.submit { processVirtualVideoFrame(bitmap, frameIndex, timeUs) }.get()
                },
                onCompleted = {
                    runOnUiThread {
                        virtualTestActive = true
                        videoPauseButton.text = "PLAY"
                        markStatusDirty(); statusText.text = "VIDEO TEST complete. Robot output remained DISABLED."
                    }
                },
                onError = { error ->
                    virtualTestActive = false
                    runOnUiThread {
                        markStatusDirty(); statusText.text = "VIDEO TEST failed: ${error.message}"
                    }
                }
            )
        }

        if (autoPlay) {
            videoTestRunner?.start()
            videoPauseButton.text = "PAUSE"
            markStatusDirty(); statusText.text = "VIDEO TEST running. Robot output is DISABLED."
        } else {
            videoTestRunner?.pause()
            videoPauseButton.text = "PLAY"
            markStatusDirty(); statusText.text = "VIDEO TEST ready. Press PLAY or STEP. Robot output is DISABLED."
        }
        updateButtons()
    }

    private fun processVirtualVideoFrame(bitmap: Bitmap, frameIndex: Int, timeUs: Long) {
        var povBitmap: Bitmap? = null
        try {
            virtualFrameIndex = frameIndex
            virtualTimeUs = timeUs
            lastFrameAt = SystemClock.elapsedRealtime()

            // Scene timing is aligned to the 10 FPS virtual clock. Reset temporal state at scene
            // boundaries so detections from the previous synthetic world cannot leak into the next.
            val scenario = virtualScenario.scenarioForFrame(frameIndex)
            if (scenario.segment != lastVirtualSegment) {
                lastVirtualSegment = scenario.segment
                virtualObstacleTracker.reset()
                virtualPlanner.reset()
                virtualController.reset()
                virtualDecisionStabilizer.reset()
                virtualPathDiscovery.reset()
                virtualDiscoveryState = "IDLE"
                virtualBackoffStartedAtMs = Long.MIN_VALUE
                virtualLastHeadingDeg = 0f
                virtualScenario.perception.reset() // do not feed the previous scene's masks into this one
                virtualScenario.finishBackoffProgress()
                virtualScenario.stopMotorSimulation()
                updateSnapshot {
                    copy(trafficSign = null, trafficSignCandidate = null, trafficSignCandidateAgeMs = Long.MAX_VALUE, trafficSignAgeMs = Long.MAX_VALUE)
                }
            }

            // Synthetic obstacle ground truth and displayed POV use the same physical scene/pose.
            // Do not run the traffic-sign model on scene-title text in the clock video; it caused
            // false signs and STOP commands in the virtual harness. Live-camera sign inference is unchanged.
            // Latency, jitter, missed obstacles and false blobs are applied here (no-ops for IDEAL).
            applySemanticMask(
                virtualScenario.perception.process(
                    scenario.mask, VirtualScenarioProvider.WIDTH, VirtualScenarioProvider.HEIGHT
                ),
                VirtualScenarioProvider.WIDTH, VirtualScenarioProvider.HEIGHT
            )

            val now = SystemClock.elapsedRealtime()
            val virtualSonarCm = virtualScenario.forwardUltrasonicDistanceCm(frameIndex)
            val snapshot = currentSnapshotWithAges(now).copy(
                ultrasonicCm = virtualSonarCm,
                ultrasonicNoEcho = virtualSonarCm == null,
                ultrasonicAgeMs = 0L,
                ultrasonicRequired = true,
                // Live age counts from frame capture; make the virtual map equally old.
                freeSpaceAgeMs = virtualScenario.perception.latencyMs
            )
            val virtualNowMs = timeUs / 1_000L
            val rawDecision = virtualController.decide(snapshot, virtualNowMs)
            val priorVirtualDiscoveryState = virtualDiscoveryState
            val virtualNearest = snapshot.obstaclePlan?.nearestObstacleCm
            val sonarAllowsScan = if (snapshot.ultrasonicNoEcho) {
                virtualNearest?.let { it >= MIN_PIVOT_CLEARANCE_CM + 3f } == true
            } else {
                virtualSonarCm?.let { it >= MIN_SCAN_ULTRASONIC_CM } == true
            }
            val scanOutput = virtualPathDiscovery.update(
                snapshot = snapshot,
                normalDecision = rawDecision,
                nowMs = virtualNowMs,
                scanClearanceVerified = virtualScenario.canRotateInPlace(frameIndex) &&
                    virtualNearest?.let { it >= MIN_PIVOT_CLEARANCE_CM } == true && sonarAllowsScan,
                rotation = virtualRotationFeedback()
            )
            virtualDiscoveryState = scanOutput.state.label
            var decision = if (scanOutput.ownsOutput) {
                // The active scan owns continuous low-speed keepalive steps; do not add a second direction debounce.
                virtualDecisionStabilizer.reset()
                scanOutput.decision
            } else {
                virtualDecisionStabilizer.apply(scanOutput.decision)
            }

            // In virtual mode the same BACKOFF state now moves the chassis backward over time,
            // rather than teleporting the full 5 cm on its first frame. The hardware pulse is 250 ms;
            // MP4 frames are 100 ms apart, so the last frame completes the bounded displacement.
            if (scanOutput.state == ActivePathDiscovery.State.BACKOFF &&
                priorVirtualDiscoveryState != ActivePathDiscovery.State.BACKOFF.label) {
                virtualBackoffStartedAtMs = virtualNowMs
                virtualScenario.finishBackoffProgress()
            }
            if (scanOutput.state == ActivePathDiscovery.State.BACKOFF ||
                priorVirtualDiscoveryState == ActivePathDiscovery.State.BACKOFF.label) {
                val elapsed = if (virtualBackoffStartedAtMs == Long.MIN_VALUE) 0L
                    else (virtualNowMs - virtualBackoffStartedAtMs).coerceAtLeast(0L)
                val completedNormally = priorVirtualDiscoveryState == ActivePathDiscovery.State.BACKOFF.label &&
                    scanOutput.state !in setOf(
                        ActivePathDiscovery.State.UNSAFE,
                        ActivePathDiscovery.State.SAFETY_HOLD,
                        ActivePathDiscovery.State.NO_ROUTE
                    )
                val fraction = if (completedNormally) 1f else (elapsed / 250f).coerceIn(0f, 1f)
                val modeledBackoffCm = virtualScenario.modeledBackoffDistanceCm()
                if (!virtualScenario.backOffToDistance(frameIndex, modeledBackoffCm, fraction)) {
                    val failed = virtualPathDiscovery.forceSafetyStop("virtual rearward footprint is obstructed")
                    decision = failed.decision
                    virtualDiscoveryState = failed.state.label
                }
                if (scanOutput.state != ActivePathDiscovery.State.BACKOFF) {
                    virtualScenario.finishBackoffProgress()
                    virtualBackoffStartedAtMs = Long.MIN_VALUE
                }
            }

            // Scan pivots use the same wheel/PWM kinematics as the virtual motor model and are
            // collision-gated against the full swept footprint.
            if (scanOutput.isRotationPulse && !virtualScenario.canRotateInPlace(frameIndex)) {
                decision = virtualDecisionStabilizer.forceStop(
                    "simulated collision: pivot footprint intersects an obstacle"
                )
                virtualDiscoveryState = "SCAN STOPPED / TURN NOT CLEAR"
            } else if (!scanOutput.ownsOutput && virtualScenario.wouldCollideOnMotorStep(
                    frameIndex, decision.command, decision.speedPwm, 0.1f
                )) {
                decision = virtualDecisionStabilizer.forceStop(
                    "simulated collision: motor-command swept footprint intersects an obstacle"
                )
            }
            lastCommand = decision.command
            lastSpeedPwm = decision.speedPwm
            logDecisionIfChanged(decision, snapshot, "VIRTUAL")

            povBitmap = virtualScenario.renderFrame(scenario, snapshot.obstaclePlan, decision, frameIndex, timeUs)
            // Scan steps and ordinary navigation now share the same differential-drive model.
            // No perfect curve-following shortcut is used in virtual mode.
            if (scanOutput.isRotationPulse && decision.command != MotorCommand.STOP) {
                if (!virtualScenario.rotateForScan(
                        frameIndex, decision.command, decision.speedPwm,
                        MotorCalibration.SCAN_PULSE_DURATION_MS / 1_000f
                    )) {
                    virtualDiscoveryState = "SCAN STOPPED / TURN NOT CLEAR"
                }
            } else if (scanOutput.ownsOutput) {
                if (decision.command == MotorCommand.STOP) virtualScenario.stopMotorSimulation()
            } else if (decision.command == MotorCommand.STOP) {
                virtualScenario.stopMotorSimulation()
            } else if (!virtualScenario.advance(
                frameIndex, decision.command, decision.speedPwm, 0.1f,
                noEcho = snapshot.ultrasonicNoEcho
            )) {
                virtualDecisionStabilizer.forceStop("simulated collision while applying motor command")
            }
            bitmap.recycleSafely() // input MP4 is now only a timing clock, not the displayed camera view.

            val outputFrame = povBitmap
            if (outputFrame == null) return
            runOnUiThread {
                if (!virtualTestActive) {
                    outputFrame.recycleSafely()
                    return@runOnUiThread
                }
                val old = displayedVideoBitmap
                videoImageView.setImageBitmap(outputFrame)
                displayedVideoBitmap = outputFrame
                old?.recycleSafely()
                telemetryText.text = buildVirtualTelemetry(scenario.title, scenario.pose, snapshot, decision)
                markStatusDirty()
                statusText.text = "VIDEO TEST • Frame $frameIndex • ${decision.command.name} @ ${decision.speedPwm} • output disabled"
            }
        } catch (t: Throwable) {
            bitmap.recycleSafely()
            povBitmap?.recycleSafely()
            DecisionLog.log("VIRTUAL_TEST", "frame=$frameIndex failed: ${t.message}")
        }
    }

    /** Virtual stand-in for the phone gyro: pose heading is right-positive, feedback yaw is left-positive. */
    private fun virtualRotationFeedback(): RotationFeedback {
        val heading = virtualScenario.currentPoseSnapshot().headingDeg
        val rate = -(heading - virtualLastHeadingDeg) / 0.1f
        virtualLastHeadingDeg = heading
        return RotationFeedback(-heading, rate)
    }

    private fun runTrafficModelOnlyWhenDue(bitmap: Bitmap) {
        var now = SystemClock.elapsedRealtime()
        if (!models.trafficSign.isLoaded || now < nextTrafficInferenceAt) return
        runCatching {
            trafficSignPreprocessor.channelFirst = models.trafficSign.inputChannelFirst
            val prepared = trafficSignPreprocessor.prepare(bitmap, 0)
            val output = models.trafficSign.runFloat(prepared.floats)
            val rawSign = TrafficSignDecoder.decode(output, prepared.transform)
            val confirmed = trafficSignFilter.update(rawSign)
            publishTrafficSignResult(rawSign, confirmed)
        }.onFailure { error ->
            if (!trafficFailing) {
                trafficFailing = true
                DecisionLog.log("MODEL", "virtual traffic-sign inference failed: ${error.message}")
            }
            updateSnapshot { copy(trafficSign = null, trafficSignCandidate = null, trafficSignCandidateAgeMs = Long.MAX_VALUE, trafficSignAgeMs = Long.MAX_VALUE) }
            lastTrafficSignAt = 0L
            lastTrafficCandidateAt = 0L
        }
        now = SystemClock.elapsedRealtime()
        nextTrafficInferenceAt = now + TRAFFIC_INFERENCE_INTERVAL_MS
    }

    private fun resetVirtualVideoTest() {
        if (!virtualTestActive) startVirtualVideoTest(autoPlay = false)
        videoTestRunner?.reset()
        controller.reset()
        virtualController.reset()
        obstacleTracker.reset()
        virtualObstacleTracker.reset()
        virtualScenario.reset()
        virtualPlanner.reset()
        virtualDecisionStabilizer.reset()
        virtualPathDiscovery.reset()
        virtualDiscoveryState = "IDLE"
        virtualBackoffStartedAtMs = Long.MIN_VALUE
        lastVirtualSegment = -1
        trafficSignFilter.reset()
        latestSnapshot = PerceptionSnapshot()
        nextFreeInferenceAt = 0L
        nextTrafficInferenceAt = 0L
        lastFreeSpaceAt = 0L
        lastTrafficSignAt = 0L
        lastTrafficCandidateAt = 0L
        lastCommand = MotorCommand.STOP
        lastSpeedPwm = 0
        lastLoggedCommand = null
        lastLoggedReason = null
        displayedVideoBitmap?.recycleSafely()
        displayedVideoBitmap = null
        videoImageView.setImageDrawable(null)
        navigationOverlay.clear()
        telemetryText.text = "SOURCE: VIRTUAL VIDEO\nREADY - press PLAY or STEP\nROBOT OUTPUT: DISABLED"
        videoPauseButton.text = "PLAY"
        markStatusDirty(); statusText.text = "VIDEO TEST reset. Robot output is DISABLED."
    }

    private fun stopVirtualVideoTest() {
        virtualTestActive = false
        obstacleTracker.reset()
        virtualObstacleTracker.reset()
        virtualScenario.reset()
        virtualPlanner.reset()
        virtualController.reset()
        virtualDecisionStabilizer.reset()
        virtualPathDiscovery.reset()
        virtualDiscoveryState = "IDLE"
        virtualBackoffStartedAtMs = Long.MIN_VALUE
        lastVirtualSegment = -1
        videoTestRunner?.release()
        videoTestRunner = null
        displayedVideoBitmap?.recycleSafely()
        displayedVideoBitmap = null
        videoImageView.setImageDrawable(null)
        videoImageView.visibility = View.GONE
        previewView.visibility = View.VISIBLE
        navigationOverlay.visibility = View.VISIBLE
        navigationOverlay.clear()
    }

    private fun shouldCaptureModelFrame(): Boolean {
        if (!models.areRequiredModelsLoaded) return false
        val now = SystemClock.elapsedRealtime()
        val due = now >= nextFreeInferenceAt || now >= nextTrafficInferenceAt
        return due && modelBusy.compareAndSet(false, true)
    }

    private fun runDueModels(bitmap: Bitmap, rotationDegrees: Int, capturedAtMs: Long) {
        var now = SystemClock.elapsedRealtime()

        if (now >= nextFreeInferenceAt) {
            runCatching {
                val prepared = freeSpacePreprocessor.prepare(bitmap, rotationDegrees)
                models.freeSpace.runSemantic(prepared.floats, 256, 256, freeSpaceMask)
                val contentMask = ModelInputPreprocessor.cropMaskToContent(
                    freeSpaceMask, 256, 256, prepared.transform, croppedMaskScratch
                )
                applySemanticMask(contentMask.data, contentMask.width, contentMask.height, capturedAtMs - CAMERA_PIPELINE_LATENCY_MS)
                if (freeSpaceFailing) {
                    freeSpaceFailing = false
                    DecisionLog.log("MODEL", "free-space inference recovered")
                }
            }.onFailure { error ->
                if (!freeSpaceFailing) {
                    freeSpaceFailing = true
                    DecisionLog.log("MODEL", "free-space inference FAILED -> robot will STOP: ${error.message}")
                }
                updateSnapshot { copy(freeSpaceValid = false, freeSpaceAgeMs = Long.MAX_VALUE, obstaclePlan = null) }
            }
            now = SystemClock.elapsedRealtime()
            nextFreeInferenceAt = now + FREE_INFERENCE_INTERVAL_MS
        }

        now = SystemClock.elapsedRealtime()
        if (now >= nextTrafficInferenceAt) {
            runCatching {
                trafficSignPreprocessor.channelFirst = models.trafficSign.inputChannelFirst
                val prepared = trafficSignPreprocessor.prepare(bitmap, rotationDegrees)
                val output = models.trafficSign.runFloat(prepared.floats)
                val rawSign = TrafficSignDecoder.decode(output, prepared.transform)
                val confirmed = trafficSignFilter.update(rawSign)
                publishTrafficSignResult(rawSign, confirmed)
                val signKey = rawSign?.label
                if (signKey != lastLoggedSign) {
                    lastLoggedSign = signKey
                    DecisionLog.log("SIGN", if (rawSign != null) {
                        "raw candidate ${rawSign.label} confidence=${"%.2f".format(java.util.Locale.US, rawSign.confidence)}; " +
                            if (confirmed != null) "confirmed" else "awaiting 3 matching frames"
                    } else "no raw detection this inference")
                }
                if (trafficFailing) {
                    trafficFailing = false
                    DecisionLog.log("MODEL", "traffic-sign inference recovered")
                }
            }.onFailure { error ->
                if (!trafficFailing) {
                    trafficFailing = true
                    DecisionLog.log("MODEL", "traffic-sign inference FAILED -> sign data dropped: ${error.message}")
                }
                updateSnapshot { copy(trafficSign = null, trafficSignCandidate = null, trafficSignCandidateAgeMs = Long.MAX_VALUE, trafficSignAgeMs = Long.MAX_VALUE) }
                lastTrafficSignAt = 0L
                lastTrafficCandidateAt = 0L
            }
            nextTrafficInferenceAt = SystemClock.elapsedRealtime() + TRAFFIC_INFERENCE_INTERVAL_MS
        }
    }

    /**
     * [capturedAtMs] is when the frame behind [rawMask] was captured (elapsedRealtime). The map's
     * age is measured from it, so camera + inference latency counts against freshness limits.
     */
    private fun applySemanticMask(
        rawMask: ByteArray, width: Int, height: Int,
        capturedAtMs: Long = SystemClock.elapsedRealtime()
    ) {
        // Temporal stabiliser: obstacles accepted at once, FREE only after consecutive agreement.
        val tracker = if (virtualTestActive) virtualObstacleTracker else obstacleTracker
        val mask = tracker.update(rawMask, width, height, SystemClock.elapsedRealtime())
        val scores = freeSpaceAnalyzer.analyze(mask, width, height)
        val plan = (if (virtualTestActive) virtualPlanner else livePlanner).analyze(mask, width, height)
        updateSnapshot {
            copy(
                freeLeft = scores.freeLeft,
                freeCenter = scores.freeCenter,
                freeRight = scores.freeRight,
                obstacleCenter = scores.obstacleCenter,
                unknownCenter = scores.unknownCenter,
                freeSpaceValid = true,
                obstaclePlan = plan,
                freeSpaceAgeMs = 0L
            )
        }
        lastFreeSpaceAt = capturedAtMs
    }

    /**
     * Keep diagnostic candidates separate from temporally confirmed signs. A single missed inference
     * no longer erases a confirmed sign immediately; it remains available to NavigationController
     * only while the same 1.2 s freshness window used by that controller is valid.
     */
    private fun publishTrafficSignResult(
        raw: com.example.robotcar.navigation.TrafficSign?,
        confirmed: com.example.robotcar.navigation.TrafficSign?
    ) {
        val now = SystemClock.elapsedRealtime()
        lastTrafficCandidateAt = if (raw != null) now else 0L
        if (confirmed != null) {
            lastTrafficSignAt = now
            updateSnapshot {
                copy(trafficSign = confirmed, trafficSignCandidate = raw, trafficSignCandidateAgeMs = if (raw != null) 0L else Long.MAX_VALUE, trafficSignAgeMs = 0L)
            }
            return
        }
        val age = if (lastTrafficSignAt > 0L) (now - lastTrafficSignAt).coerceAtLeast(0L) else Long.MAX_VALUE
        if (age <= TRAFFIC_SIGN_DISPLAY_HOLD_MS) {
            updateSnapshot { copy(trafficSignCandidate = raw, trafficSignCandidateAgeMs = if (raw != null) 0L else Long.MAX_VALUE, trafficSignAgeMs = age) }
        } else {
            lastTrafficSignAt = 0L
            // Do not clear lastTrafficCandidateAt here: a fresh raw candidate may still be visible
            // while the previous confirmed sign expires from the behavior-authorizing state.
            updateSnapshot {
                copy(trafficSign = null, trafficSignCandidate = raw, trafficSignCandidateAgeMs = if (raw != null) 0L else Long.MAX_VALUE, trafficSignAgeMs = Long.MAX_VALUE)
            }
        }
    }

    private fun currentSnapshotWithAges(now: Long): PerceptionSnapshot {
        return synchronized(snapshotLock) {
            latestSnapshot.copy(
                cameraAgeMs = if (lastFrameAt == 0L) Long.MAX_VALUE else (now - lastFrameAt).coerceAtLeast(0L),
                freeSpaceAgeMs = if (latestSnapshot.freeSpaceValid && lastFreeSpaceAt > 0L) (now - lastFreeSpaceAt).coerceAtLeast(0L) else Long.MAX_VALUE,
                trafficSignAgeMs = if (latestSnapshot.trafficSign != null && lastTrafficSignAt > 0L) (now - lastTrafficSignAt).coerceAtLeast(0L) else Long.MAX_VALUE,
                trafficSignCandidateAgeMs = if (latestSnapshot.trafficSignCandidate != null && lastTrafficCandidateAt > 0L) (now - lastTrafficCandidateAt).coerceAtLeast(0L) else Long.MAX_VALUE,
                ultrasonicAgeMs = if (lastUltrasonicAt > 0L) (now - lastUltrasonicAt).coerceAtLeast(0L) else Long.MAX_VALUE,
                ultrasonicRequired = false // pictured chassis has no visible HC-SR04; use fresh camera geometry only
            )
        }
    }

    private fun updateSnapshot(block: PerceptionSnapshot.() -> PerceptionSnapshot) {
        synchronized(snapshotLock) { latestSnapshot = latestSnapshot.block() }
    }

    private fun liveScanClearanceVerified(snapshot: PerceptionSnapshot): Boolean {
        val plan = snapshot.obstaclePlan ?: return false
        val nearest = plan.nearestObstacleCm ?: return false
        if (snapshot.cameraAgeMs > 350L || !snapshot.freeSpaceValid ||
            snapshot.freeSpaceAgeMs > SafetyConstants.SCAN_EVIDENCE_MAX_AGE_MS) return false
        if (nearest < MIN_PIVOT_CLEARANCE_CM) return false
        // The uploaded chassis photo shows an Arduino UNO + L298N but no clearly visible HC-SR04.
        // Do not wait forever for sonar telemetry that this hardware may not provide. A pivot is
        // allowed only with fresh camera geometry and a conservative front-clearance margin.
        if (snapshot.ultrasonicRequired) {
            if (snapshot.ultrasonicAgeMs > SafetyConstants.ULTRASONIC_MAX_AGE_MS) return false
            return if (snapshot.ultrasonicNoEcho) {
                nearest >= MIN_PIVOT_CLEARANCE_CM + 3f
            } else {
                snapshot.ultrasonicCm?.let { it >= MIN_SCAN_ULTRASONIC_CM } == true
            }
        }
        return nearest >= MIN_PIVOT_CLEARANCE_CM + 3f
    }

    private fun tickControlLoop() {
        // Virtual frames have their own serialized perception→plan→stabilize→simulate pipeline.
        // Do not call the same stateful controller from this scheduled live-control loop as well:
        // the old double invocation reset steering hysteresis and caused STOP/motion flicker.
        if (virtualTestActive) return
        if (arduinoSafetyHold && mode == RobotMode.AUTONOMOUS) {
            setMode(RobotMode.SAFE, "Arduino safety interlock latched")
        }

        val now = SystemClock.elapsedRealtime()
        val snapshot = currentSnapshotWithAges(now)
        val baseDecision = controller.decide(snapshot, now)
        val robotReady = usb.connected && readyConnectionId == usb.connectionId
        val firmwareRejectedPivot = arduinoTurnRejected.getAndSet(false) && activePathDiscovery.isScanning
        val discoveryOutput = if (mode == RobotMode.AUTONOMOUS && usb.connected && robotReady) {
            if (firmwareRejectedPivot) {
                activePathDiscovery.forceSafetyStop("Arduino rejected scan pivot; assumed rotation is not trusted")
            } else {
                activePathDiscovery.update(
                    snapshot = snapshot,
                    normalDecision = baseDecision,
                    nowMs = now,
                    scanClearanceVerified = liveScanClearanceVerified(snapshot),
                    // Closed-loop pivot check from the phone gyro; null (no sensor) = open loop.
                    rotation = if (yawTracker.isReady())
                        RotationFeedback(yawTracker.yawDegrees(), yawTracker.yawRateDegPerSec()) else null
                )
            }
        } else {
            activePathDiscovery.reset()
            ActivePathDiscovery.Output(baseDecision, ActivePathDiscovery.State.IDLE, ownsOutput = false, isRotationPulse = false)
        }
        activeDiscoveryState = discoveryOutput.state.label
        val decision = if (discoveryOutput.ownsOutput) {
            // The scan controller already uses explicit pulse/settle states and clearance checks.
            // Do not debounce its timed pulses, but reset live hysteresis while it owns motion.
            liveDecisionStabilizer.reset()
            discoveryOutput.decision
        } else {
            liveDecisionStabilizer.apply(discoveryOutput.decision)
        }
        lastCommand = decision.command
        lastSpeedPwm = decision.speedPwm

        if (usb.connected && !robotReady && now - lastStatusRequestAt >= STATUS_POLL_INTERVAL_MS) {
            // Opening the port resets an UNO; ask until it answers instead of assuming it is up.
            lastStatusRequestAt = now
            if (!usb.send("STATUS")) {
                setMode(RobotMode.SAFE, "USB status request failed")
            }
        }
        if (usb.connected && now - lastDistanceRequestAt >= DISTANCE_POLL_INTERVAL_MS) {
            lastDistanceRequestAt = now
            if (!usb.send("DISTANCE")) {
                setMode(RobotMode.SAFE, "USB distance request failed")
            }
        }

        if (mode == RobotMode.AUTONOMOUS && usb.connected && !robotReady) {
            setMode(RobotMode.SAFE, "Arduino not ready (no ROBOT_READY/STATUS=OK since connect)")
            postStatusIfChanged("Arduino not ready - robot forced SAFE.")
        } else if (mode == RobotMode.AUTONOMOUS) {
            if (decision.command != lastLoggedCommand || decision.reason != lastLoggedReason) {
                lastLoggedCommand = decision.command
                lastLoggedReason = decision.reason
                DecisionLog.log("CMD", "${decision.command.name},${decision.speedPwm} (${decision.reason}) | ${describe(snapshot)}")
            }
            if (usb.connected) {
                val sent = usb.send(decision.command.wire, decision.speedPwm)
                if (!sent) {
                    setMode(RobotMode.SAFE, "USB write failed")
                    postStatusIfChanged("USB write failed - robot forced SAFE.")
                } else postStatusIfChanged("AUTO: ${decision.command.name} @ ${decision.speedPwm} - ${decision.reason}")
            } else {
                setMode(RobotMode.SAFE, "USB disconnected while autonomous")
                postStatusIfChanged("USB disconnected - robot forced SAFE.")
            }
        } else if (mode == RobotMode.SAFE) {
            // SHADOW MODE: perception, planning and the scan state machine already run in SAFE; only the
            // motor output is withheld. Log what WOULD have been sent so a hand-pushed robot (or one on
            // blocks) can be compared against what is physically in front of it, with zero motion risk.
            if (decision.command != lastLoggedCommand || decision.reason != lastLoggedReason) {
                lastLoggedCommand = decision.command
                lastLoggedReason = decision.reason
                DecisionLog.log("SHADOW", "${decision.command.name},${decision.speedPwm} (${decision.reason}) | ${describe(snapshot)}")
            }
        }

        if (now - lastTelemetryUiAt >= 200L) {
            lastTelemetryUiAt = now
            runOnUiThread {
                val scanCue = if (decision.pathType == PathType.SCANNING) decision.command else null
                navigationOverlay.update(
                    snapshot.obstaclePlan,
                    snapshot.obstaclePlan?.bestPath?.lateralOffsetCm ?: 0f,
                    scanCommand = scanCue,
                    pathAuthorized = decision.command != MotorCommand.STOP ||
                        decision.reason.contains("live command stabilizer", ignoreCase = true)
                )
                telemetryText.text = buildLiveTelemetry(snapshot, decision)
                updateButtons()
            }
        }
    }

    /**
     * Applies any sonar reading carried by a firmware line. Since v0.5.11 the firmware appends the
     * reading that authorised each motion command (RESULT=F,90,D=41.2), so while moving the app's sonar
     * is ~100 ms old instead of up to 400-800 ms. Garbage is ignored and ages out into a stale-STOP.
     */
    private fun applyUltrasonicReading(line: String) {
        val reading = SerialProtocol.parseUltrasonic(line) ?: return
        val receivedAt = SystemClock.elapsedRealtime()
        lastUltrasonicAt = receivedAt
        updateSnapshot { copy(ultrasonicCm = reading.cm, ultrasonicNoEcho = reading.noEcho) }
        val validDistance = reading.cm
        if (arduinoSafetyHold && validDistance != null &&
            receivedAt > arduinoSafetyHoldSince &&
            validDistance >= SafetyConstants.ULTRASONIC_STOP_CM) {
            arduinoSafetyHold = false
            DecisionLog.log("ARDUINO_SAFETY", "fresh safe distance ${validDistance}cm; interlock cleared, manual ARM required")
            runOnUiThread {
                markStatusDirty()
                statusText.text = "Fresh safe distance received. Robot remains SAFE; press ARM to resume."
                updateButtons()
            }
        }
    }

    private fun handleUsbTelemetry(line: String) {
        applyUltrasonicReading(line)
        when {
            // Sonar readings (DISTANCE=..., RESULT=F,90,D=..., RESULT=STOP_*_OBSTACLE:...) were applied
            // by applyUltrasonicReading() before this when-block.
            line.startsWith("DISTANCE=", ignoreCase = true) -> Unit
            line.startsWith("ROBOT_READY", ignoreCase = true) ||
                line.startsWith("STATUS=OK", ignoreCase = true) -> {
                readyConnectionId = usb.connectionId
                DecisionLog.log("ARDUINO", line)
            }
            line.startsWith("RESULT=STOP_TURN_OBSTACLE", ignoreCase = true) -> {
                // The firmware's current sonar disagreed with the app's cached reading. If this
                // happened during a scan pulse, do not continue as though the robot rotated.
                arduinoTurnRejected.set(true)
                DecisionLog.log("ARDUINO_TURN_REJECTED", line)
                postStatusIfChanged("Scan pivot rejected by Arduino; stopping scan and refreshing clearance.")
            }
            line.startsWith("RESULT=STOP_OBSTACLE", ignoreCase = true) ||
                line.startsWith("RESULT=STOP_NO_ECHO", ignoreCase = true) ||
                line.startsWith("SAFETY_STOP=DISTANCE", ignoreCase = true) -> {
                // A transient obstacle/sensor stop is part of normal autonomous navigation.
                // Keep the app armed; the controller should command STOP while blocked and
                // resume only when perception/sensor evidence shows a feasible safe path.
                DecisionLog.log("ARDUINO_OBSTACLE_STOP", line)
                postStatusIfChanged("Obstacle/sensor stop: $line. Autonomous mode remains armed; waiting for a safe path.")
            }
            line.startsWith("SAFETY_STOP=WATCHDOG", ignoreCase = true) ||
                line.startsWith("ERROR=", ignoreCase = true) -> {
                // Communication/protocol faults remain latched and require a fresh clearance
                // reading plus a manual re-arm. Do not conflate these with normal obstacle stops.
                arduinoSafetyHold = true
                arduinoSafetyHoldSince = SystemClock.elapsedRealtime()
                DecisionLog.log("ARDUINO_SAFETY", line)
                runOnUiThread {
                    if (mode == RobotMode.AUTONOMOUS) {
                        setMode(RobotMode.SAFE, "Arduino communication/protocol fault: $line")
                    }
                    postStatusIfChanged("Arduino fault ($line). Wait for fresh safe distance, then press ARM.")
                    updateButtons()
                }
            }
            line.startsWith("SAFETY_STOP", ignoreCase = true) -> {
                // Unknown future safety-stop types are conservative faults; known distance stops
                // are handled above as transient obstacle events.
                arduinoSafetyHold = true
                arduinoSafetyHoldSince = SystemClock.elapsedRealtime()
                DecisionLog.log("ARDUINO_SAFETY", line)
                runOnUiThread {
                    if (mode == RobotMode.AUTONOMOUS) setMode(RobotMode.SAFE, "Arduino safety fault: $line")
                    postStatusIfChanged("Arduino safety fault ($line). Wait for fresh safe distance, then press ARM.")
                    updateButtons()
                }
            }
            line.startsWith("RESULT=", ignoreCase = true) -> DecisionLog.log("ARDUINO", line)
        }
    }

    private fun logDecisionIfChanged(
        decision: com.example.robotcar.navigation.NavigationDecision,
        snapshot: PerceptionSnapshot,
        prefix: String
    ) {
        if (decision.command != lastLoggedCommand || decision.reason != lastLoggedReason) {
            lastLoggedCommand = decision.command
            lastLoggedReason = decision.reason
            DecisionLog.log(
                if (decision.command == MotorCommand.STOP) "${prefix}_SAFETY" else "${prefix}_CMD",
                "${decision.command.name},${decision.speedPwm} (${decision.reason}) | ${describe(snapshot)}"
            )
        }
    }

    private fun setMode(newMode: RobotMode, why: String) {
        val old = mode
        mode = newMode
        if (newMode != RobotMode.AUTONOMOUS) {
            activePathDiscovery.reset()
            liveDecisionStabilizer.reset()
            activeDiscoveryState = "IDLE"
        }
        if (old != newMode) {
            DecisionLog.log("MODE", "${old.name} -> ${newMode.name}: $why")
            if (newMode != RobotMode.AUTONOMOUS) {
                lastLoggedCommand = null
                lastLoggedReason = null
                // Transmit one immediate STOP when leaving AUTO; do not leave stop commands queued.
                if (usb.connected) usb.sendStop()
            }
        } else DecisionLog.log("MODE", "stays ${old.name}: $why")
    }

    private fun describe(s: PerceptionSnapshot): String {
        val p = s.obstaclePlan
        val path = p?.bestPath
        return "free=${s.freeLeft.formatScore()}/${s.freeCenter.formatScore()}/${s.freeRight.formatScore()} " +
            "obs=${s.obstacleCenter.formatScore()} unk=${s.unknownCenter.formatScore()} " +
            "gap=${path?.minimumGapWidthCm?.formatCm() ?: "--"} " +
            "ultra=${s.ultrasonicCm?.formatCm() ?: "--"} " +
            "boundary=${p?.boundaryState ?: "--"} age(cam/free/sign)=" +
            "${s.cameraAgeMs.ageText()}/${s.freeSpaceAgeMs.ageText()}/${s.trafficSignAgeMs.ageText()}"
    }

    private fun buildLiveTelemetry(s: PerceptionSnapshot, d: com.example.robotcar.navigation.NavigationDecision): String = buildString {
        val p = s.obstaclePlan
        append("MODE: ").append(mode.name).append('\n')
        append("PATH DISCOVERY: ").append(activeDiscoveryState).append('\n')
        append("ROBOT: 15.0 x 25.5 cm  REQUIRED GAP: ")
            .append(physicalGeometryConfig.requiredGapWidthCm.formatCm()).append(" cm\n")
        append("COMMAND: ").append(d.command.name).append("  SPEED: ").append(d.speedPwm).append(" PWM\n")
        append("PATH: ").append(d.pathType.name).append("  ANGLE: ").append(d.pathAngleDeg.formatAngle()).append("°\n")
        append("BOUNDARY: ").append(d.boundaryState.name)
            .append("  NOMINAL ±").append(geometryConfig.nominalCenterOffsetCm.formatCm())
            .append(" cm  EXPANDED ±").append(geometryConfig.expandedCenterOffsetCm.formatCm()).append(" cm\n")
        append("OBSTACLES: ").append(p?.obstacleCountEstimate ?: 0)
            .append("  NEAREST: ").append(p?.nearestObstacleCm?.formatCm() ?: "--").append(" cm\n")
        append("GAP: ").append(p?.bestPath?.minimumGapWidthCm?.formatCm() ?: "--")
            .append(" cm  REQUIRED: ").append((p?.requiredGapWidthCm ?: physicalGeometryConfig.requiredGapWidthCm).formatCm())
            .append(" cm (live clearance margin included)\n")
        append("FREE SPACE: ").append(s.freeLeft.formatScore()).append('/')
            .append(s.freeCenter.formatScore()).append('/').append(s.freeRight.formatScore())
            .append("  UNKNOWN CENTER: ").append(s.unknownCenter.formatScore()).append("\n")
        append("ULTRASONIC: ").append(s.ultrasonicCm?.formatCm() ?: "--").append(" cm\n")
        append("GATES: translate STOP <30 cm; pivot >=24 cm + camera clearance >=22 cm\n")
        val signFresh = s.trafficSign != null && s.trafficSignAgeMs <= TRAFFIC_SIGN_DISPLAY_HOLD_MS
        append("SIGN: ").append(s.trafficSign?.let { "${it.label.uppercase(Locale.US)} ${it.confidence.formatScore()}" } ?: "--")
            .append(if (signFresh) " confirmed age=${s.trafficSignAgeMs}ms" else " unconfirmed/stale").append('\n')
        val candidateFresh = s.trafficSignCandidate != null && s.trafficSignCandidateAgeMs <= TRAFFIC_INFERENCE_INTERVAL_MS * 2L
        append("SIGN CANDIDATE: ").append(if (candidateFresh) s.trafficSignCandidate?.let { "${it.label.uppercase(Locale.US)} ${it.confidence.formatScore()}" } else "--")
            .append(" (raw, not yet actioned)\n")
        append("REASON: ").append(d.reason).append('\n')
        append("ROBOT OUTPUT: ").append(if (usb.connected) "ARMED/AVAILABLE" else "DISCONNECTED")
    }

    private fun buildVirtualTelemetry(
        title: String,
        pose: VirtualScenarioProvider.Pose,
        s: PerceptionSnapshot,
        d: com.example.robotcar.navigation.NavigationDecision
    ): String = buildString {
        val p = s.obstaclePlan
        append("SOURCE: VIRTUAL SCENARIO\n")
        append("SCENE: ").append(title).append('\n')
        append("PATH DISCOVERY: ").append(virtualDiscoveryState).append('\n')
        append("FRAME: ").append(virtualFrameIndex).append("  T=")
            .append(String.format(Locale.US, "%.1fs", virtualTimeUs / 1_000_000.0)).append('\n')
        append("POSE: x=").append(pose.lateralCm.formatCm()).append(" cm  z=")
            .append(pose.forwardCm.formatCm()).append(" cm  yaw=")
            .append(pose.headingDeg.formatAngle()).append("°\n")
        append("ROBOT: 15.0 x 25.5 cm  REQUIRED GAP: ")
            .append(physicalGeometryConfig.requiredGapWidthCm.formatCm()).append(" cm\n")
        append("COMMAND: ").append(d.command.name).append("  SPEED: ").append(d.speedPwm).append(" PWM\n")
        append("PATH: ").append(d.pathType.name).append("  ANGLE: ").append(d.pathAngleDeg.formatAngle()).append("°\n")
        append("BOUNDARY: ").append(d.boundaryState.name)
            .append("  NOMINAL ±").append(geometryConfig.nominalCenterOffsetCm.formatCm())
            .append(" cm  EXPANDED ±").append(geometryConfig.expandedCenterOffsetCm.formatCm()).append(" cm\n")
        append("OBSTACLES: ").append(p?.obstacleCountEstimate ?: 0)
            .append("  NEAREST: ").append(p?.nearestObstacleCm?.formatCm() ?: "--").append(" cm\n")
        append("CENTER GAP: ").append(p?.centerGapWidthCm?.formatCm() ?: "--")
            .append(" cm  REQUIRED: ").append(physicalGeometryConfig.requiredGapWidthCm.formatCm()).append(" cm effective\n")
        append("SELECTED GAP: ").append(p?.bestPath?.minimumGapWidthCm?.formatCm() ?: "--").append(" cm\n")
        append("FREE SPACE: ").append(s.freeLeft.formatScore()).append('/')
            .append(s.freeCenter.formatScore()).append('/').append(s.freeRight.formatScore())
            .append("  UNKNOWN CENTER: ").append(s.unknownCenter.formatScore()).append("\n")
        append("ULTRASONIC: ").append(s.ultrasonicCm?.formatCm() ?: "--").append(" cm\n")
        append("REASON: ").append(d.reason).append('\n')
        append("TRAFFIC SIGN MODEL: not run on synthetic video; use LIVE CAMERA to verify\n")
        append("ROBOT OUTPUT: DISABLED")
    }

    private fun markStatusDirty() { lastStatusShown = null }

    private fun postStatusIfChanged(text: String) {
        if (text == lastStatusShown) return
        lastStatusShown = text
        runOnUiThread { statusText.text = text }
    }

    private fun updateButtons() {
        armButton.text = if (mode == RobotMode.AUTONOMOUS) "DISARM" else "ARM"
        armButton.isEnabled = !virtualTestActive && !arduinoSafetyHold && ((models.areRequiredModelsLoaded && usb.connected && readyConnectionId == usb.connectionId) || mode == RobotMode.AUTONOMOUS)
        connectUsbButton.text = if (usb.connected) "USB CONNECTED" else "CONNECT USB"
        connectUsbButton.isEnabled = !virtualTestActive
        stopButton.isEnabled = !virtualTestActive || usb.connected
        videoTestButton.text = if (virtualTestActive) "LIVE CAMERA" else "VIDEO TEST"
        videoTestButton.isEnabled = models.areRequiredModelsLoaded
        videoPauseButton.isEnabled = virtualTestActive
        videoStepButton.isEnabled = virtualTestActive
        videoResetButton.isEnabled = virtualTestActive
    }

    override fun onStart() {
        super.onStart()
        if (yawSensor == null) yawSensor = YawSensor(applicationContext, yawTracker)
        yawSensor?.start()
    }

    override fun onStop() {
        yawSensor?.stop()
        videoTestRunner?.pause()
        val wasAutonomous = mode == RobotMode.AUTONOMOUS
        setMode(RobotMode.SAFE, "activity stopped (app backgrounded)")
        if (!wasAutonomous && usb.connected) usb.sendStop()
        super.onStop()
    }

    override fun onDestroy() {
        stopVirtualVideoTest()
        val wasAutonomousAtDestroy = mode == RobotMode.AUTONOMOUS
        setMode(RobotMode.SAFE, "activity destroyed")
        if (!wasAutonomousAtDestroy) try { usb.sendStop() } catch (_: Throwable) {}
        controlExecutor.shutdownNow()
        modelExecutor.shutdownNow()
        ioExecutor.shutdownNow()
        cameraExecutor.shutdownNow()
        awaitShutdown(controlExecutor)
        awaitShutdown(modelExecutor)
        awaitShutdown(ioExecutor)
        awaitShutdown(cameraExecutor)
        try { freeSpacePreprocessor.close() } catch (_: Throwable) {}
        try { trafficSignPreprocessor.close() } catch (_: Throwable) {}
        models.close()
        usb.close()
        DecisionLog.close()
        super.onDestroy()
    }

    private fun awaitShutdown(executor: java.util.concurrent.ExecutorService) {
        try { executor.awaitTermination(750L, TimeUnit.MILLISECONDS) }
        catch (_: InterruptedException) { Thread.currentThread().interrupt() }
    }
}

private fun Long.ageText(): String = if (this == Long.MAX_VALUE) "n/a" else "${this}ms"
private fun Float.formatScore(): String = String.format(Locale.US, "%.2f", this)
private fun Float.formatCm(): String = String.format(Locale.US, "%.1f", this)
private fun Float.formatAngle(): String = String.format(Locale.US, "%+.1f", this)
private fun Bitmap.recycleSafely() {
    try { if (!isRecycled) recycle() } catch (_: Throwable) {}
}

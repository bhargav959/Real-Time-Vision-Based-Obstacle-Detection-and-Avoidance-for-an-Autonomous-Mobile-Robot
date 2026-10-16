package com.example.robotcar.video

import android.content.Context
import android.graphics.Bitmap
import android.media.MediaMetadataRetriever
import java.io.File
import java.util.concurrent.atomic.AtomicBoolean

/**
 * Offline MP4 frame source for the Android vision pipeline.
 * Frames are decoded on a worker thread and delivered synchronously so slow inference cannot
 * build an unbounded queue.
 */
class VideoTestRunner(
    context: Context,
    private val resourceId: Int,
    private val onFrame: (bitmap: Bitmap, frameIndex: Int, timeUs: Long) -> Unit,
    private val onCompleted: () -> Unit,
    private val onError: (Throwable) -> Unit
) {
    companion object {
        private const val FRAME_INTERVAL_US = 100_000L // 10 virtual FPS
    }

    private val appContext = context.applicationContext
    private val running = AtomicBoolean(false)
    private val paused = AtomicBoolean(true)
    private val workerLock = Any()
    private var worker: Thread? = null
    private var nextTimeUs = 0L
    private var frameIndex = 0
    private var durationUs = 0L
    private var retriever: MediaMetadataRetriever? = null
    private var videoFile: File? = null

    @Volatile
    private var currentTimeUs = 0L

    fun durationUs(): Long = durationUs
    fun currentTimeUs(): Long = currentTimeUs
    fun isRunning(): Boolean = running.get()
    fun isPaused(): Boolean = paused.get()

    fun start() {
        synchronized(workerLock) {
            if (running.get()) {
                paused.set(false)
                return
            }
            prepareRetriever()
            running.set(true)
            paused.set(false)
            worker = Thread(::runLoop, "video-test-runner").apply { start() }
        }
    }

    fun pause() {
        paused.set(true)
    }

    fun reset() {
        pause()
        nextTimeUs = 0L
        frameIndex = 0
        currentTimeUs = 0L
    }

    fun step() {
        synchronized(workerLock) {
            if (retriever == null) prepareRetriever()
            running.set(true)
            paused.set(true)
            if (worker == null || worker?.isAlive != true) {
                worker = Thread(::stepOnce, "video-test-stepper").apply { start() }
            }
        }
    }

    fun release() {
        running.set(false)
        paused.set(true)
        synchronized(workerLock) {
            try { worker?.interrupt() } catch (_: Throwable) {}
            worker = null
            retriever?.release()
            retriever = null
        }
    }

    private fun prepareRetriever() {
        retriever?.release()
        val source = ensureVideoFile()
        retriever = MediaMetadataRetriever().also { it.setDataSource(source.absolutePath) }
        durationUs = retriever!!.extractMetadata(MediaMetadataRetriever.METADATA_KEY_DURATION)
            ?.toLongOrNull()?.times(1000L) ?: 0L
    }

    private fun ensureVideoFile(): File {
        videoFile?.let { if (it.exists() && it.length() > 0L) return it }
        val target = File(appContext.cacheDir, "robot_virtual_test.mp4")
        appContext.resources.openRawResource(resourceId).use { input ->
            target.outputStream().use { output -> input.copyTo(output, 64 * 1024) }
        }
        videoFile = target
        return target
    }

    private fun runLoop() {
        try {
            while (running.get()) {
                while (paused.get() && running.get()) Thread.sleep(20L)
                if (!running.get()) break
                if (!emitCurrentFrame()) {
                    running.set(false)
                    paused.set(true)
                    onCompleted()
                    break
                }
                nextTimeUs += FRAME_INTERVAL_US
                frameIndex++
                Thread.sleep(5L)
            }
        } catch (t: InterruptedException) {
            Thread.currentThread().interrupt()
        } catch (t: Throwable) {
            running.set(false)
            paused.set(true)
            onError(t)
        }
    }

    private fun stepOnce() {
        try {
            if (!emitCurrentFrame()) {
                onCompleted()
                return
            }
            nextTimeUs += FRAME_INTERVAL_US
            frameIndex++
        } catch (t: Throwable) {
            onError(t)
        }
    }

    private fun emitCurrentFrame(): Boolean {
        val r = retriever ?: return false
        if (durationUs > 0L && nextTimeUs > durationUs) return false
        val bitmap = r.getFrameAtTime(nextTimeUs, MediaMetadataRetriever.OPTION_CLOSEST)
            ?: return false
        currentTimeUs = nextTimeUs
        onFrame(bitmap, frameIndex, nextTimeUs)
        return true
    }
}

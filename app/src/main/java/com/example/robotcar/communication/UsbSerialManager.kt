package com.example.robotcar.communication

import android.app.PendingIntent
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.hardware.usb.UsbManager
import android.util.Log
import androidx.core.content.ContextCompat
import com.example.robotcar.logging.DecisionLog
import com.hoho.android.usbserial.driver.UsbSerialPort
import com.hoho.android.usbserial.driver.UsbSerialProber
import java.nio.charset.StandardCharsets
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import java.util.concurrent.RejectedExecutionException

/**
 * USB Host serial transport.
 * TX protocol: COMMAND,SPEED\n, e.g. F,150 or SR,95 or S,0.
 * RX telemetry may include DISTANCE=42.5.
 */
class UsbSerialManager(private val context: Context) : AutoCloseable {
    companion object {
        private const val TAG = "UsbSerialManager"
        private const val BAUD_RATE = SerialProtocol.BAUD_RATE
        private const val WRITE_TIMEOUT_MS = 120
        private const val READ_TIMEOUT_MS = 100
    }

    private val usbManager = context.getSystemService(Context.USB_SERVICE) as UsbManager
    private val permissionAction = "${context.packageName}.USB_PERMISSION"
    private var readerExecutor: ExecutorService = Executors.newSingleThreadExecutor()
    private val connectionExecutor: ExecutorService = Executors.newSingleThreadExecutor()
    @Volatile private var port: UsbSerialPort? = null

    @Volatile
    var connected: Boolean = false
        private set

    /** Incremented on every successful (re)connect; lets callers detect a reconnect (Arduino resets). */
    @Volatile
    var connectionId: Int = 0
        private set

    @Volatile
    private var closed = false

    @Volatile
    var onTelemetryLine: ((String) -> Unit)? = null

    private val permissionReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context?, intent: Intent?) {
            when (intent?.action) {
                permissionAction -> {
                    val granted = intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)
                    if (granted && !closed) {
                        try {
                            connectionExecutor.execute { connectFirstSupported() }
                        } catch (e: RejectedExecutionException) {
                            Log.w(TAG, "USB permission arrived after manager shutdown")
                        }
                    } else {
                        Log.w(TAG, "USB permission denied")
                    }
                }
                UsbManager.ACTION_USB_DEVICE_DETACHED -> {
                    closePortOnly()
                    DecisionLog.log("USB", "device detached; serial link closed")
                }
            }
        }
    }

    init {
        val filter = IntentFilter(permissionAction).apply {
            addAction(UsbManager.ACTION_USB_DEVICE_DETACHED)
        }
        ContextCompat.registerReceiver(
            context,
            permissionReceiver,
            filter,
            ContextCompat.RECEIVER_NOT_EXPORTED
        )
    }

    @Synchronized
    fun connectFirstSupported(): Boolean {
        if (closed) return false

        val drivers = UsbSerialProber.getDefaultProber().findAllDrivers(usbManager)
        val driver = drivers.firstOrNull() ?: run {
            closePortOnly()
            return false
        }

        val device = driver.device
        if (!usbManager.hasPermission(device)) {
            val flags = PendingIntent.FLAG_UPDATE_CURRENT or PendingIntent.FLAG_IMMUTABLE
            val permissionIntent = PendingIntent.getBroadcast(
                context,
                0,
                Intent(permissionAction).setPackage(context.packageName),
                flags
            )
            usbManager.requestPermission(device, permissionIntent)
            return false
        }

        val newPort = driver.ports.firstOrNull() ?: run {
            closePortOnly()
            return false
        }

        return try {
            closePortOnly()
            val connection = usbManager.openDevice(device) ?: error("Could not open USB device")
            newPort.open(connection)
            newPort.setParameters(
                BAUD_RATE,
                8,
                UsbSerialPort.STOPBITS_1,
                UsbSerialPort.PARITY_NONE
            )
            newPort.dtr = true
            newPort.rts = true
            port = newPort
            connectionId++
            connected = true
            startReader(newPort)
            Log.i(TAG, "Connected: ${device.deviceName} ${device.vendorId}:${device.productId}")
            true
        } catch (t: Throwable) {
            Log.e(TAG, "USB serial open failed", t)
            try { newPort.close() } catch (_: Throwable) {}
            connected = false
            false
        }
    }

    fun send(command: String, speedPwm: Int): Boolean {
        val record = SerialProtocol.encodeMotion(command, speedPwm) ?: run {
            Log.w(TAG, "Rejected unsupported motion command: $command")
            return false
        }
        return sendRaw(record)
    }

    fun send(command: String): Boolean {
        val request = SerialProtocol.encodeRequest(command) ?: run {
            Log.w(TAG, "Rejected unsupported telemetry request: $command")
            return false
        }
        return sendRaw(request)
    }

    fun sendStop(): Boolean = send("S", 0)

    /**
     * All writes share this monitor so commands and telemetry requests cannot interleave at byte level.
     * Callers in the control loop write directly; they do not enqueue 10 Hz stale movement commands.
     */
    @Synchronized
    private fun sendRaw(command: String): Boolean {
        if (closed || !connected) return false
        val normalized = command.trim()
        if (normalized.isEmpty() || normalized.any { it == '\n' || it == '\r' }) {
            Log.w(TAG, "Rejected invalid serial command")
            return false
        }
        val p = port ?: return false
        return try {
            p.write((normalized + "\n").toByteArray(StandardCharsets.US_ASCII), WRITE_TIMEOUT_MS)
            true
        } catch (t: Throwable) {
            Log.e(TAG, "USB write failed", t)
            DecisionLog.log("USB", "write failed (${t.message}); link closed")
            closePortOnly()
            false
        }
    }

    private fun startReader(activePort: UsbSerialPort) {
        readerExecutor.shutdownNow()
        readerExecutor = Executors.newSingleThreadExecutor()
        readerExecutor.execute {
            val buffer = ByteArray(256)
            val line = StringBuilder()
            var readFailure: Throwable? = null
            while (!closed && connected && port === activePort) {
                try {
                    val count = activePort.read(buffer, READ_TIMEOUT_MS)
                    if (count <= 0) continue
                    for (i in 0 until count) {
                        val c = buffer[i].toInt().and(0xFF).toChar()
                        if (c == '\n' || c == '\r') {
                            if (line.isNotEmpty()) {
                                val message = line.toString().trim()
                                line.setLength(0)
                                if (message.isNotEmpty() && connected && port === activePort) {
                                    try {
                                        onTelemetryLine?.invoke(message)
                                    } catch (callbackError: Throwable) {
                                        // A UI/parser callback must not silently kill the serial reader.
                                        Log.e(TAG, "Telemetry callback failed", callbackError)
                                    }
                                }
                            }
                        } else if (line.length < 160) {
                            line.append(c)
                        } else {
                            line.setLength(0)
                            Log.w(TAG, "Discarded overlong telemetry line")
                        }
                    }
                } catch (t: Throwable) {
                    if (!closed && connected && port === activePort) readFailure = t
                    break
                }
            }
            readFailure?.let { failure ->
                Log.e(TAG, "USB reader terminated; marking connection disconnected", failure)
                DecisionLog.log("USB", "reader failed (${failure.message}); link closed and robot must re-arm")
                closeAfterReaderFailure(activePort)
            }
        }
    }

    @Synchronized
    private fun closeAfterReaderFailure(activePort: UsbSerialPort) {
        // Do not let a stale reader from an old connection close a newly opened port.
        if (port !== activePort) return
        connected = false
        port = null
        try { activePort.close() } catch (_: Throwable) {}
        readerExecutor.shutdownNow()
    }

    @Synchronized
    private fun closePortOnly() {
        connected = false
        val oldPort = port
        port = null
        try { oldPort?.close() } catch (_: Throwable) {}
        readerExecutor.shutdownNow()
    }

    @Synchronized
    override fun close() {
        if (closed) return
        closed = true
        closePortOnly()
        connectionExecutor.shutdownNow()
        try { context.unregisterReceiver(permissionReceiver) } catch (_: Throwable) {}
    }
}

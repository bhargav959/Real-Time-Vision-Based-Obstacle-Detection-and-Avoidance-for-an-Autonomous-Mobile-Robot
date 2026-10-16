# Runtime Performance / GC Optimization

This revision addresses the repeated Android GC messages observed while the camera and AI pipelines were running.

## Changes

- Camera analysis now uses a bounded latest-frame pipeline and avoids unbounded queues. The previous implementation copied the entire plane into a new `ByteArray` on every camera frame.
- Free-space and traffic-sign preprocessing now reuses a model-sized letterbox `Bitmap`, `Canvas`, pixel `IntArray`, and model `FloatArray` for each model input size.
- The reusable preprocessor draws the source `Bitmap` directly into the letterbox destination, avoiding a temporary scaled `Bitmap`.
- Traffic-sign TFLite/LiteRT inference now reuses its direct input/output buffers and output `FloatArray` rather than allocating a new output array for every inference.
- The existing `KEEP_ONLY_LATEST` camera backpressure strategy and single model executor remain in place to prevent work from building up.
- `compileSdk` is set to 37 to satisfy the current `androidx.core:core-ktx:1.19.0` AAR metadata requirement.

## Expected effect

The app should allocate substantially less short-lived memory during camera analysis and model inference. This should reduce GC frequency and reduce latency spikes visible as camera/inference stutter.

## Safety behavior unchanged

These changes do not alter the navigation priority rules or motor command protocol. The robot remains fail-safe when the camera, models, or USB link are unavailable.

## Verification note

A full Android Gradle build cannot be executed in the current packaging environment because the required Android SDK/Gradle distribution is not installed locally. The project should be opened and synced in Android Studio with Android API 37 installed, then `Build > Rebuild Project` should be run on the target machine.

## Historical revision notes

Earlier performance work also removed boxed mask-decoding values, unnecessary rotated frame copies, and redundant UI posts. Remaining periodic GC lines in logcat are normal background collections;
`handleResized abandoned!` is a framework view-root message and is unrelated to this app's code.

Still open (optional): `ImageProxy.toBitmap()` allocates one frame-sized Bitmap per model frame (only when a
model run is due, ~every 140 ms). Removing it would need a direct YUV->letterbox converter.
Verify on the device: Android Studio Profiler > Memory, or `adb shell dumpsys meminfo com.example.robotcar`.

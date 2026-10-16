# Review Findings - 2026-10-11 - 29-Class Traffic Sign Model

## Current package changes

1. Replaced `app/src/main/assets/models/traffic_sign.tflite` with the supplied `best(2).tflite` model.
2. Updated the Kotlin traffic-sign decoder to the combined 29-class label order used by the training notebook.
3. Updated TFLite runtime output-shape validation from 9 attributes (5 classes) to 33 attributes (4 box values + 29 class scores).
4. Preserved dynamic input layout handling. The supplied model is expected to use float32 NCHW `[1,3,640,640]`; preprocessing selects NCHW after model load.
5. Updated STOP / NO ENTRY safety prioritization for the merged class IDs: `no_entry` = 9 and `stop` = 21.
6. Updated `traffic_sign_labels.txt`, `app/src/main/assets/models/README.txt`, `docs/MODEL_INTEGRATION.md`, and `docs/ARCHITECTURE.md`.
7. Bumped Android app version to `0.5.12-29-sign-model` / versionCode 17.

## Model assets

### `free_space.onnx`

SHA-256: `3e6342d259d148473ec00f4920faba1b7251bfeb70ada5e7459bcd5b018d94d9`

Existing free-space model, unchanged in this update.

### `traffic_sign.tflite`

SHA-256: `c7fcc15ced4db682db55c7b68254e365b691ef2ecd5381482359ab0f4a1d9132`

Copied from the user-uploaded `best(2).tflite`. Packaged target size: 9,920,526 bytes. Expected signature: float32 `[1,3,640,640]` input and `[1,33,8400]` output.

## Runtime behavior

- `stop` (class 21): STOP latch.
- `no_entry` (class 9): STOP if centered; route-direction veto when seen to one side.
- Other 27 signs: recognized/displayed by traffic-sign inference, but do not directly command movement.
- The existing free-space model, ultrasonic/USB interlocks, temporal sign filter, planner, and Arduino protocol are otherwise unchanged.

## Validation limitation

The packaged model bytes and class list were checked, and source/configuration references were updated. An Android Gradle build and on-device inference test were not completed in this environment. The first run in Android Studio should confirm the model load log reports input `[1, 3, 640, 640]` and output `[1, 33, 8400]`. Test all behavior with the drive wheels lifted before any physical run; this is a robotics prototype, not a road-vehicle control system.

#!/bin/sh
# RobotCarAI local Gradle launcher.
# The repository does not vendor the official gradle-wrapper.jar.
# Open the project in Android Studio and sync, or install Gradle 9.6+ locally.
set -eu
if ! command -v gradle >/dev/null 2>&1; then
  echo "Gradle 9.6+ is required for command-line builds. Open this project in Android Studio and sync, or install Gradle locally." >&2
  exit 1
fi
exec gradle "$@"

@echo off
rem RobotCarAI local Gradle launcher.
rem The repository does not vendor the official gradle-wrapper.jar.
rem Open the project in Android Studio and sync, or install Gradle 9.6+ locally.
where gradle >nul 2>nul
if errorlevel 1 (
  echo Gradle 9.6+ is required for command-line builds. Open this project in Android Studio and sync, or install Gradle locally. 1>&2
  exit /b 1
)
call gradle %*

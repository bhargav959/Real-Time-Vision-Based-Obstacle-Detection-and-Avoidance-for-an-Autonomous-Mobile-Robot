# v0.5.19 turn/reverse tuning
- Turn speed: TURN_SPEED_FACTOR 1.20->1.45, ARDUINO_MAX_PWM 48->58, SLIGHT_TURN_OUTER_MIN_PWM 40->48, scan pivot PWM 31/41 -> 37/49.
- Turn angle: pivot threshold 28/24 deg -> 22/18 deg (enter/hold); slight-turn differential is now 48 vs 25 PWM.
- Reverse: BACKOFF_PWM 30->25 (equals MIN_MOTOR_PWM floor; going lower needs a separate reverse floor and risks stalling). Firmware BACKOFF_MAX_PWM mirrored.
- NO_ECHO cap (30) unchanged. Tests updated; 88 pass.

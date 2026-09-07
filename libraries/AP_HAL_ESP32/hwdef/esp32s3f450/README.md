# F450 ESP32-S3 BMI323 bring-up target

Build with `./waf configure --board esp32s3f450` then `./waf copter`.
The `esp32s3` prefix is required by the Waf chip/toolchain selection.

This target matches the custom carrier that currently runs MadFlight:

- ESP32-S3-WROOM-1-N16R8
- BMI323 on SPI2: CS 10, MOSI 11, MISO 12, SCLK 13
- BMP280 and QMC5883P on I2C: SDA 8, SCL 9
- NEO-M10N UART: RX 15, TX 16
- FS-iA10B iBUS receive-only on GPIO17; GPIO18 remains unused
- PWM ESC outputs: GPIO4, GPIO7, GPIO6, GPIO5 in ArduPilot Motor1..4 order

This is a bench bring-up target, not a flight-qualified target. The BMI323
backend uses the register sequence already validated in MadFlight, while
estimation, calibration and flight control remain entirely inside ArduPilot.

Use the USB connector backed by the USB-to-UART bridge for SERIAL0/MAVLink.
Native USB on GPIO19/GPIO20 is not configured by this target.

Initial validation must be performed with propellers removed:

1. Confirm Mission Planner heartbeat and board identity.
2. Confirm BMI323 detection and calibrate accelerometers.
3. Check artificial-horizon roll, pitch and yaw signs by hand.
4. Confirm BMP280, QMC5883P and GPS health.
5. Calibrate radio and verify iBUS failsafe.
6. Run Mission Planner Motor Test at minimum power and verify physical motor,
   direction and displayed position before allowing arming.

Do not reuse MadFlight accelerometer, compass or PID calibration values.

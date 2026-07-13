# Axion
### A system built using the ESP32-S3

**Functionalities:**
- [x] Rotation and acceleration measurements
- [x] Cell tower communication
- [x] GPS tracking
- [x] Body temperature monitoring
- [x] Haptic feedback (user warnings)
- [x] Pulse oximeter (MAX30102)

**Future plans:**
- [ ] ~Implement Kalman filter for speed and position~ (Seems no longer necessary due to the accuracy of the sensors, but could help either way)
- [ ] Create a phone app (distant future)

## Quick start

1. **Install ESP-IDF v5.x** (the project has been validated against the v5 line; v4.x is no longer supported by `idf_component.yml`).

2. **Clone and prepare secrets.** The project no longer ships hard-coded device credentials. Copy the template and edit it:
   ```bash
   cp main/secrets.h.example main/secrets.h
   # edit main/secrets.h with your SIM PIN and alert phone numbers
   ```
   `main/secrets.h` is gitignored — it will never be committed by accident.

3. **Set the target and build:**
   ```bash
   idf.py set-target esp32s3
   idf.py build flash monitor
   ```

### Secrets

- `secrets.h.example` is the committed template with placeholder values.
- `secrets.h` is the real per-device file, gitignored.
- `axion.cpp` has a `static_assert` that refuses to compile if `ALERT_PHONE_1` doesn't start with `+` (i.e. if you forgot to edit the template).

Up to two recipients are supported (`ALERT_PHONE_1`, `ALERT_PHONE_2`); leave the unused one as `""`.

### What modules are used?
The main brain of the system is Espressif's ESP32-S3, which I chose because I wanted to learn a new platform and dive deeper into low-level embedded development. That, and I got tired of Arduino. The list may change at some point.

There are (so far) 5 functional modules used in this project:
- [GY-521(MPU6050)](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/) - a 6-axis motion tracking (acceleration and rotation) chip
- [A7670E](https://www.simcom.com/product/A7670X.html) - LTE and GNSS capable low-power chip
- [DS18B20](https://www.adafruit.com/product/381) - A temperature sensor
- [MAX30102](https://www.maximintegrated.com/en/products/interface/sensor-interface/MAX30102.html) - Pulse oximeter / heart-rate sensor
- A generic coin vibrator motor

### Strengths and weaknesses
This section will be filled once the final prototype is tested thoroughly.

### Things to keep in mind
- The [MPU6050 library by Espressif](https://components.espressif.com/components/espressif/mpu6050/versions/1.2.0) uses a severely outdated and now deprecated i2c library, hence the use of the jrowberg I2Cdev library instead.
- Kalman filter may not be entirely necessary, because the readings are rather accurate for relative rotation and position, however, it might be useful for velocity.
 - Relative rotation means the roll/pitch/yaw depends solely on the initial position the device was in, meaning if someone turned it upside-down then started it, the device will think it's upright.
  - To remedy this, magnetometers may be introduced to set the initial values of the MPU6050 (if such a thing is possible).
- The MAX30102 driver does no calibration of the SpO2 R-curve; the default `49.7 * R` linearization is coarse. For clinical accuracy you will need to calibrate against a reference oximeter and replace the formula in `max30102_alg_spo2()`. The HR estimator is autocorrelation-based and is robust but slow (~5 s for a full 128-sample window at 40 ms sample period).

## References and credits
### Libraries
- [OWB](https://github.com/DavidAntliff/esp32-owb) to DavidAntliff
- [DS18B20](https://github.com/DavidAntliff/esp32-ds18b20) to DavidAntliff
- [I2Cdev and MPU6050](https://github.com/jrowberg/i2cdevlib/tree/master/ESP32_ESP-IDF) to jrowberg
- The MAX30102 driver + algorithm are adapted from [Gabriel-Gardin/max30102_esp32_oximeter](https://github.com/Gabriel-Gardin/max30102_esp32_oximeter).
- The entire [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32/) dev kit to ESPRESSIF

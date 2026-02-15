# Axion
### A system built using the ESP32-S3

Note: This is still in ==early development==

**Functionalities:**
- [x] Rotation and acceleration measurements
- [x] Cell tower communication
- [x] GPS tracking
- [x] Body temperature monitoring
- [ ] Haptic feedback (user warnings)

**Future plans:**
- [ ] Implement Kalman filter for speed and position
- [ ] Create a phone app (distant future)

### What modules are used?
The main brain of the system is Espressif's ESP32-S3, which I chose because I wanted to learn a new platform and dive deeper into low-level embedded development. That, and I got tired of Arduino. The list may change at some point.

There are (so far) 4 functional modules that I have used in this project:
- [GY-521(MPU6050)](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/) - a 6-axis motion tracking (acceleration and rotation) chip
- [A7670E](https://www.simcom.com/product/A7670X.html) - LTE and GNSS capable low-power chip
- [DS18B20](https://www.adafruit.com/product/381) - A temperature sensor
- A generic coin vibrator motor 

### Strengths and weaknesses
This section will be filled once the final prototype is tested thoroughly.

### Things to keep in mind
- The [MPU6050 library by Espressif](https://components.espressif.com/components/espressif/mpu6050/versions/1.2.0) uses a severely outdated and now deprecated i2c library, hence the need ~to write my own drivers.~ find a simple and great library to use.
- ~The A7670E communicates via serial (RX/TX), and has its own series of commands which define behavior. Reading from and writing to it is rather easy, but can become unreadable quickly as time goes on. The best solution is to keep it separated as its own component.~
- Kalman filter may not be entirely necessary, because the readings are rather accurate for relative rotation and position, however, it might be useful for velocity 
  - Relative rotation means the roll/pitch/yaw depends solely on the initial position the device was in, meaning if someone turned it upside-down then started it, the device will think it's upright
  - To remedy this, magnetometers may be introduced to set the initial values of the MPU6050 (if such a thing is possible)
- The temperature sensor is finicky at best and unusable at worst. The library was built for ESP-IDF v5.0.1, which might be a source of issues, but I believe the culprit is potentially a knock-off DS18B20
  - It detects it via the OWB, but no data could ever be transferred despite using a resistor as pointed out by many instructions

## References and credits
### Libraries
- [OWB](https://github.com/DavidAntliff/esp32-owb) to DavidAntliff
- [DS18B20](https://github.com/DavidAntliff/esp32-ds18b20) to DavidAntliff
- [I2Cdev and MPU6050](https://github.com/jrowberg/i2cdevlib/tree/master/ESP32_ESP-IDF) to jrowberg
- The entire [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32/) dev kit to ESPRESSIF

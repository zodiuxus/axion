# Axion
## For extreme sport enthusiasts
### A system built using the ESP32-S3

Note: This is still in ==early development==

**Functionalities:**
- [x] Rotation and acceleration measurements
- [x] Cell tower communication
- [x] GPS tracking
- [ ] Body temperature monitoring
- [ ] Haptic feedback (user warnings)

**Future plans:**
- [ ] Write own drivers for the MPU6050 
- [ ] Write own drivers for the A7670E
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
- The [MPU6050 library by Espressif](https://components.espressif.com/components/espressif/mpu6050/versions/1.2.0) uses a severely outdated and now deprecated i2c library, hence the need to write my own drivers.
- The A7670E communicates via serial (RX/TX), and has its own series of commands which define behavior. Reading from and writing to it is rather easy, but can become unreadable quickly as time goes on. The best solution is to keep it separated as its own component.

#pragma once
// yes, i know this isn't how it should
// be normally done. i did it this way
// because i was running out of time.
#include <cmath>
#ifndef AXION_LOGIC_H
#define AXION_LOGIC_H

#include "values.h"
#include "driver/gpio.h"
#include "secrets.h"
#include "freertos/semphr.h"

// I2C (MPU6050)
#define PIN_I2C_SDA         5
#define PIN_I2C_SCL         6
#define PIN_MPU_INT         7
#define I2C_MASTER_FREQ_HZ  400000

// UART1 (A7670E)
#define PIN_UART_TX         17
#define PIN_UART_RX         18
#define PIN_UART_RTS        15
#define PIN_UART_CTS        16
#define UART_TIMEOUT_MS     2000

#define PIN_BUZZER          GPIO_NUM_10
#define PIN_BTN_CANCEL      GPIO_NUM_9
#define PIN_LED_STATUS      GPIO_NUM_8

#define BUF_UART    512 
#define PORT_UART   UART_NUM_1

// Fall detection threshold (native ±180° range; fallen if |roll| >= this)
#define FALL_ANGLE_THRESHOLD 150.0f
#define WARMUP_TIME_MS   12000
#define WARNING_MS       5000
#define ALERT_MS         5000
#define BUZZER_PERIOD_MS 500

// Temperature thresholds (°C)
#define TEMP_HYPO_DELTA   0.5f
#define TEMP_HYPER_DELTA  1.0f
#define TEMP_MIN_VALID    35.0f  // below this, sensor hasn't reached body temp yet

// Speed thresholds (m/s)
#define SPEED_STOPPED     0.5f    // below this = considered stopped

#define AT_READY BIT0
#define MPU_READY BIT0
#define I2C_READY BIT0

#include <MPU6050.h>

static char response_buf[BUF_UART];
static auto TAG = "Axion";

// defined and instantiated ONCE... I hope
inline MPU6050 mpu = MPU6050();
inline Quaternion q;
inline VectorFloat gravity;
inline float ypr[3];
inline uint16_t packetSize = 42;
inline uint16_t fifoCount;
inline uint8_t fifoBuffer[64];
inline uint8_t mpuIntStatus;

inline double lat = 0.0;
inline double lon = 0.0;
inline float alt = 0.0;
inline float speed = 0.0;
inline bool gps_fix_valid = false;

inline int spo2 = 0;
inline int heart_rate = 0;

inline bool debugOutput = false;

inline EventGroupHandle_t atReady;
inline EventGroupHandle_t mpuReady;
inline EventGroupHandle_t i2cReady;

inline SemaphoreHandle_t modem_mutex = nullptr;
inline SemaphoreHandle_t sensor_mutex = nullptr;

// function definitions
bool send_at_command(const char* cmd, int max_timeout_ms, const char* expected_response);
void get_coords();
void get_rpy();
void status_led_task();
#endif //AXION_LOGIC_H

#pragma once
// yes, i know this isn't how it should
// be normally done. i did it this way
// because i was running out of time.
#include <cmath>
#ifndef AXION_LOGIC_H
#define AXION_LOGIC_H
// multi-file definitions
#define BUF_UART 225
#define PORT_UART UART_NUM_1

#include <MPU6050.h>

static char response_buf[BUF_UART];
static auto TAG = "axion test";

// defined and instantiated ONCE... I hope
inline MPU6050 mpu = MPU6050();
inline Quaternion q;
inline VectorFloat gravity;
inline float ypr[3];
inline uint16_t packetSize = 42;
inline uint16_t fifoCount;
inline uint8_t fifoBuffer[64];
inline uint8_t mpuIntStatus;

inline float lat = 0.0;
inline float lon = 0.0;
inline float alt = 0.0;
inline float speed = 0.0;

// function definitions
bool send_at_command(const char* cmd, int max_timeout_ms, const char* expected_response);
void get_coords();
#endif //AXION_LOGIC_H

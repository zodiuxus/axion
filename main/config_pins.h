#pragma once
/**
 * config_pins.h - Hardware wiring: GPIOs, peripheral ports, electrical
 * parameters. The only config header that pulls in ESP-IDF driver headers.
 */
#ifndef AXION_CONFIG_PINS_H
#define AXION_CONFIG_PINS_H

#include "soc/gpio_num.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- I2C bus (shared by MPU6050 + MAX30102) -------------------------- */
#define PIN_I2C_SDA             GPIO_NUM_5
#define PIN_I2C_SCL             GPIO_NUM_6
#define PIN_MPU_INT             GPIO_NUM_7
/* 400 kHz: the 200 Hz MPU INT path + MAX30102 bursts saturate 100 kHz. */
#define I2C_MASTER_FREQ_HZ      400000U

/* ---- UART1 (A7670E modem) --------------------------------------------- */
#define PIN_UART_TX             GPIO_NUM_17
#define PIN_UART_RX             GPIO_NUM_18
#define PIN_UART_RTS            GPIO_NUM_15
#define PIN_UART_CTS            GPIO_NUM_16
#define UART_MODEM_PORT         UART_NUM_1
#define UART_MODEM_BAUD         115200U
/* RX/TX driver ring buffer size; also the AT response staging buffer size. */
#define UART_BUF_SIZE           512U

/* ---- 1-Wire (DS18B20 body temperature) -------------------------------- */
#define PIN_DS18B20             GPIO_NUM_4

/* ---- Haptic / buzzer feedback ------------------------------------------ */
#define PIN_BUZZER              GPIO_NUM_10

/* ---- Green status LED (JTAG pins free - no JTAG debug used) ------------ */
#define PIN_GREEN_LED           GPIO_NUM_42

/* ---- MAX30102 INT (open-drain, active-low, FIFO_A_FULL) ---------------- */
#define PIN_MAX30102_INT        GPIO_NUM_8

/* ---- Alert-abort button (active-low, internal pull-up) ------------------ */
#define PIN_ALERT_BUTTON        GPIO_NUM_9

#ifdef __cplusplus
}
#endif

#endif /* AXION_CONFIG_PINS_H */

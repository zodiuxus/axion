#pragma once
/**
 * max30102.h — Maxim MAX30102 pulse oximeter driver (clean component).
 *
 * Adapted from Gabriel-Gardin/max30102_esp32_oximeter with the following
 * changes:
 *   - Removed WiFi/MQTT coupling (this is a driver, not an app).
 *   - I2C port is passed in by the caller (no global).
 *   - `max_config` is no longer a global defined in a header (that was
 *     an ODR violation waiting to happen); instead a default is provided
 *     by max30102_default_config() and the caller may override fields.
 *   - Fixed `read_max30102_fifo` — the reference used `+=` and silently
 *     accumulated garbage instead of overwriting.
 *   - Uses i2c_master_write_to_device / i2c_master_read_from_device
 *     helpers (cleaner than manual i2c_cmd_link).
 *
 * The driver does NOT install the I2C bus — the caller is responsible
 * for that. In Axion, the MPU6050 setup task installs I2C_NUM_0 first,
 * then max30102_init() attaches to the same bus.
 */
#ifndef MAX30102_H
#define MAX30102_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX30102_I2C_ADDR        0x57

/* ---- Register map --------------------------------------------------- */
#define REG_INTR_STATUS_1        0x00
#define REG_INTR_STATUS_2        0x01
#define REG_INTR_ENABLE_1        0x02
#define REG_INTR_ENABLE_2        0x03
#define REG_FIFO_WR_PTR          0x04
#define REG_OVF_COUNTER          0x05
#define REG_FIFO_RD_PTR          0x06
#define REG_FIFO_DATA            0x07
#define REG_FIFO_CONFIG          0x08
#define REG_MODE_CONFIG          0x09
#define REG_SPO2_CONFIG          0x0A
#define REG_LED1_PA              0x0C
#define REG_LED2_PA              0x0D
#define REG_PILOT_PA             0x10
#define REG_MULTI_LED_CTRL1      0x11
#define REG_MULTI_LED_CTRL2      0x12
#define REG_TEMP_INTR            0x1F
#define REG_TEMP_FRAC            0x20
#define REG_TEMP_CONFIG          0x21
#define REG_PROX_INT_THRESH      0x30
#define REG_REV_ID               0xFE
#define REG_PART_ID              0xFF

/* ---- Bitfield-friendly register union type ------------------------- */
typedef struct {
    union { uint8_t data1; struct {
        uint8_t RESERVED     :4;
        uint8_t PROX_INT_EN  :1;
        uint8_t ALC_OVF_EN   :1;
        uint8_t PPG_RDY_EN   :1;
        uint8_t A_FULL_EN    :1;
    } INT_EN_1; };

    union { uint8_t data2; struct {
        uint8_t RESERVED1       :1;
        uint8_t DIE_TEMP_RDY_EN :1;
        uint8_t RESERVED2       :6;
    } INT_EN_2; };

    union { uint8_t data3; struct {
        uint8_t FIFO_WR_PTR :5;
        uint8_t RESERVED    :3;
    } FIFO_WRITE_PTR; };

    union { uint8_t data4; struct {
        uint8_t OVF_COUNTER :5;
        uint8_t RESERVED    :3;
    } OVEF_COUNTER; };

    union { uint8_t data5; struct {
        uint8_t FIFO_RD_PTR :5;
        uint8_t RESERVED    :3;
    } FIFO_READ_PTR; };

    union { uint8_t data6; struct {
        uint8_t FIFO_A_FULL      :4;
        uint8_t FIFO_ROLLOVER_EN :1;
        uint8_t SMP_AVE          :3;
    } FIFO_CONF; };

    union { uint8_t data7; struct {
        uint8_t MODE     :3;
        uint8_t RESERVED :3;
        uint8_t RESET    :1;
        uint8_t SHDN     :1;
    } MODE_CONF; };

    union { uint8_t data8; struct {
        uint8_t LED_PW       :2;
        uint8_t SPO2_SR      :3;
        uint8_t SPO2_ADC_RGE :2;
        uint8_t RESERVED     :1;
    } SPO2_CONF; };

    union { uint8_t data9;  struct { uint8_t LED1_PA; } LED1_PULSE_AMP; };
    union { uint8_t data10; struct { uint8_t LED2_PA; } LED2_PULSE_AMP; };
    union { uint8_t data11; struct { uint8_t PILOT_PA; } PROX_LED_PULS_AMP; };

    union { uint8_t data12; struct {
        uint8_t SLOT1     :3;
        uint8_t RESERVED2 :1;
        uint8_t SLOT2     :3;
        uint8_t RESERVED1 :1;
    } MULTI_LED_CONTROL1; };

    union { uint8_t data13; struct {
        uint8_t SLOT3     :3;
        uint8_t RESERVED2 :1;
        uint8_t SLOT4     :3;
        uint8_t RESERVED1 :1;
    } MULTI_LED_CONTROL2; };
} max30102_config_t;

/* ---- Public API ----------------------------------------------------- */

/** Returns a sensible default config (SPO2 mode, 200 Hz, 215 us LED pw). */
max30102_config_t max30102_default_config(void);

/**
 * Initialize the MAX30102 on the given I2C port. The bus must already
 * be installed (e.g. by i2c_param_config + i2c_driver_install).
 */
esp_err_t max30102_init(i2c_port_t port, const max30102_config_t *cfg);

/**
 * Read one FIFO sample (6 bytes: 3 for RED, 3 for IR).
 * Returns ESP_OK on success. *red and *ir are written as 18-bit values
 * (the sensor's native resolution at SPO2_ADC_RGE=01).
 */
esp_err_t max30102_read_fifo(i2c_port_t port, int32_t *red, int32_t *ir);

/** Trigger a die-temperature conversion, then read it.
 *  Returns the temperature in degrees Celsius (e.g. 25.6). */
esp_err_t max30102_read_temp(i2c_port_t port, float *out_c);

/** Raw register access (handy for debugging). */
esp_err_t max30102_write_reg(i2c_port_t port, uint8_t reg, uint8_t value);
esp_err_t max30102_read_reg(i2c_port_t port, uint8_t reg, uint8_t *buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* MAX30102_H */

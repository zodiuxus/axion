#pragma once
/**
 * max30102.h - Maxim MAX30102 pulse oximeter driver (clean component).
 *
 * Adapted from Gabriel-Gardin/max30102_esp32_oximeter with the following
 * changes:
 *   - Removed WiFi/MQTT coupling (this is a driver, not an app).
 *   - An i2c_master_dev_handle_t is passed in by the caller (no global).
 *   - `max_config` is no longer a global defined in a header (that was
 *     an ODR violation waiting to happen); instead a default is provided
 *     by max30102_default_config() and the caller may override fields.
 *   - Fixed `read_max30102_fifo` - the reference used `+=` and silently
 *     accumulated garbage instead of overwriting.
 *   - Migrated to the modern I2C master driver (driver/i2c_master.h):
 *     i2c_master_transmit / i2c_master_transmit_receive.
 *
 * The driver does NOT install the I2C bus - the caller is responsible
 * for that. In Axion, I2Cdev::installBus() (called from the MPU setup
 * task) creates the shared bus first, then max30102_init() attaches to
 * it via I2Cdev::deviceHandle(MAX30102_I2C_ADDR).
 */
#ifndef MAX30102_H
#define MAX30102_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

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
 * Initialize the MAX30102 on the given I2C device handle. The bus must
 * already be installed and the handle obtained (I2Cdev::installBus +
 * I2Cdev::deviceHandle(MAX30102_I2C_ADDR)).
 */
esp_err_t max30102_init(i2c_master_dev_handle_t dev, const max30102_config_t *cfg);

/**
 * Read one FIFO sample (6 bytes: 3 for RED, 3 for IR).
 * Returns ESP_OK on success. *red and *ir are written as 18-bit values
 * (the sensor's native resolution at SPO2_ADC_RGE=01).
 *
 * Prefer max30102_read_fifo_burst() when servicing the FIFO_A_FULL
 * interrupt - it reads all queued samples in a single I2C transaction
 * instead of N separate ones.
 */
esp_err_t max30102_read_fifo(i2c_master_dev_handle_t dev, int32_t *red, int32_t *ir);

/**
 * Burst-read up to `max_samples` FIFO samples in one I2C transaction.
 * `*out_count` is set to the number of samples actually read (which is
 * the smaller of max_samples and the FIFO occupancy at read time).
 *
 * Caller provides `red_out` / `ir_out` arrays of length `max_samples`.
 * Each sample is an 18-bit unsigned value (the sensor's native width
 * at SPO2_ADC_RGE=01).
 *
 * This is the preferred way to drain the FIFO from the FIFO_A_FULL
 * interrupt handler - it cuts I2C bus traffic by ~6× compared to
 * reading one sample at a time.
 */
esp_err_t max30102_read_fifo_burst(i2c_master_dev_handle_t dev,
                                   int32_t *red_out, int32_t *ir_out,
                                   size_t max_samples, size_t *out_count);

/** Enable / disable the FIFO_A_FULL interrupt (fires when the FIFO is
 *  ~75% full, i.e. 17 unread samples). The INT pin is open-drain and
 *  active-low; the caller is responsible for wiring the GPIO ISR. */
esp_err_t max30102_enable_fifo_a_full_int(i2c_master_dev_handle_t dev, bool enable);

/** Read both interrupt status registers (REG_INTR_STATUS_1/2) and
 *  return them. Reading clears latched bits. Bit 7 of status1 is
 *  A_FULL (FIFO almost full); bit 6 is PPG_RDY; bit 5 is ALC_OVF;
 *  bit 4 is PROX_INT. */
esp_err_t max30102_read_int_status(i2c_master_dev_handle_t dev,
                                   uint8_t *status1, uint8_t *status2);

/** Trigger a die-temperature conversion, then read it.
 *  Returns the temperature in degrees Celsius (e.g. 25.6). */
esp_err_t max30102_read_temp(i2c_master_dev_handle_t dev, float *out_c);

/** Raw register access (handy for debugging). */
esp_err_t max30102_write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t value);
esp_err_t max30102_read_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* MAX30102_H */

/**
 * max30102.c — Driver implementation. See max30102.h for design notes.
 */
#include "max30102.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

max30102_config_t max30102_default_config(void)
{
    /* Build the config field-by-field. C does not allow chained
     * designated initializers like .INT_EN_1.A_FULL_EN = 0, so we
     * construct via the per-register byte views (data1..data13) which
     * are part of the same union. The bit layout below mirrors the
     * struct definitions in max30102.h.
     *
     * Defaults: SPO2 mode, 200 sps, 215 us LED pulse, 4-sample averaging,
     *           FIFO rollover on, ~25.4 mA on both LEDs. */
    max30102_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    /* REG_INTR_ENABLE_1 (0x02): all interrupts off. */
    cfg.data1 = 0x00;
    /* REG_INTR_ENABLE_2 (0x03): die-temp interrupt off. */
    cfg.data2 = 0x00;
    /* FIFO pointers all start at 0. */
    cfg.data3 = 0x00;
    cfg.data4 = 0x00;
    cfg.data5 = 0x00;
    /* REG_FIFO_CONFIG (0x08): SMP_AVE=010 (avg 4), ROLLOVER=1, A_FULL=0. */
    cfg.data6 = (0b010 << 5) | (1 << 4) | 0;
    /* REG_MODE_CONFIG (0x09): SHDN=0, RESET=0, MODE=011 (SPO2). */
    cfg.data7 = 0b011;
    /* REG_SPO2_CONFIG (0x0A): ADC_RGE=01, SR=001 (200 Hz), LED_PW=10. */
    cfg.data8 = (0b01 << 5) | (0b001 << 2) | 0b10;
    /* LED currents. */
    cfg.data9  = 0x24;
    cfg.data10 = 0x24;
    cfg.data11 = 0x7F;
    /* Multi-LED control: all slots disabled in SPO2 mode. */
    cfg.data12 = 0x00;
    cfg.data13 = 0x00;

    return cfg;
}

esp_err_t max30102_write_reg(i2c_port_t port, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return i2c_master_write_to_device(port, MAX30102_I2C_ADDR, buf, sizeof(buf),
                                      pdMS_TO_TICKS(1000));
}

esp_err_t max30102_read_reg(i2c_port_t port, uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_write_read_device(port, MAX30102_I2C_ADDR,
                                        &reg, 1, buf, len,
                                        pdMS_TO_TICKS(1000));
}

esp_err_t max30102_init(i2c_port_t port, const max30102_config_t *cfg)
{
    max30102_config_t local_cfg;
    if (cfg == NULL) {
        local_cfg = max30102_default_config();
        cfg = &local_cfg;
    }

    esp_err_t err;

    /* Reset the device first so we start from a known state. */
    err = max30102_write_reg(port, REG_MODE_CONFIG, 0b01000000); /* RESET=1 */
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Clear interrupt status registers (read-to-clear). */
    uint8_t tmp[2] = {0};
    max30102_read_reg(port, REG_INTR_STATUS_1, tmp, 2);

    /* Write all config registers in the same order the reference did.
     * We access each register via the per-union byte view (dataN). */
    uint8_t regs[][2] = {
        { REG_INTR_ENABLE_2,   cfg->data2 },
        { REG_FIFO_WR_PTR,     cfg->data3 },
        { REG_OVF_COUNTER,     cfg->data4 },
        { REG_FIFO_RD_PTR,     cfg->data5 },
        { REG_FIFO_CONFIG,     cfg->data6 },
        { REG_MODE_CONFIG,     cfg->data7 },
        { REG_SPO2_CONFIG,     cfg->data8 },
        { REG_LED1_PA,         cfg->data9 },
        { REG_LED2_PA,         cfg->data10 },
        { REG_PILOT_PA,        cfg->data11 },
        { REG_MULTI_LED_CTRL1, cfg->data12 },
        { REG_MULTI_LED_CTRL2, cfg->data13 },
    };
    for (size_t i = 0; i < sizeof(regs) / sizeof(regs[0]); ++i) {
        err = max30102_write_reg(port, regs[i][0], regs[i][1]);
        if (err != ESP_OK) return err;
    }

    return ESP_OK;
}

esp_err_t max30102_read_fifo(i2c_port_t port, int32_t *red, int32_t *ir)
{
    if (red == NULL || ir == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t raw[6] = {0};
    uint8_t reg    = REG_FIFO_DATA;
    esp_err_t err = i2c_master_write_read_device(port, MAX30102_I2C_ADDR,
                                                 &reg, 1, raw, sizeof(raw),
                                                 pdMS_TO_TICKS(1000));
    if (err != ESP_OK) return err;

    /* MAX30102 FIFO data is 18-bit, MSB first, top 2 bits always 0. */
    *red = ((int32_t)(raw[0] & 0x03) << 16) |
           ((int32_t)raw[1] << 8) |
            (int32_t)raw[2];
    *ir  = ((int32_t)(raw[3] & 0x03) << 16) |
           ((int32_t)raw[4] << 8) |
            (int32_t)raw[5];
    return ESP_OK;
}

esp_err_t max30102_read_temp(i2c_port_t port, float *out_c)
{
    if (out_c == NULL) return ESP_ERR_INVALID_ARG;

    /* Trigger a temperature conversion. */
    esp_err_t err = max30102_write_reg(port, REG_TEMP_CONFIG, 0x01);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t intr = 0, frac = 0;
    err = max30102_read_reg(port, REG_TEMP_INTR, &intr, 1);
    if (err != ESP_OK) return err;
    err = max30102_read_reg(port, REG_TEMP_FRAC, &frac, 1);
    if (err != ESP_OK) return err;

    /* Intr is the integer part (signed two's complement in 8 bits);
     * frac is the fractional part in 1/16ths (low 4 bits). */
    int8_t signed_intr = (int8_t)intr;
    *out_c = (float)signed_intr + (float)(frac & 0x0F) / 16.0f;
    return ESP_OK;
}

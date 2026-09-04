/**
 * max30102.c - Driver implementation. See max30102.h for design notes.
 */
#include "max30102.h"
#include <stdlib.h>
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
     * Defaults: SPO2 mode, 200 sps, 215 us LED pulse, 8-sample averaging
     *           (effective 25 sps = 40 ms/sample - MUST match
     *           MAX30102_ALG_SAMPLE_MS in max30102_algorithm.h), FIFO
     *           rollover on, ~25.4 mA on both LEDs. */
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
    /* REG_FIFO_CONFIG (0x08): SMP_AVE=011 (avg 8), ROLLOVER=1, A_FULL=0.
     * 8x averaging gives an effective 25 sps (200/8), which matches the
     * algorithm's 40 ms sample-period assumption, plus better per-sample
     * SNR. (The old 4x = 50 sps ran the time base at 2x speed, so the
     * HR estimator reported HALF the true bpm - a resting 60 bpm came
     * out as exactly 30 and was zeroed by the hr > 30 gate.) */
    cfg.data6 = (0b011 << 5) | (1 << 4) | 0;
    /* REG_MODE_CONFIG (0x09): SHDN=0, RESET=0, MODE=011 (SPO2). */
    cfg.data7 = 0b011;
    /* REG_SPO2_CONFIG (0x0A): ADC_RGE=01, SR=001 (200 Hz), LED_PW=10. */
    cfg.data8 = (0b11 << 5) | (0b010 << 2) | 0b10;
    /* LED currents. 0x7F = 25.4 mA (0.2 mA per step). The previous 0x24
     * (7.2 mA) contradicted the documented ~25.4 mA default and left
     * the RED/IR pair underdriven: weak AC on many fingers, failing the
     * correlation gate and producing 0 % SpO2. If the envelope
     * diagnostic pins at ~262143 (saturated), drop to 0x3F (12.6 mA)
     * or raise SPO2_ADC_RGE. */
    cfg.data9  = 0x7F;
    cfg.data10 = 0x7F;
    cfg.data11 = 0x7F;
    /* Multi-LED control: all slots disabled in SPO2 mode. */
    cfg.data12 = 0x00;
    cfg.data13 = 0x00;

    return cfg;
}

esp_err_t max30102_write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return i2c_master_transmit(dev, buf, sizeof(buf), pdMS_TO_TICKS(1000));
}

esp_err_t max30102_read_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, buf, len,
                                       pdMS_TO_TICKS(1000));
}

esp_err_t max30102_init(i2c_master_dev_handle_t dev, const max30102_config_t *cfg)
{
    max30102_config_t local_cfg;
    if (cfg == NULL) {
        local_cfg = max30102_default_config();
        cfg = &local_cfg;
    }

    esp_err_t err;

    /* Reset the device first so we start from a known state. */
    err = max30102_write_reg(dev, REG_MODE_CONFIG, 0b01000000); /* RESET=1 */
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Clear interrupt status registers (read-to-clear). */
    uint8_t tmp[2] = {0};
    max30102_read_reg(dev, REG_INTR_STATUS_1, tmp, 2);

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
        err = max30102_write_reg(dev, regs[i][0], regs[i][1]);
        if (err != ESP_OK) return err;
    }

    return ESP_OK;
}

esp_err_t max30102_read_fifo(i2c_master_dev_handle_t dev, int32_t *red, int32_t *ir)
{
    if (red == NULL || ir == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t raw[6] = {0};
    uint8_t reg    = REG_FIFO_DATA;
    esp_err_t err = i2c_master_transmit_receive(dev, &reg, 1, raw, sizeof(raw),
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

esp_err_t max30102_read_fifo_burst(i2c_master_dev_handle_t dev,
                                   int32_t *red_out, int32_t *ir_out,
                                   size_t max_samples, size_t *out_count)
{
    if (red_out == NULL || ir_out == NULL || out_count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (max_samples == 0) {
        *out_count = 0;
        return ESP_OK;
    }

    /* Determine how many samples are actually in the FIFO. */
    uint8_t wr_ptr = 0, rd_ptr = 0;
    esp_err_t err;
    err = max30102_read_reg(dev, REG_FIFO_WR_PTR, &wr_ptr, 1);
    if (err != ESP_OK) return err;
    err = max30102_read_reg(dev, REG_FIFO_RD_PTR, &rd_ptr, 1);
    if (err != ESP_OK) return err;

    wr_ptr &= 0x1F;
    rd_ptr &= 0x1F;
    size_t avail = (wr_ptr >= rd_ptr)
                   ? (size_t)(wr_ptr - rd_ptr)
                   : (size_t)(32 - rd_ptr + wr_ptr);
    size_t n = (avail < max_samples) ? avail : max_samples;
    if (n == 0) {
        *out_count = 0;
        return ESP_OK;
    }

    /* Single I2C transaction: write REG_FIFO_DATA, then read n*6 bytes.
     * The MAX30102 auto-increments the FIFO read pointer on each byte
     * read, so this drains exactly n samples in one burst. */
    size_t byte_count = n * 6;
    uint8_t *buf = (uint8_t *)malloc(byte_count);
    if (buf == NULL) return ESP_ERR_NO_MEM;

    uint8_t reg = REG_FIFO_DATA;
    err = i2c_master_transmit_receive(dev, &reg, 1, buf, byte_count,
                                      pdMS_TO_TICKS(1000));
    if (err != ESP_OK) {
        free(buf);
        return err;
    }

    for (size_t i = 0; i < n; ++i) {
        const uint8_t *s = buf + i * 6;
        red_out[i] = ((int32_t)(s[0] & 0x03) << 16) |
                     ((int32_t)s[1] << 8) |
                      (int32_t)s[2];
        ir_out[i]  = ((int32_t)(s[3] & 0x03) << 16) |
                     ((int32_t)s[4] << 8) |
                      (int32_t)s[5];
    }
    free(buf);
    *out_count = n;
    return ESP_OK;
}

esp_err_t max30102_enable_fifo_a_full_int(i2c_master_dev_handle_t dev, bool enable)
{
    /* REG_INTR_ENABLE_1 bit 7 = A_FULL_EN. Read-modify-write to preserve
     * the other interrupt enables (PPG_RDY, PROX, etc.) - though we
     * currently only use A_FULL. */
    uint8_t cur = 0;
    esp_err_t err = max30102_read_reg(dev, REG_INTR_ENABLE_1, &cur, 1);
    if (err != ESP_OK) return err;
    if (enable) cur |=  (1u << 7);
    else        cur &= ~(1u << 7);
    return max30102_write_reg(dev, REG_INTR_ENABLE_1, cur);
}

esp_err_t max30102_read_int_status(i2c_master_dev_handle_t dev,
                                   uint8_t *status1, uint8_t *status2)
{
    uint8_t buf[2] = {0};
    esp_err_t err = max30102_read_reg(dev, REG_INTR_STATUS_1, buf, 2);
    if (err != ESP_OK) return err;
    if (status1) *status1 = buf[0];
    if (status2) *status2 = buf[1];
    return ESP_OK;
}

esp_err_t max30102_read_temp(i2c_master_dev_handle_t dev, float *out_c)
{
    if (out_c == NULL) return ESP_ERR_INVALID_ARG;

    /* Trigger a temperature conversion, then wait for DIE_TEMP_RDY
     * (INTR_STATUS_2 bit 1). The fixed 10 ms delay previously used
     * here raced the conversion (~tens of ms) and read TINT/TFRAC
     * before they were updated - bring-up logs showed a suspiciously
     * exact "0.0 C". Polling the ready bit (clear-on-read, same bit
     * layout as INTR_ENABLE_2) is correct without guessing the
     * conversion time. Bounded at 20 x 2 ms so a wedged converter
     * cannot hang the boot sequence. */
    esp_err_t err = max30102_write_reg(dev, REG_TEMP_CONFIG, 0x01);
    if (err != ESP_OK) return err;
    for (int i = 0; i < 20; ++i) {
        uint8_t st2 = 0;
        if (max30102_read_reg(dev, REG_INTR_STATUS_2, &st2, 1) == ESP_OK &&
            (st2 & (1u << 1))) {
            break;              /* DIE_TEMP_RDY */
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    uint8_t intr = 0, frac = 0;
    err = max30102_read_reg(dev, REG_TEMP_INTR, &intr, 1);
    if (err != ESP_OK) return err;
    err = max30102_read_reg(dev, REG_TEMP_FRAC, &frac, 1);
    if (err != ESP_OK) return err;

    /* Intr is the integer part (signed two's complement in 8 bits);
     * frac is the fractional part in 1/16ths (low 4 bits). */
    int8_t signed_intr = (int8_t)intr;
    *out_c = (float)signed_intr + (float)(frac & 0x0F) / 16.0f;
    return ESP_OK;
}

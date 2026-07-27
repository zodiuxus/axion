#pragma once
/**
 * config.h - Static, non-secret hardware configuration and tuning constants.
 *
 * Anything device-specific or sensitive (SIM PIN, alert phone numbers) goes
 * in `secrets.h` instead, which is intentionally gitignored.
 */
#ifndef AXION_CONFIG_H
#define AXION_CONFIG_H

#include <stdint.h>
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
/* Bumped from 100 kHz to 400 kHz: the MPU data-ready ISR fires at ~200 Hz
 * and each wake does an accel read (7 B) + optional DMP FIFO read (~47 B).
 * At 100 kHz that would saturate the bus; 400 kHz gives comfortable headroom
 * for the MAX30102 burst reads as well. Both chips are 400 kHz-rated. */
#define I2C_MASTER_FREQ_HZ      400000U
#define I2C_MASTER_PORT         I2C_NUM_0

/* ---- UART1 (A7670E modem) -------------------------------------------- */
#define PIN_UART_TX             GPIO_NUM_17
#define PIN_UART_RX             GPIO_NUM_18
#define PIN_UART_RTS            GPIO_NUM_15
#define PIN_UART_CTS            GPIO_NUM_16
#define UART_MODEM_PORT         UART_NUM_1
#define UART_MODEM_BAUD         115200U
#define UART_BUF_SIZE           512U
#define UART_READ_TIMEOUT_MS    2000U

/* ---- 1-Wire (DS18B20 body temperature) ------------------------------- */
#define PIN_DS18B20             GPIO_NUM_4

/* ---- Haptic / buzzer feedback ---------------------------------------- */
#define PIN_BUZZER              GPIO_NUM_10

/* ---- Green status LED ------------------------------------------------ */
/* Moved from GPIO8 to GPIO42 (one of the JTAG pins MTMS/MTDI/MTDO/MTCK =
 * GPIO39-42). Since we don't use JTAG for debugging, those pins are free
 * for general GPIO. GPIO8 is now used by the MAX30102 INT pin (see below). */
#define PIN_GREEN_LED           GPIO_NUM_42

/* ---- MAX30102 INT ---------------------------------------------------- */
/* MAX30102's INT pin (open-drain, active-low). Fires on FIFO_A_FULL
 * (17 samples queued), PPG_RDY (every sample), or PROX_INT (finger on/off).
 * We use FIFO_A_FULL so the task can burst-read ~17 samples in one I2C
 * transaction instead of polling 1-at-a-time. */
#define PIN_MAX30102_INT        GPIO_NUM_8

/* ---- Alert-abort button ---------------------------------------------- */
/* Active-low (internal pull-up): pressed = GPIO reads 0. */
#define PIN_ALERT_BUTTON        GPIO_NUM_9
#define BUTTON_DEBOUNCE_MS      50U

/* ---- Calibration ----------------------------------------------------- */
/* Duration of the first-run calibration window for each sensor (ms).
 * Only runs when the CAL_FLAG_FIRST_RUN bit is set in NVS. */
#define TEMP_CALIBRATION_MS     120000U     /* 2 minutes */
#define OXIM_CALIBRATION_MS     120000U     /* 2 minutes */
/* Safety timeout for the status LED: if calibration hasn't completed
 * by this deadline (e.g., a sensor failed to init), the LED proceeds
 * to the "complete" glow anyway so the user isn't left staring at a
 * blinking LED forever. */
#define LED_CALIBRATION_TIMEOUT_MS  240000U /* 4 minutes */

/* ---- Status LED blink timing ---------------------------------------- */
/* Boot / starting-up phase: 25 ms on, 75 ms off (100 ms period). */
#define LED_BOOT_ON_MS          25U
#define LED_BOOT_OFF_MS         75U
/* Calibration phase: 50 ms on, 150 ms off (200 ms period). */
#define LED_CALIB_ON_MS         50U
#define LED_CALIB_OFF_MS        150U
/* Completion glow: solid on, then off permanently. */
#define LED_COMPLETE_MS         2000U

/* ---- Collision detection (raw accelerometer) -------------------------- */
/* High-priority impact detector: reads raw accel X/Y/Z from the MPU6050
 * on every data-ready interrupt (~200 Hz) and computes the magnitude
 * vector. If |a| >= COLLISION_THRESHOLD_G, BIT_COLLISION_DETECTED is set
 * in the event group, which the monitor task consumes to bypass the
 * WARNING phase and escalate directly to ALERT (immediate SMS path).
 *
 * The collision check is performed inside mpu_int_task - the same task
 * that drains the DMP FIFO. There's no separate collision_task: 
 * the MPU has only one INT pin, so the data-ready interrupt drives both.
 * A cooldown (COLLISION_COOLDOWN_MS) prevents a single physical impact
 * from firing multiple events.
 *
 * The MPU6050 accel range is set to ±4g in mpu_setup() so that readings
 * above 2G don't saturate (the default ±2g range would clip at exactly
 * the trigger threshold on a single axis). ±4g gives 8192 LSB/g and
 * 2G of headroom above the trigger. */
#define COLLISION_THRESHOLD_G   2.0f        /* total-magnitude trigger (g) */
#define COLLISION_COOLDOWN_MS   3000U       /* min spacing between triggers */

/* ---- Tuning constants ------------------------------------------------- */
/* Fall detection: native ±180° roll range; considered fallen if |roll| >= this. */
#define FALL_ANGLE_THRESHOLD    150.0f

/* Vitals warning / alert timing (ms). */
#define WARNING_MS              5000U
#define ALERT_MS                5000U
#define BUZZER_PERIOD_MS        500U

/* Body temperature thresholds (°C) - baseline-relative deltas.
 * The baseline is the user's calibrated normal body temp (stored in NVS
 * after the first-run 2-minute calibration, loaded into shared state on
 * every boot). Hypo/hyper are checked as deltas from that baseline, so
 * they adapt to each user instead of assuming everyone is exactly
 * 36.1–37.2 °C.
 *   hypothermia:  temp < (baseline - TEMP_HYPO_DELTA)
 *   hyperthermia: temp > (baseline + TEMP_HYPER_DELTA)
 * TEMP_BASELINE_DEFAULT is used until calibration completes (first boot
 * or factory reset), so the monitor always has a usable reference. */
#define TEMP_HYPO_DELTA        0.7f    /* baseline - 0.7 = hypothermia trigger */
#define TEMP_HYPER_DELTA       1.0f    /* baseline + 1.0 = hyperthermia trigger */
#define TEMP_BASELINE_DEFAULT  36.5f   /* fallback if no calibrated baseline yet */

/* Speed thresholds (m/s). */
#define SPEED_STOPPED           0.5f

/* GNSS polling interval (ms). */
#define GNSS_POLL_MS            5000U

/* MAX30102 oximeter (sample period per FIFO read). */
#define MAX30102_SAMPLE_MS      40U
#define MAX30102_BUFFER_SIZE    128

/* How many alert recipients are configured (driven by secrets.h). */
#define AXION_ALERT_PHONE_MAX_COUNT 2

/* Cooldown after an alert abort (button press). Prevents the monitor
 * from immediately re-entering WARNING if the alert condition persists. */
#define ABORT_COOLDOWN_MS       10000U    /* 10 seconds */

#ifdef __cplusplus
}
#endif

#endif /* AXION_CONFIG_H */

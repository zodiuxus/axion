#pragma once
/**
 * config.h - Static, non-secret hardware configuration and tuning constants.
 * Device-specific or sensitive values (SIM PIN, phone numbers) go in
 * gitignored secrets.h.
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
/* 400 kHz: the 200 Hz MPU INT path + MAX30102 bursts saturate 100 kHz. */
#define I2C_MASTER_FREQ_HZ      400000U
#define I2C_MASTER_PORT         I2C_NUM_0

/* ---- UART1 (A7670E modem) --------------------------------------------- */
#define PIN_UART_TX             GPIO_NUM_17
#define PIN_UART_RX             GPIO_NUM_18
#define PIN_UART_RTS            GPIO_NUM_15
#define PIN_UART_CTS            GPIO_NUM_16
#define UART_MODEM_PORT         UART_NUM_1
#define UART_MODEM_BAUD         115200U
#define UART_BUF_SIZE           512U
#define UART_READ_TIMEOUT_MS    2000U

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
#define BUTTON_DEBOUNCE_MS      50U

/* ---- Calibration -------------------------------------------------------- */
/* First-run calibration window per sensor (ms); only when
 * CAL_FLAG_FIRST_RUN is set in NVS. */
#define TEMP_CALIBRATION_MS     120000U     /* 2 minutes */
#define OXIM_CALIBRATION_MS     120000U     /* 2 minutes */

/* ---- Thermal settling (every boot) -------------------------------------- */
/* Wait for the DS18B20 reading to go flat (trailing p2p < band, capped)
 * before arming temp alerting / calibrating - the probe ramps from
 * ambient to skin temp for minutes after wear starts.
 * Budget: ~0.8 s first reading + 90 s settle cap + 120 s cal window
 * must stay under LED_CALIBRATION_TIMEOUT_MS (240 s), or the LED clears
 * FIRST_RUN before the baseline is stored. */
#define TEMP_SETTLE_WINDOW_MS   15000U      /* trailing flatness window */
#define TEMP_SETTLE_BAND_C      0.2f        /* max p2p drift = "flat" */
#define TEMP_SETTLE_MAX_MS      90000U      /* cap; proceed anyway */
/* Fail-open: a probe that never reads releases BIT_TEMP_CALIBRATED
 * (the monitor's arming gate) after this long; temp alerting stays
 * disabled. Disjoint from the budget above (no reading = no settle/cal). */
#define TEMP_FIRST_READ_TIMEOUT_MS  30000U
/* Stored baselines below this are implausible for skin (bench run). */
#define TEMP_BASELINE_PLAUSIBLE_MIN_C  30.0f

/* Status-LED safety timeout: reach COMPLETE even if a sensor never
 * finishes calibrating. */
#define LED_CALIBRATION_TIMEOUT_MS  240000U

/* ---- Status LED blink timing -------------------------------------------- */
#define LED_BOOT_ON_MS          25U
#define LED_BOOT_OFF_MS         75U
#define LED_CALIB_ON_MS         50U
#define LED_CALIB_OFF_MS        150U
#define LED_COMPLETE_MS         2000U

/* ---- Collision detection (raw accelerometer) ----------------------------- */
/* Total-magnitude trigger (includes gravity: ~1 g at rest). Escalation is
 * gated on system-armed (state.h); the cooldown limits re-triggers. */
#define COLLISION_THRESHOLD_G   2.0f
#define COLLISION_COOLDOWN_MS   3000U

/* ---- Tuning constants ----------------------------------------------------- */
/* Fallen if |roll| >= this (native ±180° range). */
#define FALL_ANGLE_THRESHOLD    150.0f

/* Vitals warning / alert timing (ms). */
#define WARNING_MS              5000U
#define ALERT_MS                5000U
#define BUZZER_PERIOD_MS        500U

/* Temp thresholds are deltas from the calibrated per-user baseline.
 * TEMP_BASELINE_DEFAULT is a placeholder for shared-state init only -
 * never an alerting reference (the monitor gates on
 * temp_baseline_valid). */
#define TEMP_FIRST_READ_TIMEOUT_MS  30000U
#define TEMP_BASELINE_DEFAULT   36.2f
#define TEMP_BASELINE_PLAUSIBLE_MIN_C  30.0f
#define TEMP_HYPO_DELTA         0.7f
#define TEMP_HYPER_DELTA        1.0f

/* HR/SpO2 thresholds - wide band, this is an uncalibrated estimator.
 * 40 bpm floor sits above the estimator's own 30 bpm validity floor;
 * practically detectable abnormal SpO2 band is [70, 90) %. */
#define HR_LOW_THRESHOLD        40      /* bpm */
#define HR_HIGH_THRESHOLD       120     /* bpm */
#define SPO2_LOW_THRESHOLD      90.0f   /* % */
/* Consecutive abnormal VALID windows (1 window = 5.12 s): 3 = ~15 s
 * sustained; first SMS ~25 s after onset. */
#define VITALS_SUSTAIN_WINDOWS  3
/* Suppress re-page while the condition sustains; collisions bypass. */
#define SMS_REPAGE_COOLDOWN_MS  300000U

/* Speed thresholds (m/s). */
#define SPEED_STOPPED           0.5f

/* GNSS polling interval (ms). */
#define GNSS_POLL_MS            5000U

/* MAX30102 oximeter (must match MAX30102_ALG_SAMPLE_MS). */
#define MAX30102_SAMPLE_MS      40U
#define MAX30102_BUFFER_SIZE    128

/* Alert recipients configured (driven by secrets.h). */
#define AXION_ALERT_PHONE_MAX_COUNT 2

/* Cooldown after a button abort before re-entering WARNING. */
#define ABORT_COOLDOWN_MS       10000U

#ifdef __cplusplus
}
#endif

#endif /* AXION_CONFIG_H */

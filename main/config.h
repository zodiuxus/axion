#pragma once
/**
 * config.h - Aggregate header for static, non-secret configuration.
 *
 * Wraps config_pins.h (wiring, pulls in ESP-IDF headers) and config_tuning.h
 * (pure thresholds/timings). Host-testable code includes config_tuning.h.
 */
#ifndef AXION_CONFIG_H
#define AXION_CONFIG_H

#include "config_pins.h"
#include "config_tuning.h"

#endif /* AXION_CONFIG_H */

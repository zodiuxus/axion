#pragma once
/**
 * temperature.h — DS18B20 body-temperature task.
 */
#ifndef AXION_TEMPERATURE_H
#define AXION_TEMPERATURE_H

#ifdef __cplusplus
extern "C" {
#endif

/** FreeRTOS task: discovers DS18B20 devices on the 1-Wire bus, then
 *  periodically samples temperatures and pushes them into the shared
 *  state. Sets BIT_TEMP_READY after the first successful conversion. */
void temperature_task(void *arg);

#ifdef __cplusplus
}
#endif

#endif /* AXION_TEMPERATURE_H */

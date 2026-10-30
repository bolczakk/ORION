/*
 * distance_sensors.h
 *
 *  Created on: 27 paź 2026
 *      Author: Oleg
 */

#ifndef INC_DISTANCE_SENSORS_H_
#define INC_DISTANCE_SENSORS_H_

#include <stdint.h>
#include "stm32h7xx_hal.h"
#include "vl53lx_api.h"

#define DISTANCE_SENSORS_READY_FLAG  (1UL << 0)
#define DISTANCE_SENSORS_POLL_MS     20U

/* Debugger diagnostics, not a synchronization interface between threads. */
extern volatile uint8_t vl53_initialized;
extern volatile uint8_t vl53_init_step;
extern volatile VL53LX_Error vl53_init_status;
extern volatile VL53LX_Error vl53_read_status;
extern volatile HAL_StatusTypeDef vl61_io_status;
extern volatile uint8_t vl61_range_status;

/* Call only from distanceSensor; owns SHUT pins and both sensor instances.
 * Platform requirement: VL53LX I2C callbacks must use the same hi2c1 bus.
 */
void DistanceSensors_NotifyReadyFromISR(void);
void DistanceSensors_Init(void);
void DistanceSensors_Process(void);

#endif /* INC_DISTANCE_SENSORS_H_ */

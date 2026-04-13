/*
 * sensors.h
 *
 * Created on: Jan 23, 2026
 * Author: Oleg
 */

/**
 ******************************************************************************
 * @file           : sensors.h
 * @brief          : Header file for DS18B20 Temperature Sensor Driver.
 * Provides high-level interface for sensor initialization,
 * data acquisition, and value retrieval.
 * @author         : Oleg Swerblewski, Dawid Sobieska
 * @date           : 2026-01-23
 ******************************************************************************
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#include "ds18b20.h"
#include "main.h"
#include "cmsis_os.h"

/**
 * @brief Global handle for the DS18B20 sensor instance.
 * Exposed to allow low-level access (e.g., in timer callbacks).
 */
extern ds18b20_t ds18;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  Initializes the Temperature Sensor Module.
 * Configures the underlying 1-Wire protocol using a hardware Timer.
 * @param  htim Pointer to the TIM handle used for microsecond delays.
 * @retval None
 */

void Sensors_Init(TIM_HandleTypeDef *htim);
/**
 * @brief  Performs a complete measurement cycle.
 * Sequence: Convert -> Wait (Busy) -> Read Scratchpad.
 * @note   This function contains blocking loops while waiting for conversion.
 * @retval None
 */
void Sensors_Process(void);

/**
 * @brief  Getter for the last measured temperature.
 * @retval Temperature in degrees Celsius (float).
 */
float Sensors_GetTemperature(void);

#endif /* INC_SENSORS_H_ */

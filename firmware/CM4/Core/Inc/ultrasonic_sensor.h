/*
 * ultrasonic_sensor.h
 *
 *  Created on: 29 paź 2026
 *      Author: Oleg
 */

#ifndef INC_ULTRASONIC_SENSOR_H_
#define INC_ULTRASONIC_SENSOR_H_

#include "urm37.h"

/* Debugger output; -1 means no current valid distance. */
extern volatile float urm37_distance_cm;
extern volatile URM37_Result urm37_status;
extern volatile HAL_StatusTypeDef urm37_hal_status;
extern volatile uint32_t urm37_uart_error;

void StartUltrasonicSensor(void *argument);
void UltrasonicSensor_RxCompleteFromISR(UART_HandleTypeDef *uart);
void UltrasonicSensor_ErrorFromISR(UART_HandleTypeDef *uart);

#endif /* INC_ULTRASONIC_SENSOR_H_ */

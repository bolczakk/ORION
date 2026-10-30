/*
 * urm37.h
 *
 *  Created on: 29 paź 2026
 *      Author: Oleg
 */

#ifndef INC_URM37_H_
#define INC_URM37_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32h7xx_hal.h"

typedef enum {
	URM37_OK = 0,
	URM37_PENDING,
	URM37_NO_TARGET,
	URM37_BAD_FRAME,
	URM37_UART_ERROR,
	URM37_TIMEOUT,
	URM37_NOT_STARTED
} URM37_Result;

/* Single task owner, dedicated UART. TX and RX buffers outlive the IT transfers. */
typedef struct {
	UART_HandleTypeDef *uart;
	void (*notify_from_isr)(void);
	uint8_t tx[4];
	uint8_t rx[4];
	volatile bool active;
	volatile bool rx_complete;
	volatile bool uart_failed;
	volatile uint32_t uart_error;
	HAL_StatusTypeDef last_hal_status;
} URM37_Device;

void URM37_Init(URM37_Device *dev, UART_HandleTypeDef *uart,
		void (*notify_from_isr)(void));
HAL_StatusTypeDef URM37_StartMeasurement(URM37_Device *dev);
/* PENDING leaves the request active; all other results finish it.
 * distance_cm is written only for URM37_OK. Timeout belongs to the caller.
 */
URM37_Result URM37_GetResult(URM37_Device *dev, uint16_t *distance_cm);
HAL_StatusTypeDef URM37_Cancel(URM37_Device *dev);
void URM37_OnRxCompleteFromISR(URM37_Device *dev, UART_HandleTypeDef *uart);
void URM37_OnErrorFromISR(URM37_Device *dev, UART_HandleTypeDef *uart);

#endif /* INC_URM37_H_ */

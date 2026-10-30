/*
 * urm37.c
 *
 *  Created on: 29 paź 2026
 *      Author: Oleg
 */

/*
 * urm37.c
 *
 *  Created on: 29 paź 2026
 *      Author: Oleg
 */

#include "urm37.h"
#include <stddef.h>

#define DISTANCE_COMMAND 0x22U

void URM37_Init(URM37_Device *dev, UART_HandleTypeDef *uart,
		void (*notify_from_isr)(void)) {
	if (dev != NULL) {
		/* First initialization only; cancel an active transfer before reinit. */
		*dev = (URM37_Device ) { .uart = uart, .notify_from_isr =
						notify_from_isr, .tx = { DISTANCE_COMMAND, 0U, 0U,
				DISTANCE_COMMAND }, .last_hal_status = HAL_OK };
	}
}

HAL_StatusTypeDef URM37_Cancel(URM37_Device *dev) {
	if (dev == NULL || dev->uart == NULL) {
		return HAL_ERROR;
	}
	if (!dev->active) {
		return HAL_OK;
	}
	/* Ignore any callback caused by cancellation. Only this driver owns UART. */
	dev->active = false;
	HAL_StatusTypeDef rx = HAL_UART_AbortReceive(dev->uart);
	HAL_StatusTypeDef tx = HAL_UART_AbortTransmit(dev->uart);
	dev->last_hal_status = rx != HAL_OK ? rx : tx;
	return dev->last_hal_status;
}

HAL_StatusTypeDef URM37_StartMeasurement(URM37_Device *dev) {
	if (dev == NULL || dev->uart == NULL) {
		return HAL_ERROR;
	}
	if (dev->active || HAL_UART_GetState(dev->uart) != HAL_UART_STATE_READY) {
		dev->last_hal_status = HAL_BUSY;
		return HAL_BUSY;
	}
	/* Discard a leftover byte from a previous timed-out transaction. H7 HAL. */
	__HAL_UART_SEND_REQ(dev->uart, UART_RXDATA_FLUSH_REQUEST);
	dev->rx_complete = false;
	dev->uart_failed = false;
	dev->uart_error = 0U;
	dev->active = true;

	/* Arm reception FIRST, so an early response cannot be lost. */
	HAL_StatusTypeDef result = HAL_UART_Receive_IT(dev->uart, dev->rx, 4U);
	if (result != HAL_OK) {
		dev->active = false;
		dev->last_hal_status = result;
		return result;
	}
	result = HAL_UART_Transmit_IT(dev->uart, dev->tx, 4U);
	if (result != HAL_OK) {
		dev->active = false;
		HAL_StatusTypeDef cleanup = HAL_UART_AbortReceive(dev->uart);
		dev->last_hal_status = cleanup != HAL_OK ? cleanup : result;
		return dev->last_hal_status;
	}
	dev->last_hal_status = HAL_OK;
	return HAL_OK;
}

URM37_Result URM37_GetResult(URM37_Device *dev, uint16_t *distance_cm) {
	if (dev == NULL || dev->uart == NULL || distance_cm == NULL
			|| !dev->active) {
		return URM37_NOT_STARTED;
	}
	if (!dev->uart_failed && !dev->rx_complete) {
		return URM37_PENDING;
	}
	/* Stop transfers before copying the buffer; also handles a HAL receive error. */
	HAL_StatusTypeDef cleanup = URM37_Cancel(dev);
	bool failed = dev->uart_failed;
	if (failed || cleanup != HAL_OK) {
		return URM37_UART_ERROR;
	}
	uint8_t sum = (uint8_t) ((uint16_t) dev->rx[0] + dev->rx[1] + dev->rx[2]);
	if (dev->rx[0] != DISTANCE_COMMAND || dev->rx[3] != sum) {
		return URM37_BAD_FRAME;
	}
	uint16_t value = (uint16_t) (((uint16_t) dev->rx[1] << 8) | dev->rx[2]);
	if (value == UINT16_MAX) {
		return URM37_NO_TARGET;
	}
	*distance_cm = value;
	return URM37_OK;
}

void URM37_OnRxCompleteFromISR(URM37_Device *dev, UART_HandleTypeDef *uart) {
	if (dev != NULL && dev->uart != NULL && dev->uart == uart && dev->active) {
		dev->rx_complete = true;
		if (dev->notify_from_isr != NULL) {
			dev->notify_from_isr();
		}
	}
}

void URM37_OnErrorFromISR(URM37_Device *dev, UART_HandleTypeDef *uart) {
	if (dev != NULL && dev->uart != NULL && dev->uart == uart && dev->active) {
		dev->uart_error = HAL_UART_GetError(uart);
		dev->uart_failed = true;
		if (dev->notify_from_isr != NULL) {
			dev->notify_from_isr();
		}
	}
}


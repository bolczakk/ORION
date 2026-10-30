/*
 * ultrasonic_sensor.c
 *
 *  Created on: 29 paź 2026
 *      Author: Oleg
 */

#include "ultrasonic_sensor.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "usart.h"
#include <stddef.h>
#include "shared_data.h"
#include "shared_ipc.h"

#define UART_EVENT_FLAG   (1UL << 0)
#define RESPONSE_MS       200U
#define PERIOD_MS         250U
#define BOOT_MS           500U

static URM37_Device sensor;
static osThreadId_t owner;

volatile float urm37_distance_cm = -1.0f;
volatile URM37_Result urm37_status = URM37_NOT_STARTED;
volatile HAL_StatusTypeDef urm37_hal_status = HAL_OK;
volatile uint32_t urm37_uart_error = 0U;

static void NotifyFromISR(void) {
	if (owner != NULL) {
		(void) osThreadFlagsSet(owner, UART_EVENT_FLAG);
	}
}

void UltrasonicSensor_RxCompleteFromISR(UART_HandleTypeDef *uart) {
	URM37_OnRxCompleteFromISR(&sensor, uart);
}

void UltrasonicSensor_ErrorFromISR(UART_HandleTypeDef *uart) {
	URM37_OnErrorFromISR(&sensor, uart);
}

void StartUltrasonicSensor(void *argument) {
	(void) argument;
	SHARED_DATA->m4_distances[2] = -1.0f;
	owner = osThreadGetId();
	URM37_Init(&sensor, &huart4, NotifyFromISR);
	(void) osDelay((uint32_t) pdMS_TO_TICKS(BOOT_MS) + 2U);
	uint32_t response_ticks = (uint32_t) pdMS_TO_TICKS(RESPONSE_MS);
	uint32_t period_ticks = (uint32_t) pdMS_TO_TICKS(PERIOD_MS);
	if (response_ticks == 0U) {
		response_ticks = 1U;
	}
	if (period_ticks == 0U) {
		period_ticks = 1U;
	}

	for (;;) {
		uint32_t started = osKernelGetTickCount();
		(void) osThreadFlagsClear(UART_EVENT_FLAG);
		urm37_hal_status = URM37_StartMeasurement(&sensor);
		URM37_Result result = URM37_UART_ERROR;
		uint16_t distance_cm = 0U;

		if (urm37_hal_status == HAL_OK) {
			uint32_t flags = osThreadFlagsWait(UART_EVENT_FLAG, osFlagsWaitAny,
					response_ticks);
			/* Inspect the driver even after timeout: RX may complete at the boundary. */
			result = URM37_GetResult(&sensor, &distance_cm);
			if (result == URM37_PENDING) {
				HAL_StatusTypeDef cleanup = URM37_Cancel(&sensor);
				result =
						cleanup != HAL_OK ?
								URM37_UART_ERROR :
								(flags == osFlagsErrorTimeout ?
										URM37_TIMEOUT : URM37_UART_ERROR);
			}
			urm37_hal_status = sensor.last_hal_status;
		}
		urm37_uart_error = sensor.uart_error;
		urm37_status = result;
		urm37_distance_cm = result == URM37_OK ? (float) distance_cm : -1.0f;
		SHARED_DATA->m4_distances[2] = urm37_distance_cm;

		uint32_t elapsed = osKernelGetTickCount() - started;
		(void) osDelay(elapsed < period_ticks ? period_ticks - elapsed : 1U);
	}
}

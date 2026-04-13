/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"       // Dla funkcji wyświetlacza ssd1306_...
#include "ssd1306_fonts.h" // Dla czcionek takich jak Font_7x10
#include "sensors.h"       // Dla Sensors_GetTemperature()
#include <stdio.h>
#include <string.h>
#include "tim.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for blink01 */
osThreadId_t blink01Handle;
const osThreadAttr_t blink01_attributes = { .name = "blink01", .stack_size = 256
		* 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for blink02 */
osThreadId_t blink02Handle;
const osThreadAttr_t blink02_attributes = { .name = "blink02", .stack_size = 256
		* 4, .priority = (osPriority_t) osPriorityLow, };
/* Definitions for oled */
osThreadId_t oledHandle;
const osThreadAttr_t oled_attributes = { .name = "oled", .stack_size = 256 * 4,
		.priority = (osPriority_t) osPriorityLow7, };
/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = { .name = "SensorTask",
		.stack_size = 256 * 4, .priority = (osPriority_t) osPriorityHigh1, };
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = { .name = "MotorTask", .stack_size =
		256 * 4, .priority = (osPriority_t) osPriorityHigh, };
/* Definitions for btnS */
osSemaphoreId_t btnSHandle;
const osSemaphoreAttr_t btnS_attributes = { .name = "btnS" };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == USER_Btn_Pin) {
		osSemaphoreRelease(btnSHandle);
	}
}

/* USER CODE END FunctionPrototypes */

void StartBlink01(void *argument);
void StartBlink02(void *argument);
void StartOled(void *argument);
void StartSensorTask(void *argument);
void StartMotorTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* Create the semaphores(s) */
	/* creation of btnS */
	btnSHandle = osSemaphoreNew(1, 1, &btnS_attributes);

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of blink01 */
	blink01Handle = osThreadNew(StartBlink01, NULL, &blink01_attributes);

	/* creation of blink02 */
	blink02Handle = osThreadNew(StartBlink02, NULL, &blink02_attributes);

	/* creation of oled */
	oledHandle = osThreadNew(StartOled, NULL, &oled_attributes);

	/* creation of SensorTask */
	SensorTaskHandle = osThreadNew(StartSensorTask, NULL,
			&SensorTask_attributes);

	/* creation of MotorTask */
	MotorTaskHandle = osThreadNew(StartMotorTask, NULL, &MotorTask_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartBlink01 */
/**
 * @brief  Function implementing the blink01 thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartBlink01 */
void StartBlink01(void *argument) {
	/* USER CODE BEGIN StartBlink01 */
	/* Infinite loop */
	for (;;) {
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		osDelay(500);

	}
	osThreadTerminate(NULL);
	/* USER CODE END StartBlink01 */
}

/* USER CODE BEGIN Header_StartBlink02 */
/**
 * @brief Function implementing the blink02 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartBlink02 */
void StartBlink02(void *argument) {
	/* USER CODE BEGIN StartBlink02 */
	/* Infinite loop */
	for (;;) {
		if (osSemaphoreAcquire(btnSHandle, osWaitForever) == osOK) {
			HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
			osDelay(300);
		}
	}
	osThreadTerminate(NULL);
	/* USER CODE END StartBlink02 */
}

/* USER CODE BEGIN Header_StartOled */
/**
 * @brief Function implementing the oled thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartOled */
void StartOled(void *argument) {
	/* USER CODE BEGIN StartOled */
	char bufor[32];
	/* Infinite loop */
	for (;;) {
		ssd1306_Fill(Black);
		ssd1306_SetCursor(115, 0);
		ssd1306_WriteChar('M', Font_7x10, White);

		// Pobieramy temperaturę z czujnika
		float temp = Sensors_GetTemperature();

		if (temp <= -100.0f) {
			ssd1306_SetCursor(5, 5);
			ssd1306_WriteString("E01", Font_16x26, White);
		} else {
			sprintf(bufor, "%.1f C", temp);
			ssd1306_SetCursor(5, 5);
			ssd1306_WriteString(bufor, Font_11x18, White);
		}

		ssd1306_UpdateScreen();

		osDelay(1000);
	}
	/* USER CODE END StartOled */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
 * @brief Function implementing the SensorTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument) {
	/* USER CODE BEGIN StartSensorTask */
//HAL_TIM_Base_Start_IT(&htim3);
	Sensors_Init(&htim3);
	/* Infinite loop */
	for (;;) {
		Sensors_Process();
		//HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);

		osDelay(800);
	}
	/* USER CODE END StartSensorTask */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
 * @brief Function implementing the MotorTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void *argument) {
	/* USER CODE BEGIN StartMotorTask */
	//Motor_Init();
	uint32_t tick_count = osKernelGetTickCount();
	/* Infinite loop */
	for (;;) {
		//Motor_Process();
		tick_count += 10;
		osDelayUntil(tick_count);
	}
	/* USER CODE END StartMotorTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


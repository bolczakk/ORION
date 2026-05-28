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
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include "OLED_1in5.h"
#include "GUI_Paint.h"
#include "fonts.h"
#include "udp_comm.h"
#include "shared_data.h"

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
/* Definitions for LwipUDP */
osThreadId_t LwipUDPHandle;
const osThreadAttr_t LwipUDP_attributes = {
  .name = "LwipUDP",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for renderDisplay */
osThreadId_t renderDisplayHandle;
const osThreadAttr_t renderDisplay_attributes = {
  .name = "renderDisplay",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartLwipUDP(void *argument);
void StartRenderDisplay(void *argument);

extern void MX_LWIP_Init(void);
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
  /* creation of LwipUDP */
  LwipUDPHandle = osThreadNew(StartLwipUDP, NULL, &LwipUDP_attributes);

  /* creation of renderDisplay */
  renderDisplayHandle = osThreadNew(StartRenderDisplay, NULL, &renderDisplay_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartLwipUDP */
/**
 * @brief  Function implementing the LwipUDP thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLwipUDP */
void StartLwipUDP(void *argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN StartLwipUDP */
	osDelay(3000);
	UDP_Server_Task();
	/* Infinite loop */
	for (;;) {
		osDelay(1000);
	}
  /* USER CODE END StartLwipUDP */
}

/* USER CODE BEGIN Header_StartRenderDisplay */
/**
 * @brief Function implementing the renderDisplay thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartRenderDisplay */
void StartRenderDisplay(void *argument)
{
  /* USER CODE BEGIN StartRenderDisplay */
	static UBYTE BlackImage[8192];

	System_Init();
	OLED_1in5_Init();
	osDelay(100);
	OLED_1in5_Clear();

	Paint_NewImage(BlackImage, OLED_1in5_WIDTH, OLED_1in5_HEIGHT, 0, BLACK);
	Paint_SetScale(16);
	Paint_SelectImage(BlackImage);
	Paint_Clear(BLACK);

	char text_buffer[30];

	/* Infinite loop */
	for (;;) {
		int32_t speed_int = (int32_t) (SHARED_DATA->m4_current_speed * 10);
		sprintf(text_buffer, "Velocity: %d.%d cm", (int) (speed_int / 10),
				(int) (speed_int % 10));
		Paint_DrawString_EN(10, 60, text_buffer, &Font12, WHITE,
		BLACK);
		OLED_1in5_Display(BlackImage);
		speed_int = speed_int + 1;

		osDelay(1000);
	}
  /* USER CODE END StartRenderDisplay */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


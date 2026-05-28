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

#include "tim.h"
#include "arm_math.h"
#include "pwm.h"
#include "motors.h"
#include "pid.h"
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

volatile float motor1_rpm = 0.0f;

const float32_t iirCoeffs32[5] = { 0.020083f, 0.040167f, 0.020083f, 1.561018f,
		-0.641351f };

/* USER CODE END Variables */
/* Definitions for calculateRPM */
osThreadId_t calculateRPMHandle;
const osThreadAttr_t calculateRPM_attributes = {
  .name = "calculateRPM",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for calculatePID */
osThreadId_t calculatePIDHandle;
const osThreadAttr_t calculatePID_attributes = {
  .name = "calculatePID",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for distanceSensor */
osThreadId_t distanceSensorHandle;
const osThreadAttr_t distanceSensor_attributes = {
  .name = "distanceSensor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartCalculateRPM(void *argument);
void StartCalculatePID(void *argument);
void StartDistanceSensor(void *argument);

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
  /* creation of calculateRPM */
  calculateRPMHandle = osThreadNew(StartCalculateRPM, NULL, &calculateRPM_attributes);

  /* creation of calculatePID */
  calculatePIDHandle = osThreadNew(StartCalculatePID, NULL, &calculatePID_attributes);

  /* creation of distanceSensor */
  distanceSensorHandle = osThreadNew(StartDistanceSensor, NULL, &distanceSensor_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartCalculateRPM */
/**
 * @brief  Function implementing the calculateRPM thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartCalculateRPM */
void StartCalculateRPM(void *argument)
{
  /* USER CODE BEGIN StartCalculateRPM */
	const uint32_t dt_millis = 20;
	uint16_t prev_imp = (uint16_t) __HAL_TIM_GET_COUNTER(&htim3);

	arm_biquad_casd_df1_inst_f32 iir_filter;
	float32_t iir_state[4] = { 0.0f, 0.0f, 0.0f, 0.0f };

	arm_biquad_cascade_df1_init_f32(&iir_filter, 1, (float32_t*) iirCoeffs32,
			iir_state);

	float32_t raw_rpm = 0.0f;
	float32_t filtered_rpm = 0.0f;
	/* Infinite loop */
	for (;;) {
		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		uint16_t now_imp = (uint16_t) __HAL_TIM_GET_COUNTER(&htim3);
		int16_t dt_imp = (int16_t) (now_imp - (uint16_t) prev_imp);
		prev_imp = now_imp;

		raw_rpm = ((float) dt_imp / 1920.0f)
				* (60.0f / ((float) dt_millis / 1000.0f));
		arm_biquad_cascade_df1_f32(&iir_filter, &raw_rpm, &filtered_rpm, 1);

		motor1_rpm = filtered_rpm;
		SHARED_DATA->m4_current_speed = motor1_rpm;

		osDelay(dt_millis);
	}
  /* USER CODE END StartCalculateRPM */
}

/* USER CODE BEGIN Header_StartCalculatePID */
/**
 * @brief Function implementing the calculatePID thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartCalculatePID */
void StartCalculatePID(void *argument)
{
  /* USER CODE BEGIN StartCalculatePID */

	PID_Init();
	float setpoint = 10.0f;
	Motors_Init();
	/* Infinite loop */
	for (;;) {
		setpoint = SHARED_DATA->m7_setpoint;
		float error = setpoint - motor1_rpm;
		float duty = getOutput(error);
		if (duty >= 0.0f) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
			duty = -duty;
		}

		Motor_SetDuty(MOTOR_LEFT, duty);
		osDelay(20);
	}
  /* USER CODE END StartCalculatePID */
}

/* USER CODE BEGIN Header_StartDistanceSensor */
/**
* @brief Function implementing the distanceSensor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDistanceSensor */
void StartDistanceSensor(void *argument)
{
  /* USER CODE BEGIN StartDistanceSensor */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDistanceSensor */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


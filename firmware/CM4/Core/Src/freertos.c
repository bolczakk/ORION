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

#include "arm_math.h"
#include "bme680.h"
#include "i2c.h"
#include "motors.h"
#include "pid.h"
#include "pwm.h"
#include "shared_data.h"
#include "tim.h"
#include "vl6180x.h"
#include "ws2812b.h"

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
const float32_t iirCoeffs32[5] = { 0.020083f, 0.040167f, 0.020083f, 1.561018f,
		-0.641351f };

//volatile uint8_t distance_cm = 0;
volatile float motor_left_rpm_g = 0.0f;
volatile float motor_right_rpm_g = 0.0f;

/* USER CODE END Variables */
/* Definitions for calculateRPM */
osThreadId_t calculateRPMHandle;
const osThreadAttr_t calculateRPM_attributes = { .name = "calculateRPM",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for calculatePID */
osThreadId_t calculatePIDHandle;
const osThreadAttr_t calculatePID_attributes = { .name = "calculatePID",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityLow, };
/* Definitions for distanceSensor */
osThreadId_t distanceSensorHandle;
const osThreadAttr_t distanceSensor_attributes = { .name = "distanceSensor",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityLow, };
/* Definitions for bmeSensor */
osThreadId_t bmeSensorHandle;
const osThreadAttr_t bmeSensor_attributes = { .name = "bmeSensor", .stack_size =
		512 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for LEDTask */
osThreadId_t LEDTaskHandle;
const osThreadAttr_t LEDTask_attributes = { .name = "LEDTask", .stack_size = 256
		* 4, .priority = (osPriority_t) osPriorityLow, };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartCalculateRPM(void *argument);
void StartCalculatePID(void *argument);
void StartDistanceSensor(void *argument);
void StartBmeSensor(void *argument);
void StartLEDTask(void *argument);

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
	calculateRPMHandle = osThreadNew(StartCalculateRPM, NULL,
			&calculateRPM_attributes);

	/* creation of calculatePID */
	calculatePIDHandle = osThreadNew(StartCalculatePID, NULL,
			&calculatePID_attributes);

	/* creation of distanceSensor */
	distanceSensorHandle = osThreadNew(StartDistanceSensor, NULL,
			&distanceSensor_attributes);

	/* creation of bmeSensor */
	bmeSensorHandle = osThreadNew(StartBmeSensor, NULL, &bmeSensor_attributes);

	/* creation of LEDTask */
	LEDTaskHandle = osThreadNew(StartLEDTask, NULL, &LEDTask_attributes);

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
void StartCalculateRPM(void *argument) {
	/* USER CODE BEGIN StartCalculateRPM */
	const uint32_t dt_millis = 20;
	uint16_t prev_imp_left = (uint16_t) __HAL_TIM_GET_COUNTER(&htim3);
	uint16_t prev_imp_right = (uint16_t) __HAL_TIM_GET_COUNTER(&htim1);

	arm_biquad_casd_df1_inst_f32 iir_filter_left;
	arm_biquad_casd_df1_inst_f32 iir_filter_right;

	float32_t iir_state_left[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
	float32_t iir_state_right[4] = { 0.0f, 0.0f, 0.0f, 0.0f };

	arm_biquad_cascade_df1_init_f32(&iir_filter_left, 1,
			(float32_t*) iirCoeffs32, iir_state_left);
	arm_biquad_cascade_df1_init_f32(&iir_filter_right, 1,
			(float32_t*) iirCoeffs32, iir_state_right);

	float32_t raw_rpm_left = 0.0f, filtered_rpm_left = 0.0f;
	float32_t raw_rpm_right = 0.0f, filtered_rpm_right = 0.0f;

	//UZUPELNIC
	const float WHEEL_DIAMETER_M = 0.065f; // np. 65 mm = 0.065 m
	const float TRACK_WIDTH_M = 0.150f;    // np. 150 mm = 0.15 m (rozstaw kół)
	const float TICKS_PER_REV = 1920.0f;

	const float METERS_PER_TICK = (PI * WHEEL_DIAMETER_M) / TICKS_PER_REV;

	float robot_angle_rad = 0.0f;
	/* Infinite loop */
	for (;;) {
		// LEFT MOTOR
		uint16_t now_imp_left = (uint16_t) __HAL_TIM_GET_COUNTER(&htim3);
		int16_t dt_imp_left =
				(int16_t) (now_imp_left - (uint16_t) prev_imp_left);
		prev_imp_left = now_imp_left;

		raw_rpm_left = ((float) dt_imp_left / 1920.0f)
				* (60.0f / ((float) dt_millis / 1000.0f));

		arm_biquad_cascade_df1_f32(&iir_filter_left, &raw_rpm_left,
				&filtered_rpm_left, 1);
		SHARED_DATA->m4_motor_left_rpm = filtered_rpm_left;
		motor_left_rpm_g = filtered_rpm_left;

		// RIGHT MOTOR
		uint16_t now_imp_right = (uint16_t) __HAL_TIM_GET_COUNTER(&htim1);
		int16_t dt_imp_right = (int16_t) (now_imp_right
				- (uint16_t) prev_imp_right);
		prev_imp_right = now_imp_right;

		raw_rpm_right = ((float) dt_imp_right / 1920.0f)
				* (60.0f / ((float) dt_millis / 1000.0f));

		arm_biquad_cascade_df1_f32(&iir_filter_right, &raw_rpm_right,
				&filtered_rpm_right, 1);
		SHARED_DATA->m4_motor_right_rpm = filtered_rpm_right;

		// NASZ "IMU"
		float dist_left = (float) dt_imp_left * METERS_PER_TICK;
		float dist_right = (float) dt_imp_right * METERS_PER_TICK;

		float delta_theta = (dist_right - dist_left) / TRACK_WIDTH_M;

		robot_angle_rad += delta_theta;

		if (robot_angle_rad > PI)
			robot_angle_rad -= 2.0f * PI;
		if (robot_angle_rad < -PI)
			robot_angle_rad += 2.0f * PI;

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
void StartCalculatePID(void *argument) {
	/* USER CODE BEGIN StartCalculatePID */

	PID_Init();
	float setpoint = 0.0f;
	Motors_Init();
	/* Infinite loop */
	for (;;) {
		float current_distance = SHARED_DATA->m4_distance;
		float current_rpm_left = motor_left_rpm_g;
		float current_rpm_right = motor_right_rpm_g;
		float duty_left = 0.0f;
		float duty_right = 0.0f;

		if (current_distance > 20.0f) {
			setpoint = 0.0f;
			duty_left = 0.0f;
			duty_right = 0.0f;
		} else {
			setpoint = SHARED_DATA->m7_setpoint;

			float error_left = setpoint - current_rpm_left;
			float error_right = setpoint - current_rpm_right;

			duty_left = getOutputLeft(error_left);
			duty_right = getOutputRight(error_right);
		}

		if (duty_left >= 0.0f) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
			duty_left = -duty_left;
		}
		Motor_SetDuty(MOTOR_LEFT, duty_left);

		if (duty_right >= 0.0f) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
			duty_right = -duty_right;
		}
		Motor_SetDuty(MOTOR_RIGHT, duty_right);

		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
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
void StartDistanceSensor(void *argument) {
	/* USER CODE BEGIN StartDistanceSensor */
	VL6180X_Init(&hi2c1);
	VL6180X_SetScaling2x(&hi2c1);

	VL6180X_Write8(&hi2c1, 0x0014, 0x04);
	VL6180X_Write8(&hi2c1, 0x001B, 0x0A);
	VL6180X_Write8(&hi2c1, 0x0015, 0x07);
	VL6180X_Write8(&hi2c1, 0x0018, 0x03);
	uint8_t range = 0;
	/* Infinite loop */
	for (;;) {
		uint32_t flags = osThreadFlagsWait(0x0001, osFlagsWaitAny, 200);

		if (flags == 0x0001) {
			uint8_t status = 0;
			VL6180X_Read8(&hi2c1, 0x004F, &status);
			if (VL6180X_Read8(&hi2c1, 0x0062, &range) == HAL_OK) {
				if (range != 255) {
					uint8_t distance_cm = ((float) range * 2.0f) / 10.0f;
					SHARED_DATA->m4_distance = distance_cm;
				} else {
					SHARED_DATA->m4_distance = -1.0f;
				}
			}
			VL6180X_Write8(&hi2c1, 0x0015, 0x07);

		} else {
			VL6180X_Init(&hi2c1);
			VL6180X_SetScaling2x(&hi2c1);

			VL6180X_Write8(&hi2c1, 0x0014, 0x04);
			VL6180X_Write8(&hi2c1, 0x001B, 0x05);
			VL6180X_Write8(&hi2c1, 0x0015, 0x07);
			VL6180X_Write8(&hi2c1, 0x0018, 0x03);
		}
	}
	/* USER CODE END StartDistanceSensor */
}

/* USER CODE BEGIN Header_StartBmeSensor */
/**
 * @brief Function implementing the bmeSensor thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartBmeSensor */
void StartBmeSensor(void *argument) {
	/* USER CODE BEGIN StartBmeSensor */
	struct bme68x_dev bme;
	struct bme68x_conf conf;
	struct bme68x_heatr_conf heatr_conf;
	int8_t rslt;

	bme.read = bme68x_i2c_read;
	bme.write = bme68x_i2c_write;
	bme.delay_us = bme68x_delay_us;
	bme.intf = BME68X_I2C_INTF;
	bme.intf_ptr = &bme_dev_addr;
	bme.amb_temp = 25;

	rslt = bme68x_init(&bme);

	if (rslt != BME68X_OK) {
		SHARED_DATA->current_error = ERR_BME_INIT_FAIL;
		while (1) {
			osDelay(2000);

			rslt = bme68x_init(&bme);
			if (rslt == BME68X_OK) {
				SHARED_DATA->current_error = STATUS_OK;
				break;
			}
		}
	}

	conf.filter = BME68X_FILTER_SIZE_3;
	conf.odr = BME68X_ODR_NONE;
	conf.os_hum = BME68X_OS_16X;
	conf.os_pres = BME68X_OS_1X;
	conf.os_temp = BME68X_OS_2X;
	bme68x_set_conf(&conf, &bme);

	heatr_conf.enable = BME68X_ENABLE;
	heatr_conf.heatr_temp = 320; // 320 stopni C
	heatr_conf.heatr_dur = 150;  // 150 ms
	bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);
	/* Infinite loop */
	for (;;) {
		bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);

		uint32_t del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf,
				&bme) + (heatr_conf.heatr_dur * 1000);
		bme.delay_us(del_period, bme.intf_ptr);

		uint8_t n_fields;
		rslt = bme68x_get_data(BME68X_FORCED_MODE, &bme_last_data, &n_fields,
				&bme);

		if (rslt == BME68X_OK && n_fields > 0) {
			SHARED_DATA->m4_temperature = bme_last_data.temperature;
			SHARED_DATA->m4_humidity = bme_last_data.humidity;
			SHARED_DATA->m4_pressure = bme_last_data.pressure;
		}

		osDelay(2000);
	}
	/* USER CODE END StartBmeSensor */
}

/* USER CODE BEGIN Header_StartLEDTask */
/**
 * @brief Function implementing the LEDTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLEDTask */
void StartLEDTask(void *argument) {
	/* USER CODE BEGIN StartLEDTask */
//	WS2812_Set_LED(0, 255, 0, 0); // Dioda 0 na czerwono
//	WS2812_Set_LED(1, 0, 255, 0); // Dioda 1 na zielono
//	WS2812_Set_LED(2, 0, 0, 255); // Dioda 2 na niebiesko
//	WS2812_Set_Brightness(100);    // Ustawienie jasności na 20%
//	WS2812_Send();
	/* Infinite loop */
	for (;;) {
		osDelay(100);
	}
	/* USER CODE END StartLEDTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


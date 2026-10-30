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
#include "distance_sensors.h"
#include "ws2812b.h"
#include "vl53lx_api.h"
#include "imu.h"
#include "shared_ipc.h"
#include "ultrasonic_sensor.h"
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define WHEEL_DIAMETER_M 0.096f // 96mm
#define TRACK_WIDTH_M    0.274f
#define TICKS_PER_REV 1920.0f

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
const float32_t iirCoeffs32[5] = { 0.020083f, 0.040167f, 0.020083f, 1.561018f,
		-0.641351f };

volatile float motor_left_rpm_g = 0.0f;
volatile float motor_right_rpm_g = 0.0f;

static osThreadId_t ultrasonicSensorHandle;
static const osThreadAttr_t ultrasonicSensor_attributes = { .name =
		"ultrasonicSensor", .stack_size = 1024U, .priority = osPriorityNormal };

/* USER CODE END Variables */
/* Definitions for driveControl */
osThreadId_t driveControlHandle;
const osThreadAttr_t driveControl_attributes = { .name = "driveControl",
		.stack_size = 256 * 4, .priority = (osPriority_t) osPriorityRealtime, };
/* Definitions for distanceSensor */
osThreadId_t distanceSensorHandle;
const osThreadAttr_t distanceSensor_attributes = { .name = "distanceSensor",
		.stack_size = 4096, .priority = (osPriority_t) osPriorityHigh, };
/* Definitions for bmeSensor */
osThreadId_t bmeSensorHandle;
const osThreadAttr_t bmeSensor_attributes = { .name = "bmeSensor", .stack_size =
		512 * 4, .priority = (osPriority_t) osPriorityLow1, };
/* Definitions for ledController */
osThreadId_t ledControllerHandle;
const osThreadAttr_t ledController_attributes =
		{ .name = "ledController", .stack_size = 256 * 4, .priority =
				(osPriority_t) osPriorityBelowNormal, };
/* Definitions for calcBuzzer */
osThreadId_t calcBuzzerHandle;
const osThreadAttr_t calcBuzzer_attributes = { .name = "calcBuzzer",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityLow, };
/* Definitions for imuSensor */
osThreadId_t imuSensorHandle;
const osThreadAttr_t imuSensor_attributes = { .name = "imuSensor", .stack_size =
		256 * 4, .priority = (osPriority_t) osPriorityHigh, };
/* Definitions for i2c2Mutex */
osMutexId_t i2c2MutexHandle;
const osMutexAttr_t i2c2Mutex_attributes = { .name = "i2c2Mutex" };
/* Definitions for imuMutex */
osMutexId_t imuMutexHandle;
const osMutexAttr_t imuMutex_attributes = { .name = "imuMutex" };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDriveControl(void *argument);
void StartDistanceSensor(void *argument);
void StartBmeSensor(void *argument);
void StartLedController(void *argument);
void StartCalcBuzzer(void *argument);
void StartImuSensor(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */
	/* Create the mutex(es) */
	/* creation of i2c2Mutex */
	i2c2MutexHandle = osMutexNew(&i2c2Mutex_attributes);

	/* creation of imuMutex */
	imuMutexHandle = osMutexNew(&imuMutex_attributes);

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
	/* creation of driveControl */
	driveControlHandle = osThreadNew(StartDriveControl, NULL,
			&driveControl_attributes);

	/* creation of distanceSensor */
	distanceSensorHandle = osThreadNew(StartDistanceSensor, NULL,
			&distanceSensor_attributes);

	/* creation of bmeSensor */
	bmeSensorHandle = osThreadNew(StartBmeSensor, NULL, &bmeSensor_attributes);

	/* creation of ledController */
	ledControllerHandle = osThreadNew(StartLedController, NULL,
			&ledController_attributes);

	/* creation of calcBuzzer */
	calcBuzzerHandle = osThreadNew(StartCalcBuzzer, NULL,
			&calcBuzzer_attributes);

	/* creation of imuSensor */
	imuSensorHandle = osThreadNew(StartImuSensor, NULL, &imuSensor_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */

	ultrasonicSensorHandle = osThreadNew(StartUltrasonicSensor, NULL,
			&ultrasonicSensor_attributes);
	if (ultrasonicSensorHandle == NULL) {
		Error_Handler();
	}
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDriveControl */
/**
 * @brief  Function implementing the driveControl thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDriveControl */
void StartDriveControl(void *argument) {
	/* USER CODE BEGIN StartDriveControl */
	PID_Init();
	Motors_Init();
	const uint32_t dt_millis = 50;
	uint16_t prev_imp_left = (uint16_t) __HAL_TIM_GET_COUNTER(&htim1);
	uint16_t prev_imp_right = (uint16_t) __HAL_TIM_GET_COUNTER(&htim3);

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

	const float METERS_PER_TICK = (PI * WHEEL_DIAMETER_M) / TICKS_PER_REV;

	float robot_angle_rad = 0.0f;
	float robot_pos_x = 0.0f;
	float robot_pos_y = 0.0f;
	/* Infinite loop */
	for (;;) {
		// LEFT MOTOR
		uint16_t now_imp_left = (uint16_t) __HAL_TIM_GET_COUNTER(&htim1);
		int16_t dt_imp_left =
				(int16_t) (now_imp_left - (uint16_t) prev_imp_left);
		prev_imp_left = now_imp_left;

		raw_rpm_left = ((float) dt_imp_left / TICKS_PER_REV)
				* (60.0f / ((float) dt_millis / 1000.0f));

		arm_biquad_cascade_df1_f32(&iir_filter_left, &raw_rpm_left,
				&filtered_rpm_left, 1);
//		SHARED_DATA->m4_motor_left_rpm = filtered_rpm_left;
		motor_left_rpm_g = filtered_rpm_left;

		// RIGHT MOTOR
		uint16_t now_imp_right = (uint16_t) __HAL_TIM_GET_COUNTER(&htim3);
		int16_t dt_imp_right = (int16_t) (now_imp_right
				- (uint16_t) prev_imp_right);
		prev_imp_right = now_imp_right;

		raw_rpm_right = ((float) dt_imp_right / TICKS_PER_REV)
				* (60.0f / ((float) dt_millis / 1000.0f));

		arm_biquad_cascade_df1_f32(&iir_filter_right, &raw_rpm_right,
				&filtered_rpm_right, 1);
//		SHARED_DATA->m4_motor_right_rpm = filtered_rpm_right;
		motor_right_rpm_g = filtered_rpm_right;

		// IMU
		float dist_left = (float) dt_imp_left * METERS_PER_TICK;
		float dist_right = (float) dt_imp_right * METERS_PER_TICK;

		float delta_distance = (dist_right + dist_left) / 2.0f;
		float delta_theta = (dist_right - dist_left) / TRACK_WIDTH_M;

		robot_angle_rad += delta_theta;

		if (robot_angle_rad > PI)
			robot_angle_rad -= 2.0f * PI;
		if (robot_angle_rad < -PI)
			robot_angle_rad += 2.0f * PI;

//		SHARED_DATA->m4_angle = robot_angle_rad;

		robot_pos_x += delta_distance
				* cos(robot_angle_rad + (delta_theta / 2.0f));
		robot_pos_y += delta_distance
				* sin(robot_angle_rad + (delta_theta / 2.0f));

//		SHARED_DATA->m4_pos_x = robot_pos_x;
//		SHARED_DATA->m4_pos_y = robot_pos_y;

		Shared_PublishOdometry(robot_pos_x, robot_pos_y, robot_angle_rad,
				filtered_rpm_left, filtered_rpm_right);

//		float v = SHARED_DATA->m7_linear_speed; // TU ZMIENIC
//		float omega = SHARED_DATA->m7_angular_speed;

		SharedCommand_t command;

		float v = 0.0f;
		float omega = 0.0f;

		if (Shared_TryReadCommand(&command)) {
			v = command.linear_speed;
			omega = command.angular_speed;
		}

		v = 0.2f;
		omega = 0.0f;

		float v_left_mps = v - (omega * TRACK_WIDTH_M / 2.0f);
		float v_right_mps = v + (omega * TRACK_WIDTH_M / 2.0f);

		float wheel_circumference = PI * WHEEL_DIAMETER_M;
		float setpoint_left_rpm = (v_left_mps / wheel_circumference) * 60.0f;
		float setpoint_right_rpm = (v_right_mps / wheel_circumference) * 60.0f;

		float error_left = setpoint_left_rpm - filtered_rpm_left;
		float error_right = setpoint_right_rpm - filtered_rpm_right;

		float duty_left = getOutputLeft(error_left);
		float duty_right = getOutputRight(error_right);

//		duty_left = 50.0f;
//		duty_right = 50.0f;

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

		osDelay(dt_millis);
	}
	/* USER CODE END StartDriveControl */
}

/* USER CODE BEGIN Header_StartDistanceSensor */
/**
 * @brief Function implementing the distanceSensor thread.
 * @param argument: Not used
 * @retval None
 * It is for i2c sensors, the uart one is handled fully in ultrasonic_sensor.c (the task is there)
 */
/* USER CODE END Header_StartDistanceSensor */
void StartDistanceSensor(void *argument) {
	/* USER CODE BEGIN StartDistanceSensor */

	(void) argument;
	DistanceSensors_Init();

	/* Infinite loop */
	for (;;) {
		uint32_t flags = osThreadFlagsWait(DISTANCE_SENSORS_READY_FLAG,
		osFlagsWaitAny, pdMS_TO_TICKS(DISTANCE_SENSORS_POLL_MS));
		if ((flags & osFlagsError) != 0U && flags != osFlagsErrorTimeout) {
			/* A bad flags setup must not create a high-priority busy loop. */
			(void) osDelay(pdMS_TO_TICKS(DISTANCE_SENSORS_POLL_MS));
		}
		DistanceSensors_Process();

		//osDelay(20);
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
	(void) argument;
	struct bme68x_dev bme = { 0 };
	struct bme68x_conf conf = { 0 };
	struct bme68x_heatr_conf heatr_conf = { 0 };
	int8_t rslt;

	bme.read = bme68x_i2c_read;
	bme.write = bme68x_i2c_write;
	bme.delay_us = bme68x_delay_us;
	bme.intf = BME68X_I2C_INTF;
	bme.intf_ptr = &bme_dev_addr;
	bme.amb_temp = 25;

	conf.filter = BME68X_FILTER_SIZE_3;
	conf.odr = BME68X_ODR_NONE;
	conf.os_hum = BME68X_OS_16X;
	conf.os_pres = BME68X_OS_1X;
	conf.os_temp = BME68X_OS_2X;
	heatr_conf.enable = BME68X_ENABLE;
	heatr_conf.heatr_temp = 320;
	heatr_conf.heatr_dur = 150;

	for (;;) {
		/* Chronimy wszystkie operacje BME wykonujace I2C, takze init/config.
		 * Callbacki bme68x_i2c_read/write nie powinny ponownie brac tego mutexu.
		 */
		if (osMutexAcquire(i2c2MutexHandle, osWaitForever) != osOK) {
			osDelay(pdMS_TO_TICKS(2000U));
			continue;
		}
		rslt = bme68x_init(&bme);
		if (rslt == BME68X_OK)
			rslt = bme68x_set_conf(&conf, &bme);
		if (rslt == BME68X_OK) {
			rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);
		}
		(void) osMutexRelease(i2c2MutexHandle);

		if (rslt != BME68X_OK) {
			SHARED_DATA->current_error = ERR_BME_INIT_FAIL;
			osDelay(pdMS_TO_TICKS(2000U));
			continue;
		}
		/* Nie kasuj bledow ustawionych przez inne taski. */
		if (SHARED_DATA->current_error == ERR_BME_INIT_FAIL) {
			SHARED_DATA->current_error = STATUS_OK;
		}

		for (;;) {
			if (osMutexAcquire(i2c2MutexHandle, osWaitForever) != osOK)
				break;
			rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
			(void) osMutexRelease(i2c2MutexHandle);
			if (rslt != BME68X_OK)
				break;

			uint32_t del_period_us = bme68x_get_meas_dur(
			BME68X_FORCED_MODE, &conf, &bme)
					+ (uint32_t) heatr_conf.heatr_dur * 1000U;
			/* W czasie pomiaru/grzania BME mutex jest WOLNY i task spi.
			 * osDelay przyjmuje ticki RTOS, nie milisekundy.
			 */
			/* Dodatkowy tick pokrywa niepelny pierwszy tick osDelay(). */
			osDelay(pdMS_TO_TICKS((del_period_us + 999U) / 1000U) + 1U);

			uint8_t n_fields = 0;
			if (osMutexAcquire(i2c2MutexHandle, osWaitForever) != osOK)
				break;
			rslt = bme68x_get_data(BME68X_FORCED_MODE, &bme_last_data,
					&n_fields, &bme);
			(void) osMutexRelease(i2c2MutexHandle);
			if (rslt < BME68X_OK)
				break;

			if (rslt == BME68X_OK && n_fields > 0U) {
				SHARED_DATA->m4_temperature = bme_last_data.temperature;
				SHARED_DATA->m4_humidity = bme_last_data.humidity;
				SHARED_DATA->m4_pressure = bme_last_data.pressure;
			}
			osDelay(pdMS_TO_TICKS(2000U));
		}
		SHARED_DATA->current_error = ERR_BME_INIT_FAIL;
		osDelay(pdMS_TO_TICKS(2000U));
	}
	/* USER CODE END StartBmeSensor */
}

/* USER CODE BEGIN Header_StartLedController */
/**
 * @brief Function implementing the LedController thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLedController */
void StartLedController(void *argument) {
	/* USER CODE BEGIN StartLedController */
	(void) argument;

	// Jasność ustalona bezpośrednio wartościami RGB (0–255).
	const uint8_t colors[][3] = { { 45, 0, 0 },   // czerwony
			{ 0, 45, 0 },   // zielony
			{ 0, 0, 45 },   // niebieski
			{ 45, 45, 0 },   // żółty
			{ 0, 45, 45 },   // cyjan
			{ 45, 0, 45 }    // fioletowy
	};

	const TickType_t step_ticks = pdMS_TO_TICKS(50U) + 1U;
	const int total_leds =
	NUM_LEDS_FRONT_LEFT +
	NUM_LEDS_FRONT_RIGHT +
	NUM_LEDS_REAR;

	unsigned int color = 0;

	for (;;) {
		// Wygaś wszystkie diody przed kolejnym cyklem.
		for (int i = 0; i < NUM_LEDS_FRONT_LEFT; i++) {
			SetFrontLeftLED(i, 0, 0, 0);
		}
		for (int i = 0; i < NUM_LEDS_FRONT_RIGHT; i++) {
			SetFrontRightLED(i, 0, 0, 0);
		}
		for (int i = 0; i < NUM_LEDS_REAR; i++) {
			SetRearLED(i, 0, 0, 0);
		}

		// TUTAJ wywołaj swoje funkcje wysyłające dane do pasków.
		//WS2812_Send();

		vTaskDelay(step_ticks);

		const uint8_t r = colors[color][0];
		const uint8_t g = colors[color][1];
		const uint8_t b = colors[color][2];

		for (int i = 0; i < total_leds; i++) {
			if (i < NUM_LEDS_FRONT_LEFT) {
				SetFrontLeftLED(i, r, g, b);
			} else if (i < NUM_LEDS_FRONT_LEFT + NUM_LEDS_FRONT_RIGHT) {
				SetFrontRightLED(i - NUM_LEDS_FRONT_LEFT, r, g, b);
			} else {
				SetRearLED(i - NUM_LEDS_FRONT_LEFT - NUM_LEDS_FRONT_RIGHT, r, g,
						b);
			}

			// TUTAJ wywołaj swoje funkcje wysyłające dane do pasków.
			//WS2812_Send();
			vTaskDelay(step_ticks);
		}

		// Krótka pauza, gdy świecą już wszystkie diody.
		vTaskDelay(pdMS_TO_TICKS(300U) + 1U);

		color = (color + 1U) / 1U % (sizeof(colors) / sizeof(colors[0]));
	}
	/* USER CODE END StartLedController */
}

/* USER CODE BEGIN Header_StartCalcBuzzer */
/**
 * @brief Function implementing the calcBuzzer thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartCalcBuzzer */
void StartCalcBuzzer(void *argument) {
	/* USER CODE BEGIN StartCalcBuzzer */
	/* Infinite loop */
	for (;;) {
		//HAL_GPIO_TogglePin(BUZZ_GPIO_Port, BUZZ_Pin);
		osDelay(1000);
	}
	/* USER CODE END StartCalcBuzzer */
}

/* USER CODE BEGIN Header_StartImuSensor */
/**
 * @brief Function implementing the imuSensor thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartImuSensor */
void StartImuSensor(void *argument) {
	/* USER CODE BEGIN StartImuSensor */
	ImuSample_t sample = { 0 };
	uint8_t initialized = 0U;
	uint8_t consecutive_errors = 0U;
	uint32_t last_data_ms = 0U;

	ImuPublish(&sample);

	osDelay(pdMS_TO_TICKS(20U));
	/* Infinite loop */
	for (;;) {
		osDelay(100);
	}
	/* USER CODE END StartImuSensor */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
// wakes up i2c distance sensors task
void DistanceSensors_NotifyReadyFromISR(void) {
	if (distanceSensorHandle != NULL) {
		(void) osThreadFlagsSet(distanceSensorHandle,
		DISTANCE_SENSORS_READY_FLAG);
	}
}

/* USER CODE END Application */


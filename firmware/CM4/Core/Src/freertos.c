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
#include "drive_control.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define WHEEL_DIAMETER_M 0.1f // 90mm
#define TRACK_WIDTH_M    0.274f
#define TICKS_PER_REV 3840.0f

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
const float32_t iirCoeffs32[5] = { 0.020083f, 0.040167f, 0.020083f, 1.561018f,
		-0.641351f };

//volatile float motor_left_rpm_g = 0.0f;
//volatile float motor_right_rpm_g = 0.0f;

//volatile float motor_left_raw_rpm_g = 0.0f;
//volatile float motor_right_raw_rpm_g = 0.0f;
//volatile float motor_left_setpoint_rpm_g = 0.0f;
//volatile float motor_right_setpoint_rpm_g = 0.0f;
//volatile float motor_left_pwm_g = 0.0f;
//volatile float motor_right_pwm_g = 0.0f;
//volatile float drive_straight_error_m_g = 0.0f;
//volatile uint32_t drive_elapsed_ms_g = 0U;
//volatile uint32_t drive_command_valid_g = 0U;

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
		.stack_size = 1024 * 4, .priority = (osPriority_t) osPriorityHigh, };
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
/* Definitions for gasSensor */
osThreadId_t gasSensorHandle;
const osThreadAttr_t gasSensor_attributes = { .name = "gasSensor", .stack_size =
		128 * 4, .priority = (osPriority_t) osPriorityLow, };
/* Definitions for i2c2Mutex */
osMutexId_t i2c2MutexHandle;
const osMutexAttr_t i2c2Mutex_attributes = { .name = "i2c2Mutex" };
/* Definitions for imuMutex */
osMutexId_t imuMutexHandle;
const osMutexAttr_t imuMutex_attributes = { .name = "imuMutex" };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static void Drive_WriteDuty(uint8_t motor, uint16_t direction_pin,
		float signed_duty) {
	if (!isfinite(signed_duty))
		signed_duty = 0.0f;
	if (signed_duty > 0.0f) {
		HAL_GPIO_WritePin(GPIOD, direction_pin, GPIO_PIN_RESET);
	} else if (signed_duty < 0.0f) {
		HAL_GPIO_WritePin(GPIOD, direction_pin, GPIO_PIN_SET);
	}
	Motor_SetDuty(motor, fabsf(signed_duty));
}

static void Drive_ForceStop(void) {
	Motor_SetDuty(MOTOR_LEFT, 0.0f);
	Motor_SetDuty(MOTOR_RIGHT, 0.0f);
	PID_Reset();
}

static float applyRamp(float current, float target, float max_accel, float dt) {
	float max_change = max_accel * dt;
	float error = target - current;

	if (error > max_change)
		error = max_change;
	else if (error < -max_change)
		error = -max_change;

	return current + error;
}

/* USER CODE END FunctionPrototypes */

void StartDriveControl(void *argument);
void StartDistanceSensor(void *argument);
void StartBmeSensor(void *argument);
void StartLedController(void *argument);
void StartCalcBuzzer(void *argument);
void StartImuSensor(void *argument);
void StartGasSensor(void *argument);

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

	/* creation of gasSensor */
	gasSensorHandle = osThreadNew(StartGasSensor, NULL, &gasSensor_attributes);

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
	(void) argument;

	PID_Init();
	Motors_Init();
	Drive_ForceStop();

	const float wheel_circumference = PI * DRIVE_WHEEL_DIAMETER_M;
	const float m_per_tick = wheel_circumference / DRIVE_TICKS_PER_REV;

	float filtered_rpm_left = 0.0f;
	float filtered_rpm_right = 0.0f;
	float ramped_linear_speed = 0.0f;
	float ramped_angular_speed = 0.0f;
	DriveStraightSync_t straight_sync = { 0 };

	TickType_t period_ticks = pdMS_TO_TICKS(DRIVE_PERIOD_MS);

	uint16_t prev_imp_left = (uint16_t) __HAL_TIM_GET_COUNTER(&htim3);
	uint16_t prev_imp_right = (uint16_t) __HAL_TIM_GET_COUNTER(&htim1);
	uint32_t previous_ms = HAL_GetTick();
	TickType_t last_wake = xTaskGetTickCount();

	for (;;) {
		vTaskDelayUntil(&last_wake, period_ticks);

		const uint32_t now_ms = HAL_GetTick();
		const uint32_t elapsed_ms = (uint32_t) (now_ms - previous_ms);
		if (elapsed_ms == 0U)
			continue;

		const uint16_t now_imp_left = (uint16_t) __HAL_TIM_GET_COUNTER(&htim3);
		const uint16_t now_imp_right = (uint16_t) __HAL_TIM_GET_COUNTER(&htim1);
		const int16_t ticks_left = (int16_t) (now_imp_left - prev_imp_left);
		const int16_t ticks_right = (int16_t) (now_imp_right - prev_imp_right);

		prev_imp_left = now_imp_left;
		prev_imp_right = now_imp_right;
		previous_ms = now_ms;

		const float dt_s = (float) elapsed_ms * 0.001f;

		// w przypadku zawieszenia dluzszego niz 100ms zerujemy wszystko
		if (dt_s > DRIVE_MAX_DT_S) {
			Drive_ForceStop();
			Drive_ResetStraightSync(&straight_sync);
			filtered_rpm_left = 0.0f;
			filtered_rpm_right = 0.0f;
			ramped_linear_speed = 0.0f;
			ramped_angular_speed = 0.0f;
			last_wake = xTaskGetTickCount();
			continue;
		}

		const float raw_rpm_left = (float) ticks_left * 60.0f
				/ (DRIVE_TICKS_PER_REV * dt_s);
		const float raw_rpm_right = (float) ticks_right * 60.0f
				/ (DRIVE_TICKS_PER_REV * dt_s);
		filtered_rpm_left = Drive_FilterRpm(filtered_rpm_left, raw_rpm_left,
				dt_s);
		filtered_rpm_right = Drive_FilterRpm(filtered_rpm_right, raw_rpm_right,
				dt_s);

		Shared_PublishEncoders(raw_rpm_left, raw_rpm_right);

		SharedCommand_t command;
		// Zachowujemy zasade: nieudany odczyt nie podtrzymuje jazdy.
		if (!Shared_TryReadCommand(&command) || !isfinite(command.linear_speed)
		|| !isfinite(command.angular_speed)) {
			Drive_ForceStop();
			Drive_ResetStraightSync(&straight_sync);
			ramped_linear_speed = 0.0f;
			ramped_angular_speed = 0.0f;
			continue;
		}

		ramped_linear_speed = applyRamp(ramped_linear_speed,
				command.linear_speed,
				DRIVE_MAX_LINEAR_ACCEL, dt_s);

		ramped_angular_speed = applyRamp(ramped_angular_speed,
				command.angular_speed,
				DRIVE_MAX_ANGULAR_ACCEL, dt_s);

		float left_mps;
		float right_mps;

		Drive_WheelTargets(&straight_sync, ramped_linear_speed,
				ramped_angular_speed, ticks_left * m_per_tick,
				ticks_right * m_per_tick, &left_mps, &right_mps);

		const float left_rpm_setpoint = left_mps * 60.0f / wheel_circumference;
		const float right_rpm_setpoint = right_mps * 60.0f
				/ wheel_circumference;

//		if (!isfinite(left_ref) || !isfinite(right_ref)) {
//			Drive_ForceStop();
//			Drive_ResetStraightSync(&straight_sync);
//			continue;
//		}

		const float duty_left = getOutputLeft(left_rpm_setpoint,
				filtered_rpm_left, dt_s);
		const float duty_right = getOutputRight(right_rpm_setpoint,
				filtered_rpm_right, dt_s);

//		motor_left_pwm_g = duty_left;
//		motor_right_pwm_g = duty_right;
		Drive_WriteDuty(MOTOR_LEFT, M_LEFT_DIR_Pin, duty_left);
		Drive_WriteDuty(MOTOR_RIGHT, M_RIGHT_DIR_Pin, duty_right);
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
		osDelay(pdMS_TO_TICKS(1000));
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

	sample.state = IMU_STARTING;
	ImuPublish(&sample);
	osDelay(ImuMsToTicks(20U));

	//ImuPublish(&sample);

	//osDelay(pdMS_TO_TICKS(20U));
	/* Infinite loop */
	for (;;) {
		if (!initialized) {
			sample.valid = 0;
			sample.state = IMU_STARTING;
			sample.address_7bit = 0U;
			ImuPublish(&sample);

			uint8_t address = 0U;
			HAL_StatusTypeDef result = ImuInit(&address);
			sample.address_7bit = address;
			if (result != HAL_OK) {
				++sample.error_count;
				sample.state = IMU_INIT_ERROR;
				ImuPublish(&sample);
				osDelay(ImuMsToTicks(IMU_RETRY_MS));
				continue;
			}

			initialized = 1U;
			consecutive_errors = 0U;
			/* Pierwsza probka ma osobny limit czasu po inicjalizacji. */
			last_data_ms = HAL_GetTick();
			ImuPublish(&sample);
		}
		uint8_t has_new_data = 0U;
		HAL_StatusTypeDef result = ImuReadSample(sample.address_7bit, &sample,
				&has_new_data);
		uint32_t now_ms = HAL_GetTick();

		if (result != HAL_OK) {
			++sample.error_count;
			++consecutive_errors;
			sample.valid = 0U;
			sample.state = IMU_IO_ERROR;
			/* Zachowaj czas i licznik ostatniego udanego pomiaru. */
			ImuPublish(&sample);

			if (consecutive_errors >= IMU_MAX_CONSECUTIVE_ERRORS) {
				initialized = 0U;
				osDelay(ImuMsToTicks(IMU_RETRY_MS));
				continue;
			}
		} else {
			/* Poprawny odczyt statusu przerywa serie bledow I2C. */
			consecutive_errors = 0U;
			if (has_new_data) {
				sample.timestamp_ms = now_ms;
				++sample.sample_count;
				sample.valid = 1U;
				sample.state = IMU_READY;
				last_data_ms = now_ms;
				ImuPublish(&sample);
				Shared_PublishImu(sample.accel_mps2, sample.gyro_radps);
			} else if ((uint32_t) (now_ms - last_data_ms) >= IMU_STALE_MS) {
				/* Brak nowych danych liczymy raz, przed ponownym startem. */
				++sample.error_count;
				sample.valid = 0U;
				sample.state = IMU_STALE;
				initialized = 0U;
				ImuPublish(&sample);
				osDelay(ImuMsToTicks(IMU_RETRY_MS));
				continue;
			}
		}
		/* Polling statusu; nowa probka powstaje nominalnie co 9.62 ms. */
		osDelay(ImuMsToTicks(IMU_POLL_MS));
	}
	/* USER CODE END StartImuSensor */
}

/* USER CODE BEGIN Header_StartGasSensor */
/**
 * @brief Function implementing the gasSensor thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartGasSensor */
void StartGasSensor(void *argument) {
	/* USER CODE BEGIN StartGasSensor */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
	/* USER CODE END StartGasSensor */
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


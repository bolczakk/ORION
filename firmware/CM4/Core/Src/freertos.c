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

#define WHEEL_DIAMETER_M 0.096f // 96mm
#define TRACK_WIDTH_M    0.274f
#define TICKS_PER_REV 1920.0f

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
const float32_t iirCoeffs32[5] = { 0.020083f, 0.040167f, 0.020083f, 1.561018f,
		-0.641351f };

//volatile uint8_t distance_cm = 0;
volatile float motor_left_rpm_g = 0.0f;
volatile float motor_right_rpm_g = 0.0f;

/* USER CODE END Variables */
/* Definitions for driveControl */
osThreadId_t driveControlHandle;
const osThreadAttr_t driveControl_attributes = { .name = "driveControl",
		.stack_size = 256 * 4, .priority = (osPriority_t) osPriorityRealtime, };
/* Definitions for distanceSensor */
osThreadId_t distanceSensorHandle;
const osThreadAttr_t distanceSensor_attributes = { .name = "distanceSensor",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityHigh, };
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

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDriveControl(void *argument);
void StartDistanceSensor(void *argument);
void StartBmeSensor(void *argument);
void StartLedController(void *argument);
void StartCalcBuzzer(void *argument);

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

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
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

	const float METERS_PER_TICK = (PI * WHEEL_DIAMETER_M) / TICKS_PER_REV;

	float robot_angle_rad = 0.0f;
	float robot_pos_x = 0.0f;
	float robot_pos_y = 0.0f;
	/* Infinite loop */
	for (;;) {
		// LEFT MOTOR
		uint16_t now_imp_left = (uint16_t) __HAL_TIM_GET_COUNTER(&htim3);
		int16_t dt_imp_left =
				(int16_t) (now_imp_left - (uint16_t) prev_imp_left);
		prev_imp_left = now_imp_left;

		raw_rpm_left = ((float) dt_imp_left / TICKS_PER_REV)
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

		raw_rpm_right = ((float) dt_imp_right / TICKS_PER_REV)
				* (60.0f / ((float) dt_millis / 1000.0f));

		arm_biquad_cascade_df1_f32(&iir_filter_right, &raw_rpm_right,
				&filtered_rpm_right, 1);
		SHARED_DATA->m4_motor_right_rpm = filtered_rpm_right;
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

		SHARED_DATA->m4_angle = robot_angle_rad;

		robot_pos_x += delta_distance
				* cos(robot_angle_rad + (delta_theta / 2.0f));
		robot_pos_y += delta_distance
				* sin(robot_angle_rad + (delta_theta / 2.0f));

		SHARED_DATA->m4_pos_x = robot_pos_x;
		SHARED_DATA->m4_pos_y = robot_pos_y;

		float v = SHARED_DATA->m7_linear_speed;
		float omega = SHARED_DATA->m7_angular_speed;

		float v_left_mps = v - (omega * TRACK_WIDTH_M / 2.0f);
		float v_right_mps = v + (omega * TRACK_WIDTH_M / 2.0f);

		float wheel_circumference = PI * WHEEL_DIAMETER_M;
		float setpoint_left_rpm = (v_left_mps / wheel_circumference) * 60.0f;
		float setpoint_right_rpm = (v_right_mps / wheel_circumference) * 60.0f;

		float error_left = setpoint_left_rpm - filtered_rpm_left;
		float error_right = setpoint_right_rpm - filtered_rpm_right;

		float duty_left = getOutputLeft(error_left);
		float duty_right = getOutputRight(error_right);

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
					SHARED_DATA->m4_distances[0] = distance_cm;
				} else {
					SHARED_DATA->m4_distances[0] = -1.0f;
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
	osDelay(20);
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

/* USER CODE BEGIN Header_StartLedController */
/**
 * @brief Function implementing the LedController thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLedController */
void StartLedController(void *argument) {
	/* USER CODE BEGIN StartLedController */
	osDelay(100);

	LedState_t current_state = LED_STATE_IDLE;
	LedState_t previous_state = LED_STATE_IDLE;

	uint32_t anim_tick = 0;
	/* Infinite loop */
	for (;;) {
//		current_state = DetermineRobotState();
//
//		/* Reset animacji przy zmianie stanu */
//		if (current_state != previous_state) {
//			anim_tick = 0;
//			WS2812_Clear();
//			WS2812_Set_Brightness(50);
//			previous_state = current_state;
//		}
//
//		WS2812_Clear();
//
//		switch (current_state) {
//
//		case LED_STATE_TURN_LEFT:
//		case LED_STATE_TURN_RIGHT: {
//			int sweep_progress = anim_tick % 40;
//
//			for (int i = 0; i < NUM_LEDS_PER_STRIP; i++) {
//				if (i <= sweep_progress) {
//					if (current_state == LED_STATE_TURN_LEFT) {
//						SetLeftLED(i, COLOR_ORANGE);
//					} else {
//						SetRightLED(i, COLOR_ORANGE);
//					}
//				}
//			}
//
//			break;
//		}
//
//		case LED_STATE_REVERSE: {
//			int blink_slow = (anim_tick / 15) % 2;
//
//			if (blink_slow) {
//				for (int i = 0; i < NUM_LEDS_PER_STRIP; i++) {
//					SetLeftLED(i, COLOR_RED);
//					SetRightLED(i, COLOR_RED);
//				}
//			}
//			break;
//		}
//
//		case LED_STATE_FORWARD: {
//			/* EFEKT: Subtelny biały/zielony "szpic" wskazujący ruch do przodu */
//			/* Przesuwająca się biała kropka od środka na zewnątrz */
//			int dot_pos = anim_tick % NUM_LEDS_PER_STRIP;
//
//			/* Tło lekko zielone */
//			for (int i = 0; i < NUM_LEDS_PER_STRIP; i++) {
//				SetLeftLED(i, 0, 30, 0);
//				SetRightLED(i, 0, 30, 0);
//			}
//
//			/* Jaśniejsza "kometa" */
//			SetLeftLED(dot_pos, COLOR_WHITE);
//			SetRightLED(dot_pos, COLOR_WHITE);
//			if (dot_pos > 0) {
//				SetLeftLED(dot_pos - 1, 100, 100, 100);
//				SetRightLED(dot_pos - 1, 100, 100, 100);
//			}
//			break;
//		}
//
//		case LED_STATE_ERROR: {
//			/* EFEKT: Szybkie, agresywne miganie na czerwono całością */
//			int blink_fast = (anim_tick / 5) % 2; // mignięcie co ~150ms
//
//			if (blink_fast) {
//				for (int i = 0; i < NUM_LEDS_PER_STRIP; i++) {
//					SetLeftLED(i, COLOR_RED);
//					SetRightLED(i, COLOR_RED);
//				}
//			}
//			break;
//		}
//
//		case LED_STATE_IDLE:
//		default: {
//			/* EFEKT: "Oddychanie" (Breathing) w kolorze cyjanowym (niebiesko-zielonym) */
//			/* Korzystamy z sinusa lub prostej fali trójkątnej na podstawie anim_tick */
//
//			int breathe = (anim_tick % 100); // 0 do 99
//			if (breathe > 50)
//				breathe = 100 - breathe; // Falowanie 0 -> 50 -> 0
//
//			/* Ustawiamy globalną jasność na czas IDLE, zmapowaną od 5 do 55 */
//			WS2812_Set_Brightness(5 + breathe);
//
//			for (int i = 0; i < MAX_LED; i++) {
//				WS2812_Set_LED(led_index, Red, Green, Blue)(i, COLOR_CYAN);
//				SetRightLED(i, COLOR_CYAN);
//			}
//			break;
//		}
//		}
//
//		/* Wysyłamy ramkę do LEDów */
//		WS2812_Send();
//		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); // Opcjonalny heartbeat na PCB
//
//		anim_tick++;
		osDelay(30);
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
		osDelay(1);
	}
	/* USER CODE END StartCalcBuzzer */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


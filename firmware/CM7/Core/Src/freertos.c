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

#include "GUI_Paint.h"
#include "OLED_1in5.h"
#include "fonts.h"
#include "shared_data.h"
#include <stdio.h>
#include "ili9341.h"
#include "ili9341_touch.h"
#include "float.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <stdbool.h>
#include <uxr/client/transport.h>

#include "lwip/inet.h"
#include "lwip/sockets.h"

#include <sensor_msgs/msg/imu.h>
#include <nav_msgs/msg/odometry.h>
#include <math.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <rosidl_runtime_c/string_functions.h>
#include "app_conf.h"
#include "shared_ipc.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define NETWORK_INITIALIZED 1U
#define WHEEL_DIAMETER_M 0.1f // 96mm
#define TRACK_WIDTH_M    0.274f
#define TICKS_PER_REV 3840.0f

#ifndef PI
#define PI 3.14159265358979323846
#endif

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

volatile uint8_t current_active_spi_device = 0;
volatile bool g_microros_connected = false;
volatile uint32_t g_oled_error_count = 0;
volatile uint32_t g_uros_error_count = 0;
volatile rcl_ret_t g_uros_last_error = RCL_RET_OK;
const char *volatile g_uros_error_stage = "none";
volatile uint32_t g_invalid_cmd_count = 0;
volatile uint32_t g_cmd_timeout_count = 0;
volatile uint32_t g_time_sync_error_count = 0;

static uint32_t last_cmd_tick;
static bool cmd_valid;
static geometry_msgs__msg__Twist recv_twist_msg;
static rcl_publisher_t pub_temp, pub_hum, pub_imu, pub_odom;
static std_msgs__msg__Float32 msg_temp, msg_hum;
static sensor_msgs__msg__Imu imu_msg;
static nav_msgs__msg__Odometry odom_msg;

static double wheel_x, wheel_y, wheel_yaw;

typedef struct {
	rclc_support_t support;
	rcl_node_t node;
	rcl_subscription_t subscriber;
	rclc_executor_t executor;
	bool support_ready, node_ready, temp_ready, hum_ready, imu_ready,
			odom_ready;
	bool subscriber_ready, executor_ready, imu_msg_ready, odom_msg_ready;
} RosEntities;
static RosEntities ros;

/* USER CODE END Variables */
/* Definitions for LwipUDP */
osThreadId_t LwipUDPHandle;
const osThreadAttr_t LwipUDP_attributes = { .name = "LwipUDP", .stack_size = 256
		* 4, .priority = (osPriority_t) osPriorityAboveNormal, };
/* Definitions for renderDisplays */
osThreadId_t renderDisplaysHandle;
const osThreadAttr_t renderDisplays_attributes = { .name = "renderDisplays",
		.stack_size = 512 * 4, .priority = (osPriority_t) osPriorityLow, };
/* Definitions for microROS */
osThreadId_t microROSHandle;
const osThreadAttr_t microROS_attributes = { .name = "microROS", .stack_size =
		2048 * 4, .priority = (osPriority_t) osPriorityHigh, };
/* Definitions for commandWatchdog */
osThreadId_t commandWatchdogHandle;
const osThreadAttr_t commandWatchdog_attributes = { .name = "commandWatchdog",
		.stack_size = 256 * 4, .priority = (osPriority_t) osPriorityHigh, };
/* Definitions for network_events */
osEventFlagsId_t network_eventsHandle;
const osEventFlagsAttr_t network_events_attributes =
		{ .name = "network_events" };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

bool cubemx_transport_open(struct uxrCustomTransport *transport);
bool cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(struct uxrCustomTransport *transport,
		const uint8_t *buf, size_t len, uint8_t *err);
size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf,
		size_t len, int timeout, uint8_t *err);
bool cubemx_transport_reset(void);

void* microros_allocate(size_t size, void *state);
void microros_deallocate(void *pointer, void *state);
void* microros_reallocate(void *pointer, size_t size, void *state);
void* microros_zero_allocate(size_t number_of_elements, size_t size_of_element,
		void *state);

static uint32_t ms_to_ticks(uint32_t ms) {
	uint64_t ticks = ((uint64_t) ms * osKernelGetTickFreq() + 999U) / 1000U;
	return ticks > 0U ? (uint32_t) ticks : 1U;
}

static void stop_commands_locked(void) {
	Shared_PublishCommand(0.0f, 0.0f);
	cmd_valid = false;
}

static void stop_commands(void) {
	taskENTER_CRITICAL();
	stop_commands_locked();
	taskEXIT_CRITICAL();
}

void subscription_callback(const void *msgin) {
	const geometry_msgs__msg__Twist *msg = msgin;
	if (msg == NULL || !isfinite(msg->linear.x) || !isfinite(msg->angular.z)
			|| fabs(msg->linear.x) > FLT_MAX || fabs(msg->angular.z) > FLT_MAX
			|| (APP_MAX_LINEAR_SPEED > 0.0
					&& fabs(msg->linear.x) > APP_MAX_LINEAR_SPEED)
			|| (APP_MAX_ANGULAR_SPEED > 0.0
					&& fabs(msg->angular.z) > APP_MAX_ANGULAR_SPEED)) {
		++g_invalid_cmd_count;
		stop_commands();
		return;
	}
	taskENTER_CRITICAL();
	if (g_microros_connected) {
		Shared_PublishCommand((float) msg->linear.x, (float) msg->angular.z);
		last_cmd_tick = osKernelGetTickCount();
		cmd_valid = true;
	}
	taskEXIT_CRITICAL();
}

void euler_to_quat(float yaw, double *qz, double *qw) {
	*qz = sin(yaw / 2.0);
	*qw = cos(yaw / 2.0);
}

static bool ros_ok(rcl_ret_t rc, const char *stage) {
	if (rc == RCL_RET_OK) {
		return true;
	}
	++g_uros_error_count;
	g_uros_last_error = rc;
	g_uros_error_stage = stage;
	rcl_reset_error();
	return false;
}

static void destroy_ros_entities(void) {
	g_microros_connected = false;
	stop_commands();
	rmw_context_t *context = NULL;
	if (ros.support.context.impl != NULL) {
		context = rcl_context_get_rmw_context(&ros.support.context);
	}
	if (context != NULL) {
		(void) rmw_uros_set_context_entity_destroy_session_timeout(context, 0);
	}
	if (ros.executor_ready) {
		(void) ros_ok(rclc_executor_fini(&ros.executor), "executor_fini");
	}
	if (ros.subscriber_ready) {
		(void) ros_ok(rcl_subscription_fini(&ros.subscriber, &ros.node),
				"subscription_fini");
	}
	if (ros.imu_ready)
		(void) ros_ok(rcl_publisher_fini(&pub_imu, &ros.node), "imu_fini");
	if (ros.odom_ready)
		(void) ros_ok(rcl_publisher_fini(&pub_odom, &ros.node), "odom_fini");
	if (ros.hum_ready) {
		(void) ros_ok(rcl_publisher_fini(&pub_hum, &ros.node), "hum_fini");
	}
	if (ros.temp_ready) {
		(void) ros_ok(rcl_publisher_fini(&pub_temp, &ros.node), "temp_fini");
	}
	if (ros.node_ready) {
		(void) ros_ok(rcl_node_fini(&ros.node), "node_fini");
	}
	if (ros.support_ready) {
		(void) ros_ok(rclc_support_fini(&ros.support), "support_fini");
	} else {
		/* rclc_support_init can fail after creating a context/clock. */
		if (rcl_clock_valid(&ros.support.clock)) {
			(void) ros_ok(rcl_clock_fini(&ros.support.clock), "clock_fini");
		}
		if (ros.support.context.impl != NULL) {
			if (rcl_context_is_valid(&ros.support.context)) {
				(void) ros_ok(rcl_shutdown(&ros.support.context), "shutdown");
			}
			(void) ros_ok(rcl_context_fini(&ros.support.context),
					"context_fini");
		}
	}
	if (ros.imu_msg_ready)
		sensor_msgs__msg__Imu__fini(&imu_msg);
	if (ros.odom_msg_ready)
		nav_msgs__msg__Odometry__fini(&odom_msg);
	if (!cubemx_transport_reset()) {
		(void) ros_ok(RCL_RET_ERROR, "transport_close");
	}
	memset(&ros, 0, sizeof(ros));
}

static bool configure_publisher(rcl_publisher_t *publisher) {
	rmw_publisher_t *handle = rcl_publisher_get_rmw_handle(publisher);
	if (handle == NULL) {
		return ros_ok(RCL_RET_ERROR, "publisher_handle");
	}
	return ros_ok(rmw_uros_set_publisher_session_timeout(handle,
	APP_RELIABLE_ACK_TIMEOUT_MS), "publisher_timeout");
}

static bool create_ros_entities(rcl_allocator_t *allocator) {
	memset(&ros, 0, sizeof(ros));
	ros.node = rcl_get_zero_initialized_node();
	ros.subscriber = rcl_get_zero_initialized_subscription();
	ros.executor = rclc_executor_get_zero_initialized_executor();
	pub_temp = rcl_get_zero_initialized_publisher();
	pub_hum = rcl_get_zero_initialized_publisher();
	pub_imu = rcl_get_zero_initialized_publisher();
	pub_odom = rcl_get_zero_initialized_publisher();

#define INIT_ENTITY(call, flag, stage) do { \
    if (!ros_ok((call), (stage))) { goto fail; } \
    ros.flag = true; \
} while (0)
	INIT_ENTITY(rclc_support_init(&ros.support, 0, NULL, allocator),
			support_ready, "support_init");
	INIT_ENTITY(
			rclc_node_init_default(&ros.node, "orion_node", "", &ros.support),
			node_ready, "node_init");
	INIT_ENTITY(
			rclc_publisher_init_default(&pub_temp, &ros.node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "temperature"),
			temp_ready, "temp_init");
	INIT_ENTITY(
			rclc_publisher_init_default(&pub_hum, &ros.node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "humidity"),
			hum_ready, "hum_init");
	INIT_ENTITY(
			rclc_publisher_init_default(&pub_imu,&ros.node,ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs,msg,Imu),"/imu/data"),
			imu_ready, "imu_init");
	INIT_ENTITY(
			rclc_publisher_init_default(&pub_odom,&ros.node,ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs,msg,Odometry),"/wheel/odometry"),
			odom_ready, "odom_init");
	INIT_ENTITY(
			rclc_subscription_init_default(&ros.subscriber, &ros.node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"),
			subscriber_ready, "subscription_init");
	INIT_ENTITY(
			rclc_executor_init(&ros.executor, &ros.support.context, 1,
					allocator), executor_ready, "executor_init");
#undef INIT_ENTITY
	if (!ros_ok(
			rclc_executor_add_subscription(&ros.executor, &ros.subscriber,
					&recv_twist_msg, subscription_callback, ON_NEW_DATA),
			"add_subscription")) {
		goto fail;
	}
	if (!configure_publisher(&pub_temp) || !configure_publisher(&pub_hum)
			|| !configure_publisher(&pub_imu)
			|| !configure_publisher(&pub_odom)) {
		goto fail;
	}

	if (!sensor_msgs__msg__Imu__init(&imu_msg))
		goto memory_fail;
	ros.imu_msg_ready = true;
	if (!nav_msgs__msg__Odometry__init(&odom_msg))
		goto memory_fail;
	ros.odom_msg_ready = true;
	if (!rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link")
			|| !rosidl_runtime_c__String__assign(&odom_msg.header.frame_id,
					"odom")
			|| !rosidl_runtime_c__String__assign(&odom_msg.child_frame_id,
					"base_footprint"))
		goto memory_fail;
	imu_msg.orientation_covariance[0] = -1.0;
	/* Initial tuning only; use wheel twist, not its integrated pose, in EKF. */
	for (int i = 0; i < 6; i++) {
		odom_msg.pose.covariance[7 * i] = 1e6;
		odom_msg.twist.covariance[7 * i] = 1e6;
	}
	odom_msg.twist.covariance[0] = 0.01;
	odom_msg.twist.covariance[35] = 0.04;
	return true;
	memory_fail: (void) ros_ok(RCL_RET_BAD_ALLOC, "sensor_message_alloc");
	fail: destroy_ros_entities();
	return false;
}

/* USER CODE END FunctionPrototypes */

void StartLwipUDP(void *argument);
void StartRenderDisplays(void *argument);
void StartMicroROS(void *argument);
void StartCommandWatchdog(void *argument);

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

	/* creation of renderDisplays */
	renderDisplaysHandle = osThreadNew(StartRenderDisplays, NULL,
			&renderDisplays_attributes);

	/* creation of microROS */
	microROSHandle = osThreadNew(StartMicroROS, NULL, &microROS_attributes);

	/* creation of commandWatchdog */
	commandWatchdogHandle = osThreadNew(StartCommandWatchdog, NULL,
			&commandWatchdog_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Create the event(s) */
	/* creation of network_events */
	network_eventsHandle = osEventFlagsNew(&network_events_attributes);

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
void StartLwipUDP(void *argument) {
	/* init code for LWIP */
	MX_LWIP_Init();
	/* USER CODE BEGIN StartLwipUDP */
	(void) argument;
	/* Stack initialized does not mean DHCP/link/agent are ready. Agent ping
	 * in StartMicroROS is the actual readiness check. */
	if ((osEventFlagsSet(network_eventsHandle, NETWORK_INITIALIZED)
			& osFlagsError) != 0U) {
		Error_Handler();
	}
	for (;;) {
		osDelay(ms_to_ticks(1000U));
	}
	/* USER CODE END StartLwipUDP */
}

/* USER CODE BEGIN Header_StartRenderDisplays */
/**
 * @brief Function implementing the renderDisplays thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartRenderDisplays */
void StartRenderDisplays(void *argument) {
	/* USER CODE BEGIN StartRenderDisplays */
	static UBYTE BlackImage[8192];
	char text_buffer[30];
	SharedEncoders_t display_encoders = { 0 };
	SharedImu_t display_imu = { 0 };

	//oled_sem = osSemaphoreNew(1, 0, NULL);

	System_Init();
	OLED_1in5_Init();
	osDelay(100);
	OLED_1in5_Clear();

	Paint_NewImage(BlackImage, OLED_1in5_WIDTH, OLED_1in5_HEIGHT, 0, BLACK);
	Paint_SetScale(16);

	ILI9341_Unselect();
	//ILI9341_TouchUnselect();

	osDelay(150);
	ILI9341_Init();
	ILI9341_FillScreen(ILI9341_BLACK);
	osDelay(1000);

	/* Infinite loop */
	for (;;) {
		Paint_SelectImage(BlackImage);
		Paint_Clear(BLACK);

		if (SHARED_DATA->current_error != STATUS_OK) {

			Paint_DrawString_EN(10, 10, "SYSTEM FAULT!", &Font12, WHITE, BLACK);

			switch (SHARED_DATA->current_error) {
			case ERR_BME_INIT_FAIL:
				Paint_DrawString_EN(10, 35, "BME680 Error", &Font12, WHITE,
				BLACK);
				Paint_DrawString_EN(10, 50, "Check I2C cables", &Font12, WHITE,
				BLACK);
				break;
			case ERR_VL6180X_INIT_FAIL:
				Paint_DrawString_EN(10, 35, "Laser Sensor Err", &Font12, WHITE,
				BLACK);
				break;
			default:
				Paint_DrawString_EN(10, 35, "Unknown Error", &Font12, WHITE,
				BLACK);
				break;
			}

		} else {
			(void) Shared_TryReadEncoders(&display_encoders);
			float temp = SHARED_DATA->m4_temperature;
			float dist = SHARED_DATA->m4_distances[0];
			float dist2 = SHARED_DATA->m4_distances[1];
			float dist3 = SHARED_DATA->m4_distances[2];

			int hum = (int) SHARED_DATA->m4_humidity;
			int pre = (int) (SHARED_DATA->m4_pressure / 100.0f);

			snprintf(text_buffer, sizeof(text_buffer), "t: %.1f h: %d", temp,
					hum);
			Paint_DrawString_EN(10, 20, text_buffer, &Font12, WHITE, BLACK);

			snprintf(text_buffer, sizeof(text_buffer), "L:%7.1f RPM",
					display_encoders.left_rpm);
			Paint_DrawString_EN(10, 40, text_buffer, &Font12, WHITE, BLACK);

			snprintf(text_buffer, sizeof(text_buffer), "R:%7.1f RPM",
					display_encoders.right_rpm);
			Paint_DrawString_EN(10, 100, text_buffer, &Font12, WHITE, BLACK);

			snprintf(text_buffer, sizeof(text_buffer), "%.1f cm", dist);
			Paint_DrawString_EN(10, 60, text_buffer, &Font12, WHITE, BLACK);

//			snprintf(text_buffer, sizeof(text_buffer), "%.1f", dist3);
//			Paint_DrawString_EN(75, 40, text_buffer, &Font12, WHITE, BLACK);

			if (g_microros_connected) {
				snprintf(text_buffer, sizeof(text_buffer), "uROS: Connected");
			} else {
				snprintf(text_buffer, sizeof(text_buffer), "uROS: Connect...");
			}
			Paint_DrawString_EN(10, 80, text_buffer, &Font12, WHITE, BLACK);

		}
		OLED_1in5_Display(BlackImage);
		// ODKOMENTOWAC CZY NIE osSemaphoreAcquire(oled_sem, osWaitForever);

		(void) Shared_TryReadImu(&display_imu);

		ILI9341_WriteString(10, 10, "Witaj na STM32!", Font_11x18,
		ILI9341_GREEN, ILI9341_BLACK);
		snprintf(text_buffer, sizeof(text_buffer), "Z acc: %6.2f m/s^2",
				display_imu.accel[2]);

		ILI9341_WriteString(10, 100, text_buffer, Font_11x18, ILI9341_WHITE,
		ILI9341_BLACK);

		snprintf(text_buffer, sizeof(text_buffer), "Z omega: %7.3f rad/s",
				display_imu.gyro[2]);

		ILI9341_WriteString(10, 130, text_buffer, Font_11x18,
		ILI9341_WHITE, ILI9341_BLACK);

		ILI9341_FillRectangle(10, 40, 100, 50, ILI9341_BLACK);
		osDelay(100);
	}
	/* USER CODE END StartRenderDisplays */
}

/* USER CODE BEGIN Header_StartMicroROS */
/**
 * @brief Function implementing the microROS thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartMicroROS */
void StartMicroROS(void *argument) {
	/* USER CODE BEGIN StartMicroROS */
	(void) argument;
	g_microros_connected = false;
	stop_commands();
	uint32_t events = osEventFlagsWait(network_eventsHandle,
	NETWORK_INITIALIZED,
	osFlagsWaitAll | osFlagsNoClear, osWaitForever);
	if ((events & osFlagsError) != 0U) {
		Error_Handler();
		return;
	}

	if (!ros_ok(
			rmw_uros_set_custom_transport(false, NULL, cubemx_transport_open,
					cubemx_transport_close, cubemx_transport_write,
					cubemx_transport_read), "transport_init")) {
		Error_Handler();
		return;
	}
	rcl_allocator_t allocator = rcutils_get_zero_initialized_allocator();
	allocator.allocate = microros_allocate;
	allocator.deallocate = microros_deallocate;
	allocator.reallocate = microros_reallocate;
	allocator.zero_allocate = microros_zero_allocate;
	if (!rcutils_set_default_allocator(&allocator)) {
		(void) ros_ok(RCL_RET_BAD_ALLOC, "allocator_init");
		Error_Handler();
		return;
	}

	for (;;) {
		if (rmw_uros_ping_agent(APP_AGENT_PING_TIMEOUT_MS, 1) != RMW_RET_OK) {
			(void) cubemx_transport_reset();
			osDelay(ms_to_ticks(APP_AGENT_RETRY_MS));
			continue;
		}
		if (!create_ros_entities(&allocator)) {
			osDelay(ms_to_ticks(APP_AGENT_RETRY_MS));
			continue;
		}
		bool time_ready = rmw_uros_sync_session(APP_TIME_SYNC_TIMEOUT_MS)
				== RMW_RET_OK && rmw_uros_epoch_synchronized();
		if (!time_ready) {
			++g_time_sync_error_count;
		}
		stop_commands();
		g_microros_connected = true;
		uint32_t temp_tick = osKernelGetTickCount();
		uint32_t hum_tick = temp_tick;
		uint32_t imu_tick = temp_tick, odom_tick = temp_tick;
		uint32_t ping_tick = temp_tick, sync_tick = temp_tick;
		uint32_t imu_seq = 0, enc_seq = 0, enc_tick = 0;
		double prev_v = 0, prev_w = 0;
		bool healthy = true;

		while (healthy) {
			rcl_ret_t rc = rclc_executor_spin_some(&ros.executor,
			RCL_MS_TO_NS(1));
			if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT) {
				(void) ros_ok(rc, "executor_spin");
				break;
			}
			uint32_t now = osKernelGetTickCount();
			if ((uint32_t) (now - ping_tick)
					>= ms_to_ticks(APP_AGENT_PING_PERIOD_MS)) {
				if (rmw_uros_ping_agent(APP_AGENT_PING_TIMEOUT_MS,
						1) != RMW_RET_OK) {
					break;
				}
				ping_tick = osKernelGetTickCount();
			}
			if ((uint32_t) (now - temp_tick)
					>= ms_to_ticks(APP_TEMPERATURE_PERIOD_MS)) {
				msg_temp.data = SHARED_DATA->m4_temperature;
				if (isfinite(msg_temp.data)) {
					healthy = ros_ok(rcl_publish(&pub_temp, &msg_temp, NULL),
							"temperature_publish");
					if (healthy) {
						HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1);
					}
				}
				temp_tick = now;
			}
			if (!healthy) {
				break;
			}
			if ((uint32_t) (now - hum_tick)
					>= ms_to_ticks(APP_HUMIDITY_PERIOD_MS)) {
				msg_hum.data = 10.0f;
				if (isfinite(msg_hum.data)) {
					healthy = ros_ok(rcl_publish(&pub_hum, &msg_hum, NULL),
							"humidity_publish");
				}
				hum_tick = now;
			}
			if (!healthy) {
				break;
			}
			if ((uint32_t) (now - imu_tick) >= ms_to_ticks(APP_IMU_PERIOD_MS)) {
				imu_tick = now;
				if (time_ready && rmw_uros_epoch_synchronized()) {
					SharedImu_t im;
					if (Shared_TryReadImu(
							&im) && im.sequence != imu_seq &&
							isfinite(im.accel[0]) && isfinite(im.accel[1]) && isfinite(im.accel[2]) &&
							isfinite(im.gyro[0]) && isfinite(im.gyro[1]) && isfinite(im.gyro[2])) {
						int64_t ns = rmw_uros_epoch_nanos();
						if (ns > 0 && ns / INT64_C(1000000000) <= INT32_MAX) {
							imu_msg.header.stamp.sec = (int32_t) (ns
									/ INT64_C(1000000000));
							imu_msg.header.stamp.nanosec = (uint32_t) (ns
									% INT64_C(1000000000));
							imu_msg.angular_velocity.x = im.gyro[0];
							imu_msg.angular_velocity.y = im.gyro[1];
							imu_msg.angular_velocity.z = im.gyro[2];
							imu_msg.linear_acceleration.x = im.accel[0];
							imu_msg.linear_acceleration.y = im.accel[1];
							imu_msg.linear_acceleration.z = im.accel[2];
							healthy = ros_ok(
									rcl_publish(&pub_imu, &imu_msg, NULL),
									"imu_publish");
							if (healthy)
								imu_seq = im.sequence;
						}
					}
				}
			}
			if (!healthy) {
				break;
			}
			if ((uint32_t) (now - odom_tick)
					>= ms_to_ticks(APP_ODOM_PERIOD_MS)) {
				odom_tick = now;
				if (time_ready && rmw_uros_epoch_synchronized()) {
					SharedEncoders_t en;
					if (Shared_TryReadEncoders(&en) && en.sequence != enc_seq &&
					isfinite(en.left_rpm) && isfinite(en.right_rpm)) {
						int64_t ns = rmw_uros_epoch_nanos();
						if (ns > 0 && ns / INT64_C(1000000000) <= INT32_MAX) {
							const double k = PI * WHEEL_DIAMETER_M;
							const double rpm_to_mps = k / 60.0;
							double l = en.left_rpm * rpm_to_mps;
							double r = en.right_rpm * rpm_to_mps;
							double v = (r + l) * 0.5;
							double w = (r - l) / TRACK_WIDTH_M;
							double dt = (double) (uint32_t) (now - enc_tick)
									/ osKernelGetTickFreq();
							if (enc_seq && dt > 0 && dt <= 0.25) {
								double ds = 0.5 * (prev_v + v) * dt;
								double da = 0.5 * (prev_w + w) * dt;
								wheel_x += ds * cos(wheel_yaw + da * 0.5);
								wheel_y += ds * sin(wheel_yaw + da * 0.5);
								wheel_yaw = remainder(wheel_yaw + da, 2.0 * PI);
							}
							prev_v = v;
							prev_w = w;
							enc_seq = en.sequence;
							enc_tick = now;
							odom_msg.header.stamp.sec = (int32_t) (ns
									/ INT64_C(1000000000));
							odom_msg.header.stamp.nanosec = (uint32_t) (ns
									% INT64_C(1000000000));
							odom_msg.pose.pose.position.x = wheel_x;
							odom_msg.pose.pose.position.y = wheel_y;
							euler_to_quat(wheel_yaw,
									&odom_msg.pose.pose.orientation.z,
									&odom_msg.pose.pose.orientation.w);
							odom_msg.twist.twist.linear.x = v;
							odom_msg.twist.twist.angular.z = w;
							healthy = ros_ok(
									rcl_publish(&pub_odom, &odom_msg, NULL),
									"odom_publish");
						}
					}
				}
			}
			if (!healthy) {
				break;
			}
			if ((uint32_t) (now - sync_tick)
					>= ms_to_ticks(APP_TIME_SYNC_PERIOD_MS)) {
				time_ready = rmw_uros_sync_session(APP_TIME_SYNC_TIMEOUT_MS)
						== RMW_RET_OK && rmw_uros_epoch_synchronized();
				if (!time_ready) {
					++g_time_sync_error_count;
				}
				sync_tick = osKernelGetTickCount();
			}
			osDelay(ms_to_ticks(1U));
		}
		destroy_ros_entities();
		osDelay(ms_to_ticks(APP_AGENT_RETRY_MS));
	}
	/* USER CODE END StartMicroROS */
}

/* USER CODE BEGIN Header_StartCommandWatchdog */
/**
 * @brief Function implementing the commandWatchdog thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartCommandWatchdog */
void StartCommandWatchdog(void *argument) {
	/* USER CODE BEGIN StartCommandWatchdog */
	uint32_t deadline = osKernelGetTickCount();
	/* Infinite loop */
	for (;;) {
		taskENTER_CRITICAL();
		if (!g_microros_connected) {
			stop_commands_locked();
		} else if (cmd_valid
				&& (uint32_t) (osKernelGetTickCount() - last_cmd_tick)
						>= ms_to_ticks(APP_CMD_TIMEOUT_MS)) {
			stop_commands_locked();
			++g_cmd_timeout_count;
		}
		taskEXIT_CRITICAL();
		deadline += ms_to_ticks(APP_CMD_WATCHDOG_MS);
		if (osDelayUntil(deadline) != osOK) {
			deadline = osKernelGetTickCount();
			osDelay(ms_to_ticks(APP_CMD_WATCHDOG_MS));
		}
	}
	/* USER CODE END StartCommandWatchdog */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


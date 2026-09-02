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
#include "udp_comm.h"
#include <stdio.h>

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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define UDP_RX_PORT 5556
#define UDP_RX_TIMEOUT_MS 50

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

osSemaphoreId_t oled_sem;

volatile float g_udp_rx_value = 0.0f;
std_msgs__msg__Int32 recv_msg;
std_msgs__msg__Float32 recv_linear_speed;
geometry_msgs__msg__Twist recv_twist_msg;

rcl_publisher_t pub_temp;
rcl_publisher_t pub_hum;
std_msgs__msg__Float32 msg_temp;
std_msgs__msg__Float32 msg_hum;

volatile bool g_microros_connected = false;

/* USER CODE END Variables */
/* Definitions for LwipUDPp */
osThreadId_t LwipUDPpHandle;
const osThreadAttr_t LwipUDPp_attributes = { .name = "LwipUDPp", .stack_size =
		256 * 4, .priority = (osPriority_t) osPriorityHigh, };
/* Definitions for renderDisplay */
osThreadId_t renderDisplayHandle;
const osThreadAttr_t renderDisplay_attributes = { .name = "renderDisplay",
		.stack_size = 512 * 4, .priority = (osPriority_t) osPriorityLow, };
/* Definitions for microROS */
osThreadId_t microROSHandle;
const osThreadAttr_t microROS_attributes = { .name = "microROS", .stack_size =
		2048 * 4, .priority = (osPriority_t) osPriorityAboveNormal, };
/* Definitions for UDPReceiver */
osThreadId_t UDPReceiverHandle;
const osThreadAttr_t UDPReceiver_attributes = { .name = "UDPReceiver",
		.stack_size = 512 * 4, .priority = (osPriority_t) osPriorityHigh, };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

bool cubemx_transport_open(struct uxrCustomTransport *transport);
bool cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(struct uxrCustomTransport *transport,
		const uint8_t *buf, size_t len, uint8_t *err);
size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf,
		size_t len, int timeout, uint8_t *err);

void* microros_allocate(size_t size, void *state);
void microros_deallocate(void *pointer, void *state);
void* microros_reallocate(void *pointer, size_t size, void *state);
void* microros_zero_allocate(size_t number_of_elements, size_t size_of_element,
		void *state);

//void subscription_callback(const void *msgin) {
//	//const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32*) msgin;
//	const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32*) msgin;
//
//	//(uint8_t) msg->data;
//	SHARED_DATA->m7_linear_speed = msg->data;
//}

void subscription_callback(const void *msgin) {
	const geometry_msgs__msg__Twist *msg =
			(const geometry_msgs__msg__Twist*) msgin;

	SHARED_DATA->m7_linear_speed = msg->linear.x;
	SHARED_DATA->m7_angular_speed = msg->angular.z;
}

void temp_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
	(void) last_call_time;
	if (timer != NULL) {
		msg_temp.data = SHARED_DATA->m4_temperature;
		rcl_publish(&pub_temp, &msg_temp, NULL);
	}
}

void hum_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
	(void) last_call_time;
	if (timer != NULL) {
		msg_hum.data = SHARED_DATA->m4_humidity;
		rcl_publish(&pub_hum, &msg_hum, NULL);
	}
}

/* USER CODE END FunctionPrototypes */

void StartLwipUDP(void *argument);
void StartRenderDisplay(void *argument);
void StartMicroROS(void *argument);
void StartUDPReceiver(void *argument);

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
	/* creation of LwipUDPp */
	LwipUDPpHandle = osThreadNew(StartLwipUDP, NULL, &LwipUDPp_attributes);

	/* creation of renderDisplay */
	renderDisplayHandle = osThreadNew(StartRenderDisplay, NULL,
			&renderDisplay_attributes);

	/* creation of microROS */
	microROSHandle = osThreadNew(StartMicroROS, NULL, &microROS_attributes);

	/* creation of UDPReceiver */
	UDPReceiverHandle = osThreadNew(StartUDPReceiver, NULL,
			&UDPReceiver_attributes);

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
void StartLwipUDP(void *argument) {
	/* init code for LWIP */
	MX_LWIP_Init();
	/* USER CODE BEGIN StartLwipUDP */
	osDelay(500);
	// UDP_Server_Task();
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
void StartRenderDisplay(void *argument) {
	/* USER CODE BEGIN StartRenderDisplay */
	static UBYTE BlackImage[8192];

	oled_sem = osSemaphoreNew(1, 0, NULL);

	System_Init();
	OLED_1in5_Init();
	osDelay(100);
	OLED_1in5_Clear();

	Paint_NewImage(BlackImage, OLED_1in5_WIDTH, OLED_1in5_HEIGHT, 0, BLACK);
	Paint_SetScale(16);
	Paint_SelectImage(BlackImage);
	//Paint_Clear(BLACK);

	char text_buffer[30];

	/* Infinite loop */
	for (;;) {
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
			float rpm = SHARED_DATA->m7_linear_speed;
			float ang = SHARED_DATA->m7_angular_speed;
			float temp = SHARED_DATA->m4_temperature;
			float dist = SHARED_DATA->m4_distances[0];

			int hum = (int) SHARED_DATA->m4_humidity;
			int pre = (int) (SHARED_DATA->m4_pressure / 100.0f);

			snprintf(text_buffer, sizeof(text_buffer), "L: %.1f A: %.1f", rpm,
					ang);
			Paint_DrawString_EN(10, 20, text_buffer, &Font12, WHITE, BLACK);

			snprintf(text_buffer, sizeof(text_buffer), "%.1f",
			SHARED_DATA->m4_motor_left_rpm);
			Paint_DrawString_EN(10, 40, text_buffer, &Font12, WHITE, BLACK);

			snprintf(text_buffer, sizeof(text_buffer), "%.1f cm", dist);
			Paint_DrawString_EN(10, 60, text_buffer, &Font12, WHITE, BLACK);

			snprintf(text_buffer, sizeof(text_buffer), "%.1f",
			SHARED_DATA->m4_motor_right_rpm);
			Paint_DrawString_EN(75, 40, text_buffer, &Font12, WHITE, BLACK);

			if (g_microros_connected) {
				snprintf(text_buffer, sizeof(text_buffer), "uROS: Connected");
			} else {
				snprintf(text_buffer, sizeof(text_buffer), "uROS: Connect...");
			}
			Paint_DrawString_EN(10, 80, text_buffer, &Font12, WHITE, BLACK);
		}
		OLED_1in5_Display(BlackImage);
		//osSemaphoreAcquire(oled_sem, osWaitForever);

		osDelay(500);
	}
	/* USER CODE END StartRenderDisplay */
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
	osDelay(1500);

	rmw_uros_set_custom_transport(false, (void*) NULL, cubemx_transport_open,
			cubemx_transport_close, cubemx_transport_write,
			cubemx_transport_read);

	rcl_allocator_t freeRTOS_allocator =
			rcutils_get_zero_initialized_allocator();
	freeRTOS_allocator.allocate = microros_allocate;
	freeRTOS_allocator.deallocate = microros_deallocate;
	freeRTOS_allocator.reallocate = microros_reallocate;
	freeRTOS_allocator.zero_allocate = microros_zero_allocate;

	if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
		while (1) {
			osDelay(100);
		}
	}

	rclc_support_t support;
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rcl_ret_t rc;

	rc = rclc_support_init(&support, 0, NULL, &allocator);
	if (rc != RCL_RET_OK) {
		while (1) {
			osDelay(1000);
			rc = rclc_support_init(&support, 0, NULL, &allocator);
			if (rc == RCL_RET_OK)
				break;
		}
	}
	g_microros_connected = true;

	rcl_node_t node;
	rc = rclc_node_init_default(&node, "orion_node", "", &support);

	// 1. Inicjalizacja Publisherów
	rclc_publisher_init_default(&pub_temp, &node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "temperature");
	rclc_publisher_init_default(&pub_hum, &node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "humidity");

	// 2. Inicjalizacja Subskrypcji
	rcl_subscription_t subscriber;
	rc = rclc_subscription_init_default(&subscriber, &node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");

	// 3. Executor tylko dla 1 elementu (subskrypcji) - ZNACZNIE odciąża pamięć sterty
	rclc_executor_t executor;
	executor = rclc_executor_get_zero_initialized_executor();
	rc = rclc_executor_init(&executor, &support.context, 1, &allocator);
	if (rc != RCL_RET_OK) {
		// Obsłuż błąd (np. w nieskończonej pętli z miganiem diody), brak sterty FreeRTOS!
		while (1) {
			osDelay(100);
		}
	}

	rc = rclc_executor_add_subscription(&executor, &subscriber, &recv_twist_msg,
			&subscription_callback, ON_NEW_DATA);

	// Zmienne do trzymania czasu
	uint32_t last_temp_time = osKernelGetTickCount();
	uint32_t last_hum_time = osKernelGetTickCount();
	uint32_t last_ping_time = osKernelGetTickCount();
	/* Infinite loop */
	for (;;) {
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

		uint32_t now = osKernelGetTickCount();

		// Publikacja Temperatury co 1000 ms
		if (now - last_temp_time >= 1000) {
			msg_temp.data = SHARED_DATA->m4_temperature;
			rcl_publish(&pub_temp, &msg_temp, NULL);
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_14);

			last_temp_time = now;
		}

		// Publikacja Wilgotności co 2000 ms
		if (now - last_hum_time >= 2000) {
			msg_hum.data = SHARED_DATA->m4_humidity;
			rcl_publish(&pub_hum, &msg_hum, NULL);
			last_hum_time = now;
		}

		if (now - last_ping_time >= 3000) {
			// Pinguj agenta (timeout = 100ms, próby = 1)
			if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
				g_microros_connected = true;
			} else {
				g_microros_connected = false;
			}
			last_ping_time = now;
		}

		osDelay(10);
	}
	/* USER CODE END StartMicroROS */
}

/* USER CODE BEGIN Header_StartUDPReceiver */
/**
 * @brief Function implementing the UDPReceiver thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartUDPReceiver */
void StartUDPReceiver(void *argument) {
	/* USER CODE BEGIN StartUDPReceiver */

	//TEN TASK BĘDZIE USUNIĘTY
	osDelay(1500);

	int sock = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0) {
		osThreadTerminate(NULL);
		return;
	}

	struct timeval timeout =
			{ .tv_sec = 0, .tv_usec = UDP_RX_TIMEOUT_MS * 1000, };
	lwip_setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

	struct sockaddr_in local_addr = { .sin_family = AF_INET, .sin_port = htons(
			UDP_RX_PORT), .sin_addr.s_addr = htonl(INADDR_ANY), };

	if (lwip_bind(sock, (struct sockaddr*) &local_addr, sizeof(local_addr))
			< 0) {
		lwip_close(sock);
		osThreadTerminate(NULL);
		return;
	}

	uint8_t rx_buf = 0.0f;
	/* Infinite loop */
	for (;;) {
		int bytes = lwip_recvfrom(sock, &rx_buf, sizeof(rx_buf), 0, NULL, NULL);

		if (bytes == sizeof(uint8_t)) {
			g_udp_rx_value = rx_buf;
			SHARED_DATA->m7_angular_speed = g_udp_rx_value;
		}
		osDelay(1);
	}
	/* USER CODE END StartUDPReceiver */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


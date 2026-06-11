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

#include <stdbool.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>

#include "lwip/sockets.h"
#include "lwip/inet.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define UDP_RX_PORT      5556
#define UDP_RX_TIMEOUT_MS 50

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

volatile float g_udp_rx_value = 0.0f;
std_msgs__msg__Int32 recv_msg;

/* USER CODE END Variables */
/* Definitions for LwipUDP */
osThreadId_t LwipUDPHandle;
const osThreadAttr_t LwipUDP_attributes = { .name = "LwipUDP", .stack_size = 256
		* 4, .priority = (osPriority_t) osPriorityHigh, };
/* Definitions for renderDisplay */
osThreadId_t renderDisplayHandle;
const osThreadAttr_t renderDisplay_attributes = { .name = "renderDisplay",
		.stack_size = 512 * 4, .priority = (osPriority_t) osPriorityLow, };
/* Definitions for microROS */
osThreadId_t microROSHandle;
const osThreadAttr_t microROS_attributes = { .name = "microROS", .stack_size =
		2560 * 4, .priority = (osPriority_t) osPriorityAboveNormal, };
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

void subscription_callback(const void *msgin) {
	// Rzutowanie odebranych surowych danych na nasz typ Int32
	const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32*) msgin;

	// TUTAJ ROBISZ CO CHCESZ Z ODEBRANĄ DANA!
	// Np. zapisujesz do struktury SHARED_DATA:
	// SHARED_DATA->target_speed = msg->data;

	SHARED_DATA->m7_angle = (uint8_t) msg->data;
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
	/* creation of LwipUDP */
	LwipUDPHandle = osThreadNew(StartLwipUDP, NULL, &LwipUDP_attributes);

	/* creation of renderDisplay */
	renderDisplayHandle = osThreadNew(StartRenderDisplay, NULL,
			&renderDisplay_attributes);

	/* creation of microROS */
	microROSHandle = osThreadNew(StartMicroROS, NULL, &microROS_attributes);

	/* creation of UDPReceiver */
//	UDPReceiverHandle = osThreadNew(StartUDPReceiver, NULL,
//			&UDPReceiver_attributes);
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
	osDelay(3000);
	//UDP_Server_Task();
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
	SHARED_DATA->m7_setpoint = 0.0f;

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
		uint8_t distance = SHARED_DATA->m7_angle;
		snprintf(text_buffer, sizeof(text_buffer), "Distance: %-3d cm   ",
				distance);
		Paint_DrawString_EN(10, 60, text_buffer, &Font12, WHITE,
		BLACK);
		OLED_1in5_Display(BlackImage);

		osDelay(100);
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
	osDelay(5000);

	// 2. Podpinamy transport UDP
	rmw_uros_set_custom_transport(
	false, (void*) NULL, cubemx_transport_open, cubemx_transport_close,
			cubemx_transport_write, cubemx_transport_read);

	// 3. Inicjalizacja alokatorów pamięci FreeRTOS
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

	// 4. Inicjalizacja wsparcia ROS 2
	rclc_support_t support;
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rcl_ret_t rc;

	rc = rclc_support_init(&support, 0, NULL, &allocator);
	if (rc != RCL_RET_OK) {
		// Zamiast blokować procesor na zawsze, próbujemy co sekundę!
		while (1) {
			osDelay(1000);
			rc = rclc_support_init(&support, 0, NULL, &allocator);
			if (rc == RCL_RET_OK)
				break;
		}
	}

	// 5. Inicjalizacja Node'a i Publishera
	rcl_node_t node;
	rc = rclc_node_init_default(&node, "orion_node", "", &support);

	rcl_publisher_t publisher;
	rc = rclc_publisher_init_default(&publisher, &node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
			"orion_temperature");

	rcl_subscription_t subscriber;
	rc = rclc_subscription_init_default(&subscriber, &node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "orion_setpoint" // Nazwa topiku, na którym STM32 będzie nasłuchiwać
			);

	// 8. Inicjalizacja Egzekutora (Kierownika zadań)
	rclc_executor_t executor;
	executor = rclc_executor_get_zero_initialized_executor();

	// Inicjujemy egzekutora z 1 uchwytem (bo mamy 1 subskrybenta)
	rc = rclc_executor_init(&executor, &support.context, 1, &allocator);

	// Podpinamy naszego subskrybenta, zmienną buforową i funkcję Callback do egzekutora
	rc = rclc_executor_add_subscription(&executor, &subscriber, &recv_msg,
			&subscription_callback, ON_NEW_DATA);

	std_msgs__msg__Float32 msg;
	msg.data = 0.0f;
	/* Infinite loop */
	for (;;) {
		msg.data = 20.5f;
		rc = rcl_publish(&publisher, &msg, NULL);

		// --- ODBIERANIE (Sprawdzanie przychodzących danych) ---
		// rclc_executor_spin_some daje egzekutorowi 100 milisekund na przetworzenie
		// wszystkiego, co wpadło przez UDP. Jeśli coś przyszło - wywoła Callback!
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

		// Krótka pauza, aby oddać czas procesora (FreeRTOS) innym zadaniom
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
			SHARED_DATA->m7_angle = g_udp_rx_value;
		}
		osDelay(1);
	}
	/* USER CODE END StartUDPReceiver */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


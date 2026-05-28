/*
 * udp_comm.c
 *
 * Created on: 18 maj 2026
 * Author: Oleg
 */

#include "udp_comm.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "FreeRTOS.h"
#include "task.h"
#include "shared_data.h"

#define STM_PORT 5001

void UDP_Server_Task(void) {
	int sock;
	struct sockaddr_in stm_addr, laptop_addr;
	socklen_t laptop_addr_len;

	uint8_t rx_buffer[32];
	UdpTxPacket_t tx_data = { 0 };

	while (1) {
		sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
		if (sock < 0) {
			osDelay(1000);
			continue;
		}

		struct timeval timeout;
		timeout.tv_sec = 1;
		timeout.tv_usec = 0;
		setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

		stm_addr.sin_family = AF_INET;
		stm_addr.sin_port = htons(STM_PORT);
		stm_addr.sin_addr.s_addr = INADDR_ANY;

		if (bind(sock, (struct sockaddr *)&stm_addr, sizeof(stm_addr)) < 0) {
			close(sock);
			osDelay(1000);
			continue;
		}

		// Main loop for UDP
		while (1) {
			laptop_addr_len = sizeof(laptop_addr);

			int rx_len = recvfrom(sock, rx_buffer, sizeof(rx_buffer), 0,
					(struct sockaddr* )&laptop_addr, &laptop_addr_len);

			if (rx_len > 0) {
				if (rx_len == sizeof(UdpRxPacket_t)) {
					UdpRxPacket_t *rx_data = (UdpRxPacket_t*) rx_buffer;

					SHARED_DATA->m7_setpoint = rx_data->setpoint;
					SHARED_DATA->m7_angle = rx_data->angle;

					tx_data.current_speed = SHARED_DATA->m4_current_speed;
					tx_data.temperature = 20.0f;

					sendto(sock, &tx_data, sizeof(UdpTxPacket_t), 0,
							(struct sockaddr* )&laptop_addr, laptop_addr_len);
				}
			} else {
				if (errno == EAGAIN || errno == EWOULDBLOCK || errno == 0) {
					// SOFT ERROR e.g. timeout
					continue;
				} else {
					// HARD ERROR e.g. disconnect
					break;
				}
			}
		}
		close(sock);
	}
}

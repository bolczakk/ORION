/*
 * udp_comm.h
 *
 *  Created on: 18 maj 2026
 *      Author: Oleg
 */

#ifndef INC_UDP_COMM_H_
#define INC_UDP_COMM_H_

#include <stdint.h>

typedef struct __attribute__((packed)) {
	float setpoint;
	float angle;
	float extra_param;
} UdpRxPacket_t;

typedef struct __attribute__((packed)) {
	float current_speed;
	float temperature;
} UdpTxPacket_t;

void UDP_Server_Task(void);

#endif /* INC_UDP_COMM_H_ */

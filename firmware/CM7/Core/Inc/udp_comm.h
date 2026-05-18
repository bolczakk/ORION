/*
 * udp_comm.h
 *
 *  Created on: 18 maj 2026
 *      Author: Oleg
 */

#ifndef INC_UDP_COMM_H_
#define INC_UDP_COMM_H_

#include <stdint.h>

/* Inicjalizacja serwera UDP */
void UDP_Init(void);

/* Funkcja do wysyłania danych na PC (aktualizuje dane przed wysłaniem) */
void UDP_Send(float current_speed, float temperature);

#endif /* INC_UDP_COMM_H_ */

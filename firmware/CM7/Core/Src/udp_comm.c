/*
 * udp_comm.c
 *
 *  Created on: 18 maj 2026
 *      Author: Oleg
 */

#include "udp_comm.h"
#include "lwip/udp.h"
#include "string.h"
#include "gpio.h"
#include "shared_data.h"

#define LAPTOP_PORT 5000
#define STM_PORT    5001

/* Prywatne struktury danych */
typedef struct {
	float setpoint;
	float angle;
	float extra_param;
} __attribute__((packed)) LaptopData_t;

typedef struct {
	float current_speed;
	float temperature;
} __attribute__((packed)) STMData_t;

/* Prywatne zmienne pliku */
static LaptopData_t rx_from_laptop = { 0 };
static STMData_t tx_to_laptop = { 0.0f, 24.5f };

static struct udp_pcb *udp_proto_pcb = NULL;
static ip_addr_t laptop_ip;
static uint8_t is_laptop_connected = 0; // Flaga zabezpieczająca przed wysyłaniem w ciemno

static void udp_receive_callback(void *arg, struct udp_pcb *upcb,
		struct pbuf *p, const ip_addr_t *addr, u16_t port) {
	if (p != NULL) {
		if (p->len == sizeof(LaptopData_t)) {
			// 1. Skopiowanie danych do lokalnej struktury
			memcpy(&rx_from_laptop, p->payload, sizeof(LaptopData_t));

			// 2. Przekazanie do PAMIĘCI WSPÓŁDZIELONEJ (Odkomentowane!)
			// Upewnij się, że wskaźnik SHARED_DATA wskazuje na poprawny adres w SRAM (np. D3 SRAM / SRAM4)
			SHARED_DATA->m7_setpoint = rx_from_laptop.setpoint;

			// 3. Sygnalizacja LED
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1);

			// 4. Zapisanie IP nadawcy, aby wiedzieć, gdzie odsyłać telemetrię
			ip_addr_copy(laptop_ip, *addr);
			is_laptop_connected = 1;
		}
		pbuf_free(p);
	}
}

/* Inicjalizacja */
void UDP_Init(void) {
	udp_proto_pcb = udp_new();
	if (udp_proto_pcb != NULL) {
		err_t err = udp_bind(udp_proto_pcb, IP_ADDR_ANY, STM_PORT);
		if (err == ERR_OK) {
			udp_recv(udp_proto_pcb, udp_receive_callback, NULL);
		}
	}
}

/* Wysyłanie telemetrii na PC */
void UDP_Send(float current_speed, float temperature) {
	if (is_laptop_connected && udp_proto_pcb != NULL) {

		// Aktualizacja danych w strukturze
		tx_to_laptop.current_speed = current_speed;
		tx_to_laptop.temperature = temperature;

		struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, sizeof(STMData_t),
				PBUF_RAM);
		if (p != NULL) {
			memcpy(p->payload, &tx_to_laptop, sizeof(STMData_t));
			udp_sendto(udp_proto_pcb, p, &laptop_ip, LAPTOP_PORT);
			pbuf_free(p);
		}
	}
}


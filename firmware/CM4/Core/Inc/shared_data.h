/*
 * shared_data.h
 *
 *  Created on: 17 maj 2026
 *      Author: Oleg
 */

#ifndef INC_SHARED_DATA_H_
#define INC_SHARED_DATA_H_

#include <stdint.h>

// Struktura zawierająca Twoje dane
typedef struct {
	// Dane zapisywane przez M7, czytane przez M4
	volatile float m7_setpoint;
	volatile float m7_angle;

	// Dane zapisywane przez M4, czytane przez M7
	volatile float m4_current_speed;
	volatile float m4_temperature;

} SharedData_t;

// Wskaźnik na stały adres w pamięci SRAM4 (0x38000000)
#define SHARED_DATA ((SharedData_t *) 0x38000000)

#endif /* INC_SHARED_DATA_H_ */

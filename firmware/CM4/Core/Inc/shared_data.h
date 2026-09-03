/*
 * shared_data.h
 *
 *  Created on: 17 maj 2026
 *      Author: Oleg
 */

#ifndef INC_SHARED_DATA_H_
#define INC_SHARED_DATA_H_

#include <stdint.h>

typedef enum {
	STATUS_OK = 0,
	ERR_BME_INIT_FAIL,
	ERR_VL6180X_INIT_FAIL,
	ERR_I2C_TIMEOUT,
	ERR_UROS_DISCONNECTED
} SystemError_t;

typedef struct {
	volatile float m7_linear_speed;
	volatile float m7_angular_speed;

	volatile float m4_motor_left_rpm;
	volatile float m4_motor_right_rpm;

	volatile float m4_angle;
	volatile float m4_pos_x;
	volatile float m4_pos_y;

	volatile float m4_distances[3];

	volatile float m4_temperature;
	volatile float m4_humidity;
	volatile float m4_pressure;

	SystemError_t current_error;

} SharedData_t;

#define SHARED_DATA ((SharedData_t *)0x38000000)

#endif /* INC_SHARED_DATA_H_ */

/*
 * motors.h
 *
 *  Created on: 18 maj 2026
 *      Author: Oleg
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include <stdint.h>
#include "pwm.h"

#define MOTOR_LEFT  0
#define MOTOR_RIGHT 1

void Motors_Init(void);
void Motor_SetDuty(uint8_t motor_id, float duty);
float Motor_GetRPM(uint8_t motor_id);

#endif /* INC_MOTORS_H_ */

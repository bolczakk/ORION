/*
 * motors.c
 *
 *  Created on: 18 maj 2026
 *      Author: Oleg
 */

#include "motors.h"
#include "tim.h"

static PWM_Handle_TypeDef motor_left_pwm;
static PWM_Handle_TypeDef motor_right_pwm;

void Motors_Init(void) {
	// Inicjalizacja Silnika 1
	motor_left_pwm.Timer = &htim4;
	motor_left_pwm.Channel = TIM_CHANNEL_1;
	motor_left_pwm.Duty = 10.0f;
	PWM_Init(&motor_left_pwm);

	//Inicjalizacja Silnika 2
	motor_right_pwm.Timer = &htim4;
	motor_right_pwm.Channel = TIM_CHANNEL_2;
	motor_right_pwm.Duty = 10.0f;
	PWM_Init(&motor_right_pwm);
}

void Motor_SetDuty(uint8_t motor_id, float duty) {
	if (duty > 100.0f)
		duty = 100.0f;
	if (duty < 0.0f)
		duty = 0.0f;

	if (motor_id == MOTOR_LEFT) {
		PWM_WriteDuty(&motor_left_pwm, duty);
	} else if (motor_id == MOTOR_RIGHT) {
		PWM_WriteDuty(&motor_right_pwm, duty);
	}
}

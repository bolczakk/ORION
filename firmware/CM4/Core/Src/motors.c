/*
 * motors.c
 *
 *  Created on: 18 maj 2026
 *      Author: Oleg
 */

#include "motors.h"
#include "tim.h"

static PWM_Handle_TypeDef motor1_pwm;
static PWM_Handle_TypeDef motor2_pwm;

void Motors_Init(void) {
	// Inicjalizacja Silnika 1
	motor1_pwm.Timer = &htim4;
	motor1_pwm.Channel = TIM_CHANNEL_1;
	motor1_pwm.Duty = 50.0f;
	PWM_Init(&motor1_pwm);

	// Inicjalizacja Silnika 2 (przykładowo)
	// motor2_pwm.Timer = &htim4;
	// motor2_pwm.Channel = TIM_CHANNEL_2;
	// motor2_pwm.Duty = 0.0f;
	// PWM_Init(&motor2_pwm);
}

void Motor_SetDuty(uint8_t motor_id, float duty) {
	// Zabezpieczenie przed przekroczeniem zakresu
	if (duty > 100.0f)
		duty = 100.0f;
	if (duty < 0.0f)
		duty = 0.0f;

	if (motor_id == MOTOR_LEFT) {
		PWM_WriteDuty(&motor1_pwm, duty);
	} else if (motor_id == MOTOR_RIGHT) {
		// PWM_WriteDuty(&motor2_pwm, duty);
	}
}

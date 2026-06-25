/*
 * pid.c
 *
 *  Created on: 12 maj 2026
 *      Author: Oleg
 */

#include "pid.h"

arm_pid_instance_f32 pid_left;
arm_pid_instance_f32 pid_right;

void PID_Init(void) {
	pid_left.Kp = 0.2f;
	pid_left.Ki = 0.02f;
	pid_left.Kd = 0.0f;
	arm_pid_init_f32(&pid_left, 1);

	pid_right.Kp = 0.2f;
	pid_right.Ki = 0.02f;
	pid_right.Kd = 0.0f;
	arm_pid_init_f32(&pid_right, 1);
}

float getOutputLeft(float error) {
	return arm_pid_f32(&pid_left, error);
}

float getOutputRight(float error) {
	return arm_pid_f32(&pid_right, error);
}

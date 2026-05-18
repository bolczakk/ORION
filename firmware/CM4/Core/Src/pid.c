/*
 * pid.c
 *
 *  Created on: 12 maj 2026
 *      Author: Oleg
 */

#include "pid.h"

arm_pid_instance_f32 pid;

void PID_Init(void) {
	pid.Kp = 1.0f;
	pid.Ki = 0.5f;
	pid.Kd = 0.1f;

	arm_pid_init_f32(&pid, 1);
}

float getOutput(float error) {
	return arm_pid_f32(&pid, error);
}

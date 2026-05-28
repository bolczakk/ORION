/*
 * pid.c
 *
 *  Created on: 12 maj 2026
 *      Author: Oleg
 */

#include "pid.h"

arm_pid_instance_f32 pid;

void PID_Init(void) {
	pid.Kp = 0.2f;
	pid.Ki = 0.02f;
	pid.Kd = 0.0f;

	arm_pid_init_f32(&pid, 1);
}

float getOutput(float error) {
	return arm_pid_f32(&pid, error);
}

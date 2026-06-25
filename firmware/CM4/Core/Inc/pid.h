/*
 * pid.h
 *
 *  Created on: 12 maj 2026
 *      Author: Oleg
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "arm_math.h"

extern arm_pid_instance_f32 pid;

void PID_Init(void);

float getOutputLeft(float error);

float getOutputRight(float error);

#endif /* INC_PID_H_ */

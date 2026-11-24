#ifndef DRIVE_CONTROL_H
#define DRIVE_CONTROL_H

#include "drive_control_config.h"
#include <math.h>
#include <stdint.h>

typedef struct {
	float distance_error_m;
	int straight_direction;
} DriveStraightSync_t;

static inline float Drive_FilterRpm(float previous, float raw, float dt_s) {
	const float alpha = dt_s / (DRIVE_RPM_FILTER_TAU_S + dt_s);
	return previous + alpha * (raw - previous);
}

static inline void Drive_ResetStraightSync(DriveStraightSync_t *state) {
	state->distance_error_m = 0.0f;
	state->straight_direction = 0;
}

static inline void Drive_WheelTargets(DriveStraightSync_t *state, float v,
		float omega, float delta_left_m, float delta_right_m, float *left_mps,
		float *right_mps) {
	*left_mps = v - omega * DRIVE_TRACK_WIDTH_M * 0.5f;
	*right_mps = v + omega * DRIVE_TRACK_WIDTH_M * 0.5f;

	if (DRIVE_STRAIGHT_GAIN <= 0.0f|| fabsf(v) < DRIVE_STRAIGHT_MIN_SPEED_MPS
	|| fabsf(omega) > DRIVE_STRAIGHT_OMEGA_EPS_RADPS) {
		Drive_ResetStraightSync(state);
		return;
	}

	const int direction = v > 0.0f ? 1 : -1;
	if (state->straight_direction != direction) {
		// Jesli robot zmienil kierunek jazdy zerujemy blad roznicy drog
		state->distance_error_m = 0.0f;
		state->straight_direction = direction;
	} else {
		state->distance_error_m += delta_left_m - delta_right_m;
	}

	const float limit = fminf(DRIVE_STRAIGHT_MAX_CORR_MPS,
	DRIVE_STRAIGHT_MAX_CORR_FRACTION * fabsf(v));

	const float error_limit = 2.0f * limit / DRIVE_STRAIGHT_GAIN;
	state->distance_error_m = fminf(error_limit,
			fmaxf(-error_limit, state->distance_error_m));
	const float correction = 0.5f * DRIVE_STRAIGHT_GAIN
			* state->distance_error_m;
	*left_mps -= correction;
	*right_mps += correction;
}

#endif

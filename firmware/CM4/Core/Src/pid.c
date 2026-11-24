/*
 * pid.c - regulatory PI predkosci kol.
 * Nastawy i kompensacja oporow: drive_control_config.h.
 */
#include "pid.h"
#include "drive_control_config.h"
#include <math.h>
#include <stddef.h>

const PID_WheelConfig_t pid_left_config = { .kp = DRIVE_LEFT_KP, .ki =
DRIVE_LEFT_KI_PER_S, .integral_limit_pwm = DRIVE_PI_INTEGRAL_LIMIT_PWM,

.zero_setpoint_rpm = DRIVE_ZERO_SETPOINT_RPM, .stopped_rpm = DRIVE_STOPPED_RPM,
		.start_detect_rpm = DRIVE_START_DETECT_RPM, .start_max_s =
		DRIVE_START_MAX_S,

		.ks_pwm = DRIVE_LEFT_KS_PWM, .kv_pwm_per_rpm = DRIVE_LEFT_KV,
		.start_pwm = DRIVE_LEFT_START_PWM };

const PID_WheelConfig_t pid_right_config = { .kp = DRIVE_RIGHT_KP, .ki =
DRIVE_RIGHT_KI_PER_S, .integral_limit_pwm = DRIVE_PI_INTEGRAL_LIMIT_PWM,

.zero_setpoint_rpm = DRIVE_ZERO_SETPOINT_RPM, .stopped_rpm = DRIVE_STOPPED_RPM,
		.start_detect_rpm = DRIVE_START_DETECT_RPM, .start_max_s =
		DRIVE_START_MAX_S,

		.ks_pwm = DRIVE_RIGHT_KS_PWM, .kv_pwm_per_rpm = DRIVE_RIGHT_KV,
		.start_pwm = DRIVE_RIGHT_START_PWM };

static PID_WheelState_t left_state;
static PID_WheelState_t right_state;

static float clampf(float value, float low, float high) {
	return fminf(high, fmaxf(low, value));
}

//static int direction_config_valid(const PID_DirectionConfig_t *c) {
//	return isfinite(c->ks_pwm) && c->ks_pwm >= 0.0f && c->ks_pwm <= 100.0f
//			&& isfinite(c->kv_pwm_per_rpm) && c->kv_pwm_per_rpm >= 0.0f
//			&& isfinite(c->start_pwm) && c->start_pwm >= 0.0f
//			&& c->start_pwm <= 100.0f;
//}

//static int config_valid(const PID_WheelConfig_t *c) {
//	return c != NULL && isfinite(c->kp) && c->kp >= 0.0f
//			&& isfinite(c->ki_per_s) && c->ki_per_s >= 0.0f
//			&& isfinite(c->integral_limit_pwm) && c->integral_limit_pwm >= 0.0f
//			&& c->integral_limit_pwm <= 100.0f && isfinite(c->zero_setpoint_rpm)
//			&& c->zero_setpoint_rpm >= 0.0f && isfinite(c->stopped_rpm)
//			&& c->stopped_rpm >= 0.0f && isfinite(c->start_detect_rpm)
//			&& c->start_detect_rpm > 0.0f && isfinite(c->start_max_s)
//			&& c->start_max_s >= 0.0f && direction_config_valid(&c->forward)
//			&& direction_config_valid(&c->reverse);
//}

void PID_WheelReset(PID_WheelState_t *state) {
	if (state != NULL) {
		state->integral_pwm = 0.0f;
		state->start_remaining_s = 0.0f;
		state->command_direction = 0;
	}
}

float PID_WheelStep(PID_WheelState_t *state, const PID_WheelConfig_t *config,
		float setpoint_rpm, float measured_rpm, float dt_s) {
	if (state == NULL || config == NULL) {
		return 0.0f;
	}

	// zatrzymanie kola przy niskich predkosciach i hamowanie
	if (fabsf(setpoint_rpm) <= config->zero_setpoint_rpm) {
		PID_WheelReset(state);
		if (fabsf(measured_rpm) <= config->stopped_rpm) {
			return 0.0f;
		}
		float braking = -config->kp * measured_rpm;
		return clampf(braking, -100.0f, 100.0f);
	}

	// kierunek obrotów
	const int direction = setpoint_rpm > 0.0f ? 1 : -1;

	// zmiana kierunku
	if (direction != state->command_direction) {
		// zerujemy calke
		state->integral_pwm = 0.0f;
		// rozruch tylko gdy koło stoi
		if (fabsf(measured_rpm) <= config->stopped_rpm
				&& config->start_pwm > 0.0f) {

			state->start_remaining_s = config->start_max_s;
		} else {
			state->start_remaining_s = 0.0f;
		}
		state->command_direction = direction;
	}

	// wykrycie rozpoczęcia ruchu, 1/4 zadanej predkosci albo prog wylacza wspomaganie rozruchowe
	const float moving_threshold = fminf(config->start_detect_rpm,
			0.25f * fabsf(setpoint_rpm));

	if (direction * measured_rpm >= moving_threshold) {
		state->start_remaining_s = 0.0f;
	}

	const int starting = state->start_remaining_s > 0.0f;
	state->start_remaining_s =
			state->start_remaining_s <= dt_s + 0.000001f ?
					0.0f : state->start_remaining_s - dt_s;

	const float error = setpoint_rpm - measured_rpm;
	//ks_pwm to stale wzmocnienie, inaczej "kompensacja oporow"
	// kv_pwm_per_rpm to odwrotnosc wzmocnienia statycznego naszego silnika [pwm %/rpm]
//	const float feedforward = direction * config->ks_pwm
//			+ config->kv_pwm_per_rpm * setpoint_rpm;

	// Człon proporcjonalny + Feedforward
	//const float base = feedforward + config->kp * error;
	const float base = config->kp * error;

	// Człon całkujący, dziala od poczatku jesli START_PWM=0
	const float delta_i = starting ? 0.0f : config->ki * error * dt_s;

	// Zabezpieczenie obliczeń
	if (!isfinite(base) || !isfinite(delta_i)) {
		PID_WheelReset(state);
		return 0.0f;
	}

	// Ograniczenie całki
	const float candidate_i = clampf(state->integral_pwm + delta_i,
			-config->integral_limit_pwm, config->integral_limit_pwm);

	const float candidate_output = base + candidate_i;

	// Anti-windup
	if (!((candidate_output > 100.0f && delta_i > 0.0f)
			|| (candidate_output < -100.0f && delta_i < 0.0f))) {

		state->integral_pwm = candidate_i;
	}

	// Wyjście regulatora
	float output = base + state->integral_pwm;

	// Minimalne PWM podczas rozruchu
	if (starting && direction * output < config->start_pwm) {
		output = direction * config->start_pwm;
	}

	// Ograniczenie PWM
	return clampf(output, -100.0f, 100.0f);
}

//float PID_WheelStep(PID_WheelState_t *state, const PID_WheelConfig_t *config,
//		float setpoint_rpm, float measured_rpm, float dt_s) {
////    if (state == NULL) {
////        return 0.0f;
////    }
////    if (!config_valid(config) || !isfinite(setpoint_rpm)
////            || !isfinite(measured_rpm) || !isfinite(dt_s)
////            || dt_s <= 0.0f || dt_s > DRIVE_MAX_DT_S
////            || !isfinite(state->integral_pwm)
////            || !isfinite(state->start_remaining_s)) {
////        PID_WheelReset(state);
////        return 0.0f;
////    }
//
//	/* Zero dotyczy tego kola, a nie predkosci liniowej calego robota.
//	 * Przy zatrzymywaniu pozostaje hamowanie P. Przy postoju PWM = 0;
//	 * calka i feedforward nie wywoluja pelzania ani kolejnych impulsow. */
//
////Jesli setpoint jest za maly, zerujemy
//	if (fabsf(setpoint_rpm) <= config->zero_setpoint_rpm) {
//		PID_WheelReset(state);
//		if (fabsf(measured_rpm) <= config->stopped_rpm) {
//			return 0.0f;
//		}
//		const float braking = -config->kp * measured_rpm;
//		return clampf(braking, -100.0f, 100.0f);
//	}
//
//	const int direction = setpoint_rpm > 0.0f ? 1 : -1;
//	const PID_DirectionConfig_t *ff =
//			direction > 0 ? &config->forward : &config->reverse;
//
//	if (direction != state->command_direction) {
//		state->integral_pwm = 0.0f;
//		/* Jedna proba na nowe polecenie, tylko gdy kolo juz stoi.
//		 * Zmiana kierunku podczas ruchu nie uruchamia impulsu rozruchowego. */
//		state->start_remaining_s =
//				(fabsf(measured_rpm) <= config->stopped_rpm
//						&& ff->start_pwm > 0.0f) ? config->start_max_s : 0.0f;
//		state->command_direction = direction;
//	}
//
//	/* Prog ruchu dostosowany do malych zadanych predkosci.*/
//	const float moving_threshold = fminf(config->start_detect_rpm,
//			0.5f * fabsf(setpoint_rpm));
//	if ((float) direction * measured_rpm >= moving_threshold) {
//		state->start_remaining_s = 0.0f;
//	}
//	const int starting = state->start_remaining_s > 0.0f;
//	state->start_remaining_s =
//			state->start_remaining_s <= dt_s + 0.000001f ?
//					0.0f : state->start_remaining_s - dt_s;
//
//	const float error = setpoint_rpm - measured_rpm;
//	const float feedforward = (float) direction * ff->ks_pwm
//			+ ff->kv_pwm_per_rpm * setpoint_rpm;
//	const float base = feedforward + config->kp * error;
//	const float delta_i = starting ? 0.0f : config->ki_per_s * error * dt_s;
//	if (!isfinite(base) || !isfinite(delta_i)) {
//		PID_WheelReset(state);
//		return 0.0f;
//	}
//
//	const float candidate_i = clampf(state->integral_pwm + delta_i,
//			-config->integral_limit_pwm, config->integral_limit_pwm);
//	const float candidate_output = base + candidate_i;
//	/* Anti-windup uwzglednia CALA sume: FF + P + I.
//	 * Pozwol na calkowanie, jezeli wyprowadza regulator z nasycenia. */
//	if (!((candidate_output > 100.0f && delta_i > 0.0f)
//			|| (candidate_output < -100.0f && delta_i < 0.0f))) {
//		state->integral_pwm = candidate_i;
//	}
//
//	float output = base + state->integral_pwm;
//	if (starting && (float) direction * output < ff->start_pwm) {
//		output = (float) direction * ff->start_pwm;
//	}
//	return clampf(output, -100.0f, 100.0f);
//}

void PID_Init(void) {
	PID_Reset();
}

void PID_Reset(void) {
	PID_WheelReset(&left_state);
	PID_WheelReset(&right_state);
}

float getOutputLeft(float setpoint_rpm, float measured_rpm, float dt_s) {
	return PID_WheelStep(&left_state, &pid_left_config, setpoint_rpm,
			measured_rpm, dt_s);
}

float getOutputRight(float setpoint_rpm, float measured_rpm, float dt_s) {
	return PID_WheelStep(&right_state, &pid_right_config, setpoint_rpm,
			measured_rpm, dt_s);
}

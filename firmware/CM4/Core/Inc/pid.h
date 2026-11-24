/* PI predkosci kola z feedforward i ograniczonym rozruchem. */
#ifndef INC_PID_H_
#define INC_PID_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	float kp;
	float ki;

	float ks_pwm;
	float kv_pwm_per_rpm;
	float start_pwm;

	float start_max_s;
	float start_detect_rpm;

	float zero_setpoint_rpm;
	float stopped_rpm;

	float integral_limit_pwm;
} PID_WheelConfig_t;

typedef struct {
	float integral_pwm;
	float start_remaining_s;
	int command_direction;
} PID_WheelState_t;

extern const PID_WheelConfig_t pid_left_config;
extern const PID_WheelConfig_t pid_right_config;

void PID_WheelReset(PID_WheelState_t *state);
float PID_WheelStep(PID_WheelState_t *state, const PID_WheelConfig_t *config,
		float setpoint_rpm, float measured_rpm, float dt_s);

void PID_Init(void);
void PID_Reset(void);
/* NOWE API: zadana predkosc, zmierzona predkosc, rzeczywisty czas w sekundach.
 * Wynik ma znak i zakres -100..100 %PWM. Wywoluj tylko z zadania napedu. */
float getOutputLeft(float setpoint_rpm, float measured_rpm, float dt_s);
float getOutputRight(float setpoint_rpm, float measured_rpm, float dt_s);

#ifdef __cplusplus
}
#endif
#endif

#ifndef DRIVE_CONTROL_CONFIG_H
#define DRIVE_CONTROL_CONFIG_H

#define DRIVE_PERIOD_MS                  20U
#define DRIVE_MAX_DT_S                   0.100f
#define DRIVE_RPM_FILTER_TAU_S           0.040f
#define DRIVE_TRACK_WIDTH_M              0.274f
#define DRIVE_WHEEL_DIAMETER_M           0.1f
#define DRIVE_TICKS_PER_REV              3840.0f

/* Korekta roznicy drogi dziala tylko podczas jazdy prosto.
 * GAIN = 0 wylacza korekte*/
#define DRIVE_STRAIGHT_GAIN              1.0f
#define DRIVE_STRAIGHT_MAX_CORR_MPS      0.030f
#define DRIVE_STRAIGHT_MAX_CORR_FRACTION 0.25f
#define DRIVE_STRAIGHT_MIN_SPEED_MPS     0.005f
#define DRIVE_STRAIGHT_OMEGA_EPS_RADPS   0.001f

#define DRIVE_LEFT_KP                   1.24f
#define DRIVE_LEFT_KI_PER_S             0.4f
#define DRIVE_RIGHT_KP                  1.24f
#define DRIVE_RIGHT_KI_PER_S            0.4f
#define DRIVE_PI_INTEGRAL_LIMIT_PWM     100.0f
#define DRIVE_ZERO_SETPOINT_RPM         0.10f
#define DRIVE_STOPPED_RPM               1.0f
#define DRIVE_START_DETECT_RPM          2.0f
#define DRIVE_START_MAX_S               0.20f

#define DRIVE_MAX_LINEAR_ACCEL   0.5f  // m/s^2
#define DRIVE_MAX_ANGULAR_ACCEL  1.5f  // rad/s^2

/* KALIBRACJA NA PODLOZU, OSOBNO DLA KAZDEGO KOLA I KIERUNKU.
 * KS: skladowa tarcia podczas ruchu [%PWM].
 * KV: nachylenie charakterystyki [%PWM/RPM].
 * START: minimalne PWM w krotkiej fazie rozruchu, nie staly dolny limit.
 * 0 oznacza brak kompensacji / wylaczony impuls. Nie sa to pomiary robota!
 * Najpierw ustaw KS/KV, dopiero potem START, jezeli jest potrzebny. */
#define DRIVE_LEFT_KS_PWM      0.0f
#define DRIVE_LEFT_KV          0.0f
#define DRIVE_LEFT_START_PWM   12.0f

#define DRIVE_RIGHT_KS_PWM     0.0f
#define DRIVE_RIGHT_KV         0.0f
#define DRIVE_RIGHT_START_PWM  12.0f

#endif

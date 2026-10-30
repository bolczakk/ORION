/* Shared layout for both STM32H7 cores. Compile BOTH firmwares with this file. */
#ifndef INC_SHARED_DATA_H_
#define INC_SHARED_DATA_H_

#include <stddef.h>
#include <stdint.h>

typedef enum {
    STATUS_OK = 0,
    ERR_BME_INIT_FAIL,
    ERR_VL6180X_INIT_FAIL,
    ERR_I2C_TIMEOUT,
    ERR_UROS_DISCONNECTED
} SystemError_t;

typedef struct {
    float m7_linear_speed;
    float m7_angular_speed;
    float m4_motor_left_rpm;
    float m4_motor_right_rpm;
    float m4_angle;
    float m4_pos_x;
    float m4_pos_y;
    float m4_distances[3];
    float m4_temperature;
    float m4_humidity;
    float m4_pressure;

    /* Fixed-width representation of SystemError_t in the shared ABI. */
    uint32_t current_error;

    /* 0 = not published; odd = writing; nonzero even = complete sample. */
    uint32_t m7_cmd_seq;
    uint32_t m4_odom_seq;
} SharedData_t;

#define SHARED_DATA_ADDRESS     UINT32_C(0x38000000)
#define SHARED_DATA_REGION_SIZE 256U
#define SHARED_DATA ((volatile SharedData_t *)(uintptr_t)SHARED_DATA_ADDRESS)

/* Keep the old payload offsets, add only the two counters at the end. */
_Static_assert(sizeof(float) == 4U, "Shared ABI requires 32-bit float");
_Static_assert(offsetof(SharedData_t, current_error) == 52U, "Shared ABI mismatch");
_Static_assert(offsetof(SharedData_t, m7_cmd_seq) == 56U, "Shared ABI mismatch");
_Static_assert(offsetof(SharedData_t, m4_odom_seq) == 60U, "Shared ABI mismatch");
_Static_assert(sizeof(SharedData_t) == 64U, "Shared ABI mismatch; do not pack");
_Static_assert(_Alignof(SharedData_t) >= 4U, "Do not pack shared data");
_Static_assert(sizeof(SharedData_t) <= SHARED_DATA_REGION_SIZE, "MPU region too small");

#endif

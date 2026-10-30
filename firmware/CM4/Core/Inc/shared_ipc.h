#ifndef INC_SHARED_IPC_H_
#define INC_SHARED_IPC_H_

#include <stdbool.h>
#include "stm32h7xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "shared_data.h"

/*
 * Target: STM32H745/755/747/757, two independent firmwares, CMSIS barriers,
 * aligned 32-bit accesses, shared SRAM configured Normal/non-cacheable.
 * This is a target-specific snapshot protocol, not a portable C thread lock.
 *
 * Runtime writers MUST be tasks, never ISRs. Only M7 writes commands; only
 * M4 writes odometry. ALL writes to each group must use these functions.
 * taskENTER_CRITICAL serializes writers on the SAME core (also M7 watchdog).
 * Version checks, not this local critical section, protect the other core.
 *
 * Readers try at most three times. On failure *out is unchanged. Never use
 * its contents after failure unless intentionally retaining a prior sample.
 * The API guarantees consistency, not sample age or motor stopping time.
 */
#define SHARED_READ_ATTEMPTS 3U

typedef struct {
    float linear_speed;
    float angular_speed;
    uint32_t sequence;
} SharedCommand_t;

typedef struct {
    float x;
    float y;
    float yaw;
    float left_rpm;
    float right_rpm;
    uint32_t sequence;
} SharedOdometry_t;

/* M7 ONLY, once per coordinated boot, BEFORE M4 can access shared SRAM.
 * SRAM must be accessible and MPU configured first. Never call from a task
 * or on a micro-ROS reconnect, or while M4 is already running.
 */
static inline void Shared_InitBeforeM4Start(void)
{
    *SHARED_DATA = (SharedData_t){0};
    __DSB();
}

static inline uint32_t Shared_NextSequence(uint32_t current)
{
    uint32_t next = (current & ~UINT32_C(1)) + UINT32_C(2);
    return next != 0U ? next : UINT32_C(2);
}

/* M7 task context. Nested FreeRTOS critical sections are intentional. */
static inline void Shared_PublishCommand(float linear, float angular)
{
    taskENTER_CRITICAL();
    uint32_t next = Shared_NextSequence(SHARED_DATA->m7_cmd_seq);
    SHARED_DATA->m7_cmd_seq = next - 1U;
    __DMB();
    SHARED_DATA->m7_linear_speed = linear;
    SHARED_DATA->m7_angular_speed = angular;
    __DMB();
    SHARED_DATA->m7_cmd_seq = next;
    taskEXIT_CRITICAL();
}

static inline bool Shared_TryReadCommand(SharedCommand_t *out)
{
    if (out == NULL) {
        return false;
    }
    for (uint32_t attempt = 0; attempt < SHARED_READ_ATTEMPTS; ++attempt) {
        uint32_t before = SHARED_DATA->m7_cmd_seq;
        if (before == 0U || (before & 1U) != 0U) {
            continue;
        }
        __DMB();
        SharedCommand_t sample;
        sample.linear_speed = SHARED_DATA->m7_linear_speed;
        sample.angular_speed = SHARED_DATA->m7_angular_speed;
        __DMB();
        uint32_t after = SHARED_DATA->m7_cmd_seq;
        if (before == after) {
            sample.sequence = after;
            *out = sample;
            return true;
        }
    }
    return false;
}

/* M4 task context. Compute pose and filtering BEFORE entering this function. */
static inline void Shared_PublishOdometry(float x, float y, float yaw,
                                         float left_rpm, float right_rpm)
{
    taskENTER_CRITICAL();
    uint32_t next = Shared_NextSequence(SHARED_DATA->m4_odom_seq);
    SHARED_DATA->m4_odom_seq = next - 1U;
    __DMB();
    SHARED_DATA->m4_pos_x = x;
    SHARED_DATA->m4_pos_y = y;
    SHARED_DATA->m4_angle = yaw;
    SHARED_DATA->m4_motor_left_rpm = left_rpm;
    SHARED_DATA->m4_motor_right_rpm = right_rpm;
    __DMB();
    SHARED_DATA->m4_odom_seq = next;
    taskEXIT_CRITICAL();
}

static inline bool Shared_TryReadOdometry(SharedOdometry_t *out)
{
    if (out == NULL) {
        return false;
    }
    for (uint32_t attempt = 0; attempt < SHARED_READ_ATTEMPTS; ++attempt) {
        uint32_t before = SHARED_DATA->m4_odom_seq;
        if (before == 0U || (before & 1U) != 0U) {
            continue;
        }
        __DMB();
        SharedOdometry_t sample;
        sample.x = SHARED_DATA->m4_pos_x;
        sample.y = SHARED_DATA->m4_pos_y;
        sample.yaw = SHARED_DATA->m4_angle;
        sample.left_rpm = SHARED_DATA->m4_motor_left_rpm;
        sample.right_rpm = SHARED_DATA->m4_motor_right_rpm;
        __DMB();
        uint32_t after = SHARED_DATA->m4_odom_seq;
        if (before == after) {
            sample.sequence = after;
            *out = sample;
            return true;
        }
    }
    return false;
}

#endif

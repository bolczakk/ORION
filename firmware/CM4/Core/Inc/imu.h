/*
 * imu.h
 *
 *  Created on: 11 paź 2026
 *      Author: Oleg
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include <stdint.h>
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_REG_WHO_AM_I      0x0FU
#define IMU_WHO_AM_I_VALUE    0x6CU
#define IMU_REG_CTRL1_XL      0x10U
#define IMU_REG_CTRL2_G       0x11U
#define IMU_REG_CTRL3_C       0x12U
#define IMU_REG_CTRL9_XL      0x18U
#define IMU_REG_STATUS        0x1EU
#define IMU_REG_OUT_TEMP_L    0x20U
#define IMU_I2C_TIMEOUT_MS    20U
#define IMU_BUS_WAIT_MS       50U
#define IMU_POLL_MS           5U
#define IMU_STALE_MS          250U
#define IMU_RETRY_MS          1000U
#define IMU_ACCEL_SCALE      (0.000122f * 9.80665f)
#define IMU_GYRO_SCALE       (0.0175f * 0.017453292519943295f)

extern osMutexId_t i2c2MutexHandle;
extern osMutexId_t imuMutexHandle;

typedef enum {
	IMU_STARTING = 0, IMU_READY, IMU_INIT_ERROR, IMU_IO_ERROR, IMU_STALE
} ImuState_t;

typedef struct {
	float accel_mps2[3]; /* X, Y, Z w osiach CZUJNIKA, zawiera grawitacje. */
	float gyro_radps[3]; /* X, Y, Z; bez odejmowania biasu. */
	float temperature_c; /* Temperatura ukladu IMU, nie otoczenia. */
	uint32_t timestamp_ms; /* HAL_GetTick() po ostatnim udanym odczycie. */
	uint32_t sample_count; /* Licznik odczytanych zestawow, nie wszystkich probek ODR. */
	uint32_t error_count; /* Nieudane inicjalizacje/odczyty + brak danych >250 ms. */
	ImuState_t state;
	uint8_t address_7bit; /* 0x6A / 0x6B; 0 jesli nie rozpoznano czujnika. */
	uint8_t valid; /* 1: udany pomiar, 0: brak pomiaru lub blad. */
} ImuSample_t;

/* Tylko taski tego samego rdzenia, po uruchomieniu RTOS. Nie wywolywac w ISR.
 * Zwraca 1 po skopiowaniu snapshotu, 0 dla NULL/braku mutexu/timeoutu.
 * Osobno sprawdz valid i wiek timestamp_ms; snapshot moze zawierac stan bledu.
 */
int Imu_GetLatest(ImuSample_t *out);

uint32_t ImuMsToTicks(uint32_t ms);
HAL_StatusTypeDef ImuReadRegs(uint8_t address, uint8_t reg, uint8_t *data,
		uint16_t size);

HAL_StatusTypeDef ImuWriteReg(uint8_t address, uint8_t reg, uint8_t value);

HAL_StatusTypeDef ImuInit(uint8_t *address);

HAL_StatusTypeDef ImuReadSample(uint8_t address, ImuSample_t *sample,
		uint8_t *has_new_data);

void ImuPublish(const ImuSample_t *sample);

#ifdef __cplusplus
}
#endif

#endif /* INC_IMU_H_ */

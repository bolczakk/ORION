/*
 * imu.c
 *
 *  Created on: 11 paź 2026
 *      Author: Oleg
 */

#include "imu.h"

ImuSample_t imuLatest = { 0 };

uint32_t ImuMsToTicks(uint32_t ms) {
	//uint64_t ticks = ((uint64_t) ms * osKernelGetTickFreq() + 999U) / 1000U;
	//return (ticks > 0U) ? (uint32_t) ticks : 1U;
	return pdMS_TO_TICKS(ms);
}

HAL_StatusTypeDef ImuReadRegs(uint8_t address, uint8_t reg, uint8_t *data,
		uint16_t size) {
	if (osMutexAcquire(i2c2MutexHandle, ImuMsToTicks(IMU_BUS_WAIT_MS))
			!= osOK) {
		return HAL_TIMEOUT;
	}
	/* STM32 HAL przyjmuje adres 7-bit przesuniety o 1 bit w lewo. */
	HAL_StatusTypeDef result = HAL_I2C_Mem_Read(&hi2c2,
			(uint16_t) address << 1U, reg, I2C_MEMADD_SIZE_8BIT, data, size,
			IMU_I2C_TIMEOUT_MS);
	(void) osMutexRelease(i2c2MutexHandle);
	return result;
}

HAL_StatusTypeDef ImuWriteReg(uint8_t address, uint8_t reg, uint8_t value) {
	if (osMutexAcquire(i2c2MutexHandle, ImuMsToTicks(IMU_BUS_WAIT_MS))
			!= osOK) {
		return HAL_TIMEOUT;
	}
	HAL_StatusTypeDef result = HAL_I2C_Mem_Write(&hi2c2,
			(uint16_t) address << 1U, reg, I2C_MEMADD_SIZE_8BIT, &value, 1U,
			IMU_I2C_TIMEOUT_MS);
	(void) osMutexRelease(i2c2MutexHandle);
	return result;
}

HAL_StatusTypeDef ImuInit(uint8_t *address) {
	const uint8_t candidates[] = { 0x6AU, 0x6BU };
	uint8_t id = 0U;
	*address = 0U;
	for (uint32_t i = 0U; i < 2U; ++i) {
		if (ImuReadRegs(candidates[i], IMU_REG_WHO_AM_I, &id, 1U)
				== HAL_OK&& id == IMU_WHO_AM_I_VALUE) {
			*address = candidates[i];
			break;
		}
	}
	if (*address == 0U)
		return HAL_ERROR;

	HAL_StatusTypeDef result = ImuWriteReg(*address, IMU_REG_CTRL3_C, 0x01U);
	if (result != HAL_OK)
		return result;
	uint32_t reset_start = HAL_GetTick();
	uint8_t ctrl3 = 0U;
	do {
		osDelay(ImuMsToTicks(1U));
		result = ImuReadRegs(*address, IMU_REG_CTRL3_C, &ctrl3, 1U);
		if (result != HAL_OK)
			return result;
		if ((uint32_t) (HAL_GetTick() - reset_start) > 100U)
			return HAL_TIMEOUT;
	} while ((ctrl3 & 0x01U) != 0U);

	/* BDU=1: chroni pare LSB/MSB przed nadpisaniem; IF_INC=1: burst. */
	result = ImuWriteReg(*address, IMU_REG_CTRL3_C, 0x44U);
	if (result != HAL_OK)
		return result;
	/* Wylacz I3C, zachowujac pozostale bity CTRL9_XL. */
	uint8_t ctrl9 = 0U;
	result = ImuReadRegs(*address, IMU_REG_CTRL9_XL, &ctrl9, 1U);
	if (result != HAL_OK)
		return result;
	result = ImuWriteReg(*address, IMU_REG_CTRL9_XL, (uint8_t) (ctrl9 | 0x02U));
	if (result != HAL_OK)
		return result;
	result = ImuWriteReg(*address, IMU_REG_CTRL1_XL, 0x48U); /* 104 Hz, +/-4 g */
	if (result != HAL_OK)
		return result;
	result = ImuWriteReg(*address, IMU_REG_CTRL2_G, 0x44U); /* 104 Hz, +/-500 dps */
	if (result != HAL_OK)
		return result;

	/* Czas na ustalenie pomiarow, bez trzymania mutexu magistrali. */
	osDelay(ImuMsToTicks(100U));
	return HAL_OK;
}

int16_t ImuDecodeLE(const uint8_t *bytes) {
	uint16_t word = (uint16_t) bytes[0] | ((uint16_t) bytes[1] << 8U);
	/* Jawna konwersja two's complement, takze dla -32768. */
	int32_t signed_word = (word & 0x8000U) ? (int32_t) word - 65536L : word;
	return (int16_t) signed_word;
}

HAL_StatusTypeDef ImuReadSample(uint8_t address, ImuSample_t *sample,
		uint8_t *has_new_data) {
	uint8_t status = 0U;
	uint8_t raw[14]; /* 20h..2Dh: temp, gyro XYZ, accel XYZ. */
	*has_new_data = 0U;
	if (osMutexAcquire(i2c2MutexHandle, ImuMsToTicks(IMU_BUS_WAIT_MS))
			!= osOK) {
		return HAL_TIMEOUT;
	}
	HAL_StatusTypeDef result = HAL_I2C_Mem_Read(&hi2c2,
			(uint16_t) address << 1U, IMU_REG_STATUS, I2C_MEMADD_SIZE_8BIT,
			&status, 1U, IMU_I2C_TIMEOUT_MS);
	if (result == HAL_OK && (status & 0x03U) == 0x03U) {
		result = HAL_I2C_Mem_Read(&hi2c2, (uint16_t) address << 1U,
		IMU_REG_OUT_TEMP_L, I2C_MEMADD_SIZE_8BIT, raw, sizeof(raw),
		IMU_I2C_TIMEOUT_MS);
		if (result == HAL_OK)
			*has_new_data = 1U;
	}
	(void) osMutexRelease(i2c2MutexHandle);
	if (result != HAL_OK || !*has_new_data)
		return result;

	sample->temperature_c = 25.0f + (float) ImuDecodeLE(&raw[0]) / 256.0f;
	for (uint32_t axis = 0U; axis < 3U; ++axis) {
		sample->gyro_radps[axis] = (float) ImuDecodeLE(
				&raw[2U + axis * 2U]) * IMU_GYRO_SCALE;
		sample->accel_mps2[axis] = (float) ImuDecodeLE(
				&raw[8U + axis * 2U]) * IMU_ACCEL_SCALE;
	}
	return HAL_OK;
}

void ImuPublish(const ImuSample_t *sample) {
	if (osMutexAcquire(imuMutexHandle, osWaitForever) == osOK) {
		imuLatest = *sample;
		(void) osMutexRelease(imuMutexHandle);
	}
}

int Imu_GetLatest(ImuSample_t *out) {
	if (out == NULL || imuMutexHandle == NULL)
		return 0;
	if (osMutexAcquire(imuMutexHandle, ImuMsToTicks(10U)) != osOK)
		return 0;
	*out = imuLatest;
	(void) osMutexRelease(imuMutexHandle);
	return 1;
}

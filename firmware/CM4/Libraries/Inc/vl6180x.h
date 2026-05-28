#ifndef __VL6180X_H
#define __VL6180X_H

#include "stm32h7xx_hal.h"

#define VL6180X_ADDR         (0x29 << 1)

HAL_StatusTypeDef VL6180X_Write8(I2C_HandleTypeDef *hi2c, uint16_t reg,
		uint8_t value);
HAL_StatusTypeDef VL6180X_Read8(I2C_HandleTypeDef *hi2c, uint16_t reg,
		uint8_t *value);
HAL_StatusTypeDef VL6180X_Init(I2C_HandleTypeDef *hi2c);
uint8_t VL6180X_ReadRange(I2C_HandleTypeDef *hi2c);
void VL6180X_SetScaling2x(I2C_HandleTypeDef *hi2c);

#endif

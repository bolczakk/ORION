#ifndef __VL6180X_H
#define __VL6180X_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VL6180X_DEFAULT_ADDRESS_7BIT 0x29U

/* One owner at a time. This driver does not lock the I2C bus. */
typedef struct {
	I2C_HandleTypeDef *i2c;
	uint8_t address_7bit;
	uint8_t scaling;
	int16_t offset_1x;
	uint8_t crosstalk_height_1x;
	bool initialized;
	bool running;
} VL6180X_Device;

typedef struct {
	bool ready; /* A new result was read and acknowledged. */
	bool valid; /* No range error and raw result != 255. */
	uint8_t range_status; /* RESULT__RANGE_STATUS[7:4]. */
	uint16_t distance_mm; /* Already includes the scaling factor. */
} VL6180X_Sample;

/* Call only AFTER a hardware reset/SHUT cycle; does not reset the chip. */
void VL6180X_ResetContext(VL6180X_Device *dev, I2C_HandleTypeDef *i2c);
/* Fresh chip only. After a configuration failure, reset hardware and context. */
HAL_StatusTypeDef VL6180X_Init(VL6180X_Device *dev);
HAL_StatusTypeDef VL6180X_SetAddress(VL6180X_Device *dev, uint8_t address_7bit);
HAL_StatusTypeDef VL6180X_IsReady(VL6180X_Device *dev, bool *ready);
/* Before StartContinuous only; preserves and rescales the factory offset. */
HAL_StatusTypeDef VL6180X_SetScaling2x(VL6180X_Device *dev);
/* Delay between measurements: 10..2550 ms, multiple of 10 ms. */
HAL_StatusTypeDef VL6180X_StartContinuous(VL6180X_Device *dev,
		uint16_t intermeasurement_ms);
/* HAL_OK + !ready means no new sample; an invalid sample is NOT an I2C error. */
HAL_StatusTypeDef VL6180X_ReadRange(VL6180X_Device *dev, VL6180X_Sample *sample);

#ifdef __cplusplus
}
#endif
#endif


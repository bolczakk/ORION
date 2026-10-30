#include "vl6180x.h"
#include <stddef.h>

#define I2C_TIMEOUT_MS                       100U
#define REG_MODEL_ID                         0x0000U
#define REG_MODE_GPIO1                       0x0011U
#define REG_INTERRUPT_CONFIG                 0x0014U
#define REG_INTERRUPT_CLEAR                  0x0015U
#define REG_FRESH_OUT_OF_RESET               0x0016U
#define REG_RANGE_START                      0x0018U
#define REG_INTERMEASUREMENT_PERIOD          0x001BU
#define REG_MAX_CONVERGENCE_TIME             0x001CU
#define REG_CROSSTALK_VALID_HEIGHT           0x0021U
#define REG_PART_TO_PART_OFFSET              0x0024U
#define REG_RANGE_CHECK_ENABLES              0x002DU
#define REG_VHV_RECALIBRATE                  0x002EU
#define REG_VHV_REPEAT_RATE                  0x0031U
#define REG_ALS_ANALOGUE_GAIN                0x003FU
#define REG_ALS_INTEGRATION_PERIOD           0x0040U
#define REG_RANGE_STATUS                     0x004DU
#define REG_INTERRUPT_STATUS                 0x004FU
#define REG_RANGE_VALUE                      0x0062U
#define REG_RANGE_SCALER                     0x0096U
#define REG_AVERAGING_SAMPLE_PERIOD          0x010AU
#define REG_I2C_ADDRESS                      0x0212U

#define MODEL_ID                             0xB4U
#define GPIO1_INTERRUPT_ACTIVE_LOW           0x10U
#define INTERRUPT_NEW_SAMPLE                 0x04U
#define INTERRUPT_RANGE_MASK                 0x07U
#define INTERRUPT_CLEAR_ALL                  0x07U
#define RANGE_START_CONTINUOUS               0x03U
#define RANGE_DEVICE_READY                   0x01U
#define RANGE_CHECK_ECE                      0x01U
#define RANGE_SCALER_2X                      127U

typedef struct {
	uint16_t address;
	uint8_t value;
} RegisterSetting;

static bool HasBus(const VL6180X_Device *dev) {
	return dev != NULL && dev->i2c != NULL && dev->address_7bit >= 0x08U
			&& dev->address_7bit <= 0x77U;
}

static HAL_StatusTypeDef VL6180X_Write8(VL6180X_Device *dev, uint16_t reg,
		uint8_t value) {
	return HAL_I2C_Mem_Write(dev->i2c, (uint16_t) (dev->address_7bit << 1), reg,
	I2C_MEMADD_SIZE_16BIT, &value, 1U,
	I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef VL6180X_Read8(VL6180X_Device *dev, uint16_t reg,
		uint8_t *value) {
	return HAL_I2C_Mem_Read(dev->i2c, (uint16_t) (dev->address_7bit << 1), reg,
	I2C_MEMADD_SIZE_16BIT, value, 1U,
	I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef VL6180X_Write16(VL6180X_Device *dev, uint16_t reg,
		uint16_t value) {
	/* Consecutive registers use most significant byte first. */
	uint8_t bytes[2] = { (uint8_t) (value >> 8), (uint8_t) value };
	return HAL_I2C_Mem_Write(dev->i2c, (uint16_t) (dev->address_7bit << 1), reg,
	I2C_MEMADD_SIZE_16BIT, bytes, 2U,
	I2C_TIMEOUT_MS);
}

void VL6180X_ResetContext(VL6180X_Device *dev, I2C_HandleTypeDef *i2c) {
	if (dev != NULL) {
		*dev = (VL6180X_Device ) { .i2c = i2c, .address_7bit =
				VL6180X_DEFAULT_ADDRESS_7BIT, .scaling = 1U };
	}
}

HAL_StatusTypeDef VL6180X_Init(VL6180X_Device *dev) {
	uint8_t model, fresh, offset, height;
	if (!HasBus(dev) || dev->initialized || dev->running) {
		return HAL_ERROR;
	}
	VL6180X_Read8(dev, REG_MODEL_ID, &model);
	VL6180X_Read8(dev, REG_FRESH_OUT_OF_RESET, &fresh);
	if (model != MODEL_ID || fresh != 1U) {
		return HAL_ERROR;
	}
	/* Save calibration before any write that might depend on scaling. */
	VL6180X_Read8(dev, REG_PART_TO_PART_OFFSET, &offset);
	VL6180X_Read8(dev, REG_CROSSTALK_VALID_HEIGHT, &height);
	dev->offset_1x = offset < 128U ? (int16_t) offset : (int16_t) offset - 256;
	dev->crosstalk_height_1x = height;

	VL6180X_Write8(dev, 0x0207, 0x01);
	VL6180X_Write8(dev, 0x0208, 0x01);
	VL6180X_Write8(dev, 0x0096, 0x00);
	VL6180X_Write8(dev, 0x0097, 0xfd);
	VL6180X_Write8(dev, 0x00e3, 0x00);
	VL6180X_Write8(dev, 0x00e4, 0x04);
	VL6180X_Write8(dev, 0x00e5, 0x02);
	VL6180X_Write8(dev, 0x00e6, 0x01);
	VL6180X_Write8(dev, 0x00e7, 0x03);
	VL6180X_Write8(dev, 0x00f5, 0x02);
	VL6180X_Write8(dev, 0x00d9, 0x05);
	VL6180X_Write8(dev, 0x00db, 0xce);
	VL6180X_Write8(dev, 0x00dc, 0x03);
	VL6180X_Write8(dev, 0x00dd, 0xf8);
	VL6180X_Write8(dev, 0x009f, 0x00);
	VL6180X_Write8(dev, 0x00a3, 0x3c);
	VL6180X_Write8(dev, 0x00b7, 0x00);
	VL6180X_Write8(dev, 0x00bb, 0x3c);
	VL6180X_Write8(dev, 0x00b2, 0x09);
	VL6180X_Write8(dev, 0x00ca, 0x09);
	VL6180X_Write8(dev, 0x0198, 0x01);
	VL6180X_Write8(dev, 0x01b0, 0x17);
	VL6180X_Write8(dev, 0x01ad, 0x00);
	VL6180X_Write8(dev, 0x00ff, 0x05);
	VL6180X_Write8(dev, 0x0100, 0x05);
	VL6180X_Write8(dev, 0x0199, 0x05);
	VL6180X_Write8(dev, 0x01a6, 0x1b);
	VL6180X_Write8(dev, 0x01ac, 0x3e);
	VL6180X_Write8(dev, 0x01a7, 0x1f);
	VL6180X_Write8(dev, 0x0030, 0x00);

	VL6180X_Write8(dev, 0x0011, 0x10); /* Enables polling for New Sample ready when measurement completes */
	VL6180X_Write8(dev, 0x010a, 0x30); /* Set the averaging sample period (compromise between lower noise and increased execution time) */
	VL6180X_Write8(dev, 0x003f, 0x46); /* Sets the light and dark gain (upper nibble). Dark gain should not be changed.*/
	VL6180X_Write8(dev, 0x0031, 0xFF); /* sets the # of range measurements after which auto calibration of system is performed */
	VL6180X_Write8(dev, 0x0040, 0x63); /* Set ALS integration time to 100ms */
	VL6180X_Write8(dev, 0x002e, 0x01); /* perform a single temperature calibration of the ranging sensor */

	VL6180X_Write8(dev, REG_MODE_GPIO1, GPIO1_INTERRUPT_ACTIVE_LOW);
	/* Noise/execution-time compromise recommended by ST. */
	VL6180X_Write8(dev, REG_AVERAGING_SAMPLE_PERIOD, 0x30U);
	/* ALS defaults retained; this application does not start ALS measurements. */
	VL6180X_Write8(dev, REG_ALS_ANALOGUE_GAIN, 0x46U);
	VL6180X_Write8(dev, REG_VHV_REPEAT_RATE, 0xFFU);
	/* 0x0040 is the HIGH byte; 0x0041 is the LOW byte (AN4545 Rev 2 fix). */
	VL6180X_Write8(dev, REG_ALS_INTEGRATION_PERIOD, 99U);
	VL6180X_Write8(dev, REG_VHV_RECALIBRATE, 0x01U);
	VL6180X_Write8(dev, REG_FRESH_OUT_OF_RESET, 0x00U);

	dev->scaling = 1U;
	dev->initialized = true;
	/* Optional: Public registers - See data sheet for more detail */
//	VL6180X_Write8(hi2c, 0x001b, 0x09); /* Set default ranging inter-measurement period to 100ms */
//	VL6180X_Write8(hi2c, 0x003e, 0x31); /* Set default ALS inter-measurement period to 500ms */
//	VL6180X_Write8(hi2c, 0x0014, 0x04);
	return HAL_OK;
}

HAL_StatusTypeDef VL6180X_SetScaling2x(VL6180X_Device *dev) {
	uint8_t checks;
	bool ready;
	if (!HasBus(dev) || !dev->initialized || dev->running) {
		return HAL_ERROR;
	}
	VL6180X_IsReady(dev, &ready);
	if (!ready) {
		return HAL_BUSY;
	}
	VL6180X_Read8(dev, REG_RANGE_CHECK_ENABLES, &checks);
	VL6180X_Write8(dev, REG_MAX_CONVERGENCE_TIME, 63U);
	VL6180X_Write16(dev, REG_RANGE_SCALER, RANGE_SCALER_2X);
	VL6180X_Write8(dev, REG_PART_TO_PART_OFFSET,
			(uint8_t) (dev->offset_1x / 2));
	VL6180X_Write8(dev, REG_CROSSTALK_VALID_HEIGHT,
			(uint8_t) (dev->crosstalk_height_1x / 2U));
	/* Disable ECE at 2x; preserve the other checks. */
	VL6180X_Write8(dev, REG_RANGE_CHECK_ENABLES,
			(uint8_t) (checks & (uint8_t) ~RANGE_CHECK_ECE));
	dev->scaling = 2U;
	return HAL_OK;
}

HAL_StatusTypeDef VL6180X_SetAddress(VL6180X_Device *dev, uint8_t address_7bit) {
	if (!HasBus(dev) || !dev->initialized || dev->running
			|| address_7bit < 0x08U || address_7bit > 0x77U) {
		return HAL_ERROR;
	}
	VL6180X_Write8(dev, REG_I2C_ADDRESS, address_7bit);
	/* The chip changes address immediately. On a failed ACK, caller resets it. */
	dev->address_7bit = address_7bit;
	return HAL_I2C_IsDeviceReady(dev->i2c, (uint16_t) (address_7bit << 1), 3U,
	I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef VL6180X_IsReady(VL6180X_Device *dev, bool *ready) {
	uint8_t status, vhv;
	if (!HasBus(dev) || !dev->initialized || ready == NULL) {
		return HAL_ERROR;
	}
	*ready = false;
	VL6180X_Read8(dev, REG_RANGE_STATUS, &status);
	VL6180X_Read8(dev, REG_VHV_RECALIBRATE, &vhv);
	*ready = (status & RANGE_DEVICE_READY) != 0U && (vhv & 0x03U) == 0U;
	return HAL_OK;
}

HAL_StatusTypeDef VL6180X_StartContinuous(VL6180X_Device *dev,
		uint16_t intermeasurement_ms) {
	bool ready;
	if (!HasBus(dev) || !dev->initialized || dev->running
			|| intermeasurement_ms < 10U || intermeasurement_ms > 2550U
			|| intermeasurement_ms % 10U != 0U) {
		return HAL_ERROR;
	}
	VL6180X_IsReady(dev, &ready);
	if (!ready) {
		return HAL_BUSY;
	}
	VL6180X_Write8(dev, REG_INTERRUPT_CONFIG, INTERRUPT_NEW_SAMPLE);
	VL6180X_Write8(dev, REG_INTERMEASUREMENT_PERIOD,
			(uint8_t) (intermeasurement_ms / 10U - 1U));
	VL6180X_Write8(dev, REG_INTERRUPT_CLEAR, INTERRUPT_CLEAR_ALL);
	VL6180X_Write8(dev, REG_RANGE_START, RANGE_START_CONTINUOUS);
	dev->running = true;
	return HAL_OK;
}

HAL_StatusTypeDef VL6180X_ReadRange(VL6180X_Device *dev, VL6180X_Sample *sample) {
	uint8_t interrupt_status, range_status, raw, fresh;
	if (sample == NULL) {
		return HAL_ERROR;
	}
	*sample = (VL6180X_Sample ) { 0 };
	if (!HasBus(dev) || !dev->initialized || !dev->running) {
		return HAL_ERROR;
	}
	/* Catch an unexpected reset instead of interpreting a default-scale result. */
	VL6180X_Read8(dev, REG_FRESH_OUT_OF_RESET, &fresh);
	if (fresh != 0U) {
		return HAL_ERROR;
	}
	VL6180X_Read8(dev, REG_INTERRUPT_STATUS, &interrupt_status);
	if ((interrupt_status & INTERRUPT_RANGE_MASK) != INTERRUPT_NEW_SAMPLE) {
		return HAL_OK;
	}
	VL6180X_Read8(dev, REG_RANGE_STATUS, &range_status);
	VL6180X_Read8(dev, REG_RANGE_VALUE, &raw);
	/* Also acknowledge a completed but invalid measurement; do not stall the pipe. */
	VL6180X_Write8(dev, REG_INTERRUPT_CLEAR, INTERRUPT_CLEAR_ALL);
	sample->ready = true;
	sample->range_status = (uint8_t) (range_status >> 4);
	sample->valid = sample->range_status == 0U && raw != 255U;
	sample->distance_mm = (uint16_t) raw * dev->scaling;
	return HAL_OK;
}

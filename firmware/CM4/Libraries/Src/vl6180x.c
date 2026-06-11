#include "vl6180x.h"

HAL_StatusTypeDef VL6180X_Write8(I2C_HandleTypeDef *hi2c, uint16_t reg,
		uint8_t value) {
	return HAL_I2C_Mem_Write(hi2c, VL6180X_ADDR, reg, I2C_MEMADD_SIZE_16BIT,
			&value, 1, 100);
}

HAL_StatusTypeDef VL6180X_Read8(I2C_HandleTypeDef *hi2c, uint16_t reg,
		uint8_t *value) {
	return HAL_I2C_Mem_Read(hi2c, VL6180X_ADDR, reg, I2C_MEMADD_SIZE_16BIT,
			value, 1, 100);
}

HAL_StatusTypeDef VL6180X_Init(I2C_HandleTypeDef *hi2c) {
	VL6180X_Write8(hi2c, 0x0207, 0x01);
	VL6180X_Write8(hi2c, 0x0208, 0x01);
	VL6180X_Write8(hi2c, 0x0096, 0x00);
	VL6180X_Write8(hi2c, 0x0097, 0xfd);
	VL6180X_Write8(hi2c, 0x00e3, 0x00);
	VL6180X_Write8(hi2c, 0x00e4, 0x04);
	VL6180X_Write8(hi2c, 0x00e5, 0x02);
	VL6180X_Write8(hi2c, 0x00e6, 0x01);
	VL6180X_Write8(hi2c, 0x00e7, 0x03);
	VL6180X_Write8(hi2c, 0x00f5, 0x02);
	VL6180X_Write8(hi2c, 0x00d9, 0x05);
	VL6180X_Write8(hi2c, 0x00db, 0xce);
	VL6180X_Write8(hi2c, 0x00dc, 0x03);
	VL6180X_Write8(hi2c, 0x00dd, 0xf8);
	VL6180X_Write8(hi2c, 0x009f, 0x00);
	VL6180X_Write8(hi2c, 0x00a3, 0x3c);
	VL6180X_Write8(hi2c, 0x00b7, 0x00);
	VL6180X_Write8(hi2c, 0x00bb, 0x3c);
	VL6180X_Write8(hi2c, 0x00b2, 0x09);
	VL6180X_Write8(hi2c, 0x00ca, 0x09);
	VL6180X_Write8(hi2c, 0x0198, 0x01);
	VL6180X_Write8(hi2c, 0x01b0, 0x17);
	VL6180X_Write8(hi2c, 0x01ad, 0x00);
	VL6180X_Write8(hi2c, 0x00ff, 0x05);
	VL6180X_Write8(hi2c, 0x0100, 0x05);
	VL6180X_Write8(hi2c, 0x0199, 0x05);
	VL6180X_Write8(hi2c, 0x01a6, 0x1b);
	VL6180X_Write8(hi2c, 0x01ac, 0x3e);
	VL6180X_Write8(hi2c, 0x01a7, 0x1f);
	VL6180X_Write8(hi2c, 0x0030, 0x00);

	VL6180X_Write8(hi2c, 0x0011, 0x10); /* Enables polling for New Sample ready when measurement completes */
	VL6180X_Write8(hi2c, 0x010a, 0x30); /* Set the averaging sample period (compromise between lower noise and increased execution time) */
	VL6180X_Write8(hi2c, 0x003f, 0x46); /* Sets the light and dark gain (upper nibble). Dark gain should not be changed.*/
	VL6180X_Write8(hi2c, 0x0031, 0xFF); /* sets the # of range measurements after which auto calibration of system is performed */
	VL6180X_Write8(hi2c, 0x0040, 0x63); /* Set ALS integration time to 100ms */
	VL6180X_Write8(hi2c, 0x002e, 0x01); /* perform a single temperature calibration of the ranging sensor */

	/* Optional: Public registers - See data sheet for more detail */
//	VL6180X_Write8(hi2c, 0x001b, 0x09); /* Set default ranging inter-measurement period to 100ms */
//	VL6180X_Write8(hi2c, 0x003e, 0x31); /* Set default ALS inter-measurement period to 500ms */
//	VL6180X_Write8(hi2c, 0x0014, 0x04);
	return HAL_OK;
}

void VL6180X_SetScaling2x(I2C_HandleTypeDef *hi2c) {
	VL6180X_Write8(hi2c, 0x001C, 0x3F);
	VL6180X_Write8(hi2c, 0x0097, 0x7F);

	uint8_t rce = 0;
	VL6180X_Read8(hi2c, 0x002D, &rce);
	VL6180X_Write8(hi2c, 0x002D, rce & 0xFE);
}

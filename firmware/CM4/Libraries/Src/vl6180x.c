#include "vl6180x.h"

HAL_StatusTypeDef VL6180X_Write8(I2C_HandleTypeDef *hi2c, uint16_t reg,
		uint8_t value) {
	// HAL samo dba o poprawne ułożenie 16-bitowego adresu i wysłanie danych
	return HAL_I2C_Mem_Write(hi2c, VL6180X_ADDR, reg, I2C_MEMADD_SIZE_16BIT,
			&value, 1, 100);
}

HAL_StatusTypeDef VL6180X_Read8(I2C_HandleTypeDef *hi2c, uint16_t reg,
		uint8_t *value) {
	// HAL poprawnie generuje Repeated Start między wysłaniem adresu a odczytem
	return HAL_I2C_Mem_Read(hi2c, VL6180X_ADDR, reg, I2C_MEMADD_SIZE_16BIT,
			value, 1, 100);
}

HAL_StatusTypeDef VL6180X_Init(I2C_HandleTypeDef *hi2c) {
	// konfiguracja zalecana przez ST

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
	VL6180X_Write8(hi2c, 0x001b, 0x09); /* Set default ranging inter-measurement period to 100ms */
	VL6180X_Write8(hi2c, 0x003e, 0x31); /* Set default ALS inter-measurement period to 500ms */
	VL6180X_Write8(hi2c, 0x0014, 0x24);

	return HAL_OK;
}

uint8_t VL6180X_ReadRange(I2C_HandleTypeDef *hi2c) {
	uint8_t range = 0;
	uint8_t status = 0;
	uint16_t timeout = 0;

	// start pomiaru - jeśli błąd, wyjdź natychmiast
//	if (VL6180X_Write8(hi2c, 0x0018, 0x01) != HAL_OK)
//		return 255;

// czekaj aż gotowe
	do {
		// Wyjdź awaryjnie w razie błędu na I2C
		if (VL6180X_Read8(hi2c, 0x004F, &status) != HAL_OK) {
			return 255;
		}

		timeout++;
		// Wyjdź awaryjnie jeśli czujnik nie odpowiada za długo
		if (timeout > 20000)
			return 255;

	} while ((status & 0x07) != 0x04);

	// odczyt wyniku
	if (VL6180X_Read8(hi2c, 0x0062, &range) != HAL_OK)
		return 255;

	// skasuj flagę przerwania
	VL6180X_Write8(hi2c, 0x0015, 0x07);

	return range;
}

void VL6180X_SetScaling2x(I2C_HandleTypeDef *hi2c) {
	// Te rejestry i wartości konfigurują wewnętrzny zegar i wzmocnienie dla trybu 2x
	VL6180X_Write8(hi2c, 0x0024, 0x01); // Zmiana okresu integracji
	VL6180X_Write8(hi2c, 0x01A6, 0x0D); // Kalibracja lasera dla 2x

	// Kluczowy rejestr: ustawienie samego mnożnika sprzętowego (0x01 = 1x, 0x02 = 2x, 0x03 = 3x)
	VL6180X_Write8(hi2c, 0x0102, 0x02);

	// Ponowna kalibracja czujnika po zmianie mnożnika
	VL6180X_Write8(hi2c, 0x002e, 0x01);
}

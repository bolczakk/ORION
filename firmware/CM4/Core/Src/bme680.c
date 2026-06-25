/*
 * bme680.c
 *
 *  Created on: 18 cze 2026
 *      Author: Oleg
 */

#include "bme680.h"
#include "cmsis_os.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c2;

struct bme68x_data bme_last_data;

uint8_t bme_dev_addr = BME68X_I2C_ADDR_LOW;

BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data,
		uint32_t len, void *intf_ptr) {
	uint8_t dev_addr = *(uint8_t*) intf_ptr;
	if (HAL_I2C_Mem_Read(&hi2c2, (dev_addr << 1), reg_addr, 1, reg_data, len,
			1000) == HAL_OK) {
		return BME68X_OK;
	}
	return BME68X_E_COM_FAIL;
}

BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data,
		uint32_t len, void *intf_ptr) {
	uint8_t dev_addr = *(uint8_t*) intf_ptr;
	if (HAL_I2C_Mem_Write(&hi2c2, (dev_addr << 1), reg_addr, 1,
			(uint8_t*) reg_data, len, 1000) == HAL_OK) {
		return BME68X_OK;
	}
	return BME68X_E_COM_FAIL;
}

void bme68x_delay_us(uint32_t period, void *intf_ptr) {
	uint32_t ms = (period / 1000);
	if (ms == 0)
		ms = 1;

	osDelay(ms);
}

/*
 * bme680.h
 *
 *  Created on: 18 cze 2026
 *      Author: Oleg
 */

#ifndef INC_BME680_H_
#define INC_BME680_H_

#include "bme68x.h"

extern struct bme68x_data bme_last_data;
extern uint8_t bme_dev_addr;

BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data,
                                     uint32_t len, void *intf_ptr);

BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data,
                                      uint32_t len, void *intf_ptr);

void bme68x_delay_us(uint32_t period, void *intf_ptr);

#endif /* INC_BME680_H_ */

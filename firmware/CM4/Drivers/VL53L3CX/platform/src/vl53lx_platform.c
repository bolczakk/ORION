// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved
 *
 * This file is part of VL53LX and is dual licensed,
 * either GPL-2.0+
 * or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "vl53lx_platform.h"
#include "vl53lx_platform_log.h"
#include "i2c.h" // Dodano: Niezbędne, aby kompilator wiedział o hi2c1

// Deklaracja zmiennej reprezentującej magistralę I2C zdefiniowanej w głównym programie
extern I2C_HandleTypeDef hi2c1;

#define trace_print(level, ...) \
	_LOG_TRACE_PRINT(VL53LX_TRACE_MODULE_PLATFORM, \
	level, VL53LX_TRACE_FUNCTION_NONE, ##__VA_ARGS__)

#define trace_i2c(...) \
	_LOG_TRACE_PRINT(VL53LX_TRACE_MODULE_NONE, \
	VL53LX_TRACE_LEVEL_NONE, VL53LX_TRACE_FUNCTION_I2C, ##__VA_ARGS__)

VL53LX_Error VL53LX_CommsInitialise(
	VL53LX_Dev_t *pdev,
	uint8_t       comms_type,
	uint16_t      comms_speed_khz)
{
	return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_CommsClose(
	VL53LX_Dev_t *pdev)
{
	return VL53LX_ERROR_NONE;
}

// ----------------------------------------------------------------------
// GŁÓWNE FUNKCJE KOMUNIKACYJNE (Wypełnione przez HAL I2C)
// ----------------------------------------------------------------------

VL53LX_Error VL53LX_WriteMulti(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint8_t      *pdata,
	uint32_t      count)
{
    if (HAL_I2C_Mem_Write(&hi2c1, pdev->i2c_slave_address, index, I2C_MEMADD_SIZE_16BIT, pdata, count, 100) == HAL_OK) {
        return VL53LX_ERROR_NONE;
    }
    return VL53LX_ERROR_CONTROL_INTERFACE;
}

VL53LX_Error VL53LX_ReadMulti(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint8_t      *pdata,
	uint32_t      count)
{
    if (HAL_I2C_Mem_Read(&hi2c1, pdev->i2c_slave_address, index, I2C_MEMADD_SIZE_16BIT, pdata, count, 100) == HAL_OK) {
        return VL53LX_ERROR_NONE;
    }
    return VL53LX_ERROR_CONTROL_INTERFACE;
}

// Pozostałe funkcje Wr/Rd pozostają bez zmian, bo korzystają z WriteMulti/ReadMulti pod spodem.
VL53LX_Error VL53LX_WrByte(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint8_t       data)
{
	VL53LX_Error status = VL53LX_ERROR_NONE;
	uint8_t  buffer[1];

	buffer[0] = (uint8_t)(data);
	status = VL53LX_WriteMulti(pdev, index, buffer, 1);
	return status;
}

VL53LX_Error VL53LX_WrWord(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint16_t      data)
{
	VL53LX_Error status = VL53LX_ERROR_NONE;
	uint8_t  buffer[2];

	buffer[0] = (uint8_t)(data >> 8);
	buffer[1] = (uint8_t)(data &  0x00FF);
	status = VL53LX_WriteMulti(pdev, index, buffer, VL53LX_BYTES_PER_WORD);
	return status;
}

VL53LX_Error VL53LX_WrDWord(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint32_t      data)
{
	VL53LX_Error status = VL53LX_ERROR_NONE;
	uint8_t  buffer[4];

	buffer[0] = (uint8_t) (data >> 24);
	buffer[1] = (uint8_t)((data &  0x00FF0000) >> 16);
	buffer[2] = (uint8_t)((data &  0x0000FF00) >> 8);
	buffer[3] = (uint8_t) (data &  0x000000FF);
	status = VL53LX_WriteMulti(pdev, index, buffer, VL53LX_BYTES_PER_DWORD);
	return status;
}

VL53LX_Error VL53LX_RdByte(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint8_t      *pdata)
{
	VL53LX_Error status = VL53LX_ERROR_NONE;
	uint8_t  buffer[1];

	status = VL53LX_ReadMulti(pdev, index, buffer, 1);
	*pdata = buffer[0];
	return status;
}

VL53LX_Error VL53LX_RdWord(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint16_t     *pdata)
{
	VL53LX_Error status = VL53LX_ERROR_NONE;
	uint8_t  buffer[2];

	status = VL53LX_ReadMulti(pdev, index, buffer, VL53LX_BYTES_PER_WORD);
	*pdata = (uint16_t)(((uint16_t)(buffer[0])<<8) + (uint16_t)buffer[1]);
	return status;
}

VL53LX_Error VL53LX_RdDWord(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint32_t     *pdata)
{
	VL53LX_Error status = VL53LX_ERROR_NONE;
	uint8_t  buffer[4];

	status = VL53LX_ReadMulti(pdev, index, buffer, VL53LX_BYTES_PER_DWORD);
	*pdata = ((uint32_t)buffer[0]<<24) + ((uint32_t)buffer[1]<<16) + ((uint32_t)buffer[2]<<8) + (uint32_t)buffer[3];
	return status;
}

// ----------------------------------------------------------------------
// FUNKCJE CZASOWE I OPÓŹNIAJĄCE
// ----------------------------------------------------------------------

VL53LX_Error VL53LX_WaitUs(
	VL53LX_Dev_t *pdev,
	int32_t       wait_us)
{
    // Zamieniamy na ms i zaokrąglamy w górę
    HAL_Delay((wait_us / 1000) + 1);
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WaitMs(
	VL53LX_Dev_t *pdev,
	int32_t       wait_ms)
{
    HAL_Delay(wait_ms);
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GetTickCount(
	VL53LX_Dev_t *pdev,
	uint32_t *ptick_count_ms)
{
	(void) pdev;
	// HAL_GetTick() zwraca czas w milisekundach od uruchomienia uC
    *ptick_count_ms = HAL_GetTick(); 

	trace_print(
	VL53LX_TRACE_LEVEL_DEBUG,
	"VL53LX_GetTickCount() = %5u ms;\n",
	*ptick_count_ms);

	return VL53LX_ERROR_NONE;
}

// ----------------------------------------------------------------------
// FUNKCJE NIEUŻYWANE BEZPOŚREDNIO PRZEZ PROSTE API (Zwracają OK)
// ----------------------------------------------------------------------

VL53LX_Error VL53LX_GetTimerFrequency(int32_t *ptimer_freq_hz)
{
	return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GetTimerValue(int32_t *ptimer_count)
{
	return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GpioSetMode(uint8_t pin, uint8_t mode)
{
	return VL53LX_ERROR_NONE;
}

VL53LX_Error  VL53LX_GpioSetValue(uint8_t pin, uint8_t value)
{
	return VL53LX_ERROR_NONE;
}

VL53LX_Error  VL53LX_GpioGetValue(uint8_t pin, uint8_t *pvalue)
{
	return VL53LX_ERROR_NONE;
}

VL53LX_Error  VL53LX_GpioXshutdown(uint8_t value)
{
	return VL53LX_ERROR_NONE;
}

VL53LX_Error  VL53LX_GpioCommsSelect(uint8_t value)
{
	return VL53LX_ERROR_NONE;
}

VL53LX_Error  VL53LX_GpioPowerEnable(uint8_t value)
{
	return VL53LX_ERROR_NONE;
}

VL53LX_Error  VL53LX_GpioInterruptEnable(void (*function)(void), uint8_t edge_type)
{
	return VL53LX_ERROR_NONE;
}

VL53LX_Error  VL53LX_GpioInterruptDisable(void)
{
	return VL53LX_ERROR_NONE;
}

// Funkcja WaitValueMaskEx nie wymagała modyfikacji - operuje na skonfigurowanym wyżej I2C
VL53LX_Error VL53LX_WaitValueMaskEx(
	VL53LX_Dev_t *pdev,
	uint32_t      timeout_ms,
	uint16_t      index,
	uint8_t       value,
	uint8_t       mask,
	uint32_t      poll_delay_ms)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;
	uint32_t     start_time_ms   = 0;
	uint32_t     current_time_ms = 0;
	uint8_t      byte_value      = 0;
	uint8_t      found           = 0;
#ifdef VL53LX_LOG_ENABLE
	uint32_t     trace_functions = 0;
#endif

#ifdef VL53LX_LOG_ENABLE
	trace_i2c("WaitValueMaskEx(%5d, 0x%04X, 0x%02X, 0x%02X, %5d);\n",
		timeout_ms, index, value, mask, poll_delay_ms);
#endif

	VL53LX_GetTickCount(pdev, &start_time_ms);
	pdev->new_data_ready_poll_duration_ms = 0;

#ifdef VL53LX_LOG_ENABLE
	trace_functions = _LOG_GET_TRACE_FUNCTIONS();
#endif
	_LOG_SET_TRACE_FUNCTIONS(VL53LX_TRACE_FUNCTION_NONE);

	while ((status == VL53LX_ERROR_NONE) &&
		   (pdev->new_data_ready_poll_duration_ms < timeout_ms) &&
		   (found == 0))
	{
		status = VL53LX_WaitMs(pdev, poll_delay_ms);
		status = VL53LX_RdByte(pdev, index, &byte_value);

		if ((byte_value & mask) == value)
		{
			found = 1;
		}

		VL53LX_GetTickCount(pdev, &current_time_ms);
		pdev->new_data_ready_poll_duration_ms = current_time_ms - start_time_ms;
	}

#ifdef VL53LX_LOG_ENABLE
	_LOG_SET_TRACE_FUNCTIONS(trace_functions);
#endif
	if (found == 0 && status == VL53LX_ERROR_NONE)
		status = VL53LX_ERROR_TIME_OUT;

	return status;
}
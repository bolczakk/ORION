#ifndef __SSD1306_CONF_H__
#define __SSD1306_CONF_H__

#include "main.h"

// --- 1. Wybór Rodziny Mikrokontrolera ---
#define STM32F7

// --- 2. Wybór Interfejsu (TO BYŁO BRAKUJĄCE!) ---
#define SSD1306_USE_I2C

// --- 3. Konfiguracja I2C ---
#define SSD1306_I2C_PORT        hi2c1
#define SSD1306_I2C_ADDR        (0x3C << 1) // lub (0x78)

// --- 4. Rozdzielczość ---
#define SSD1306_HEIGHT          64
#define SSD1306_WIDTH           128

// --- 5. Czcionki (Włączenie) ---
#define SSD1306_INCLUDE_FONT_6x8
#define SSD1306_INCLUDE_FONT_7x10
#define SSD1306_INCLUDE_FONT_11x18
#define SSD1306_INCLUDE_FONT_16x26

#endif /* __SSD1306_CONF_H__ */

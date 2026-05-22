#ifndef OLED_SSD1306_H
#define OLED_SSD1306_H

#include "stm32g4xx_hal.h"
#include <stdint.h>

void OLED_Init(I2C_HandleTypeDef *hi2c);
void OLED_Clear(void);
void OLED_ShowString(uint8_t x, uint8_t y, const char *str);
void OLED_ShowKey(uint8_t key);

#endif

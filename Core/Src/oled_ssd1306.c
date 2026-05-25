#include "oled_ssd1306.h"

#define OLED_I2C_ADDR 0x78U
#define OLED_WIDTH 128U
#define OLED_PAGES 8U

static I2C_HandleTypeDef *g_oled_i2c = NULL;

static const uint8_t font_space[6] = {0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
static const uint8_t font_S[6] = {0x46U, 0x49U, 0x49U, 0x49U, 0x31U, 0x00U};
static const uint8_t font_digits[10][6] = {
  {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU, 0x00U},
  {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U, 0x00U},
  {0x42U, 0x61U, 0x51U, 0x49U, 0x46U, 0x00U},
  {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U, 0x00U},
  {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U, 0x00U},
  {0x27U, 0x45U, 0x45U, 0x45U, 0x39U, 0x00U},
  {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U, 0x00U},
  {0x01U, 0x71U, 0x09U, 0x05U, 0x03U, 0x00U},
  {0x36U, 0x49U, 0x49U, 0x49U, 0x36U, 0x00U},
  {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU, 0x00U}
};

static const uint8_t *OLED_Font6x8(char c)
{
  if (c == 'S')
  {
    return font_S;
  }
  if ((c >= '0') && (c <= '9'))
  {
    return font_digits[(uint8_t)(c - '0')];
  }
  return font_space;
}

static void OLED_WriteCommand(uint8_t cmd)
{
  uint8_t buf[2] = {0x00U, cmd};
  (void)HAL_I2C_Master_Transmit(g_oled_i2c, OLED_I2C_ADDR, buf, 2U, 10U);
}

static void OLED_WriteData(uint8_t data)
{
  uint8_t buf[2] = {0x40U, data};
  (void)HAL_I2C_Master_Transmit(g_oled_i2c, OLED_I2C_ADDR, buf, 2U, 10U);
}

static void OLED_SetPos(uint8_t x, uint8_t y)
{
  OLED_WriteCommand((uint8_t)(0xB0U + y));
  OLED_WriteCommand((uint8_t)(((x & 0xF0U) >> 4U) | 0x10U));
  OLED_WriteCommand((uint8_t)(x & 0x0FU));
}

void OLED_Init(I2C_HandleTypeDef *hi2c)
{
  g_oled_i2c = hi2c;
  HAL_Delay(100U);

  OLED_WriteCommand(0xAEU);
  OLED_WriteCommand(0x20U);
  OLED_WriteCommand(0x10U);
  OLED_WriteCommand(0xB0U);
  OLED_WriteCommand(0xC8U);
  OLED_WriteCommand(0x00U);
  OLED_WriteCommand(0x10U);
  OLED_WriteCommand(0x40U);
  OLED_WriteCommand(0x81U);
  OLED_WriteCommand(0xFFU);
  OLED_WriteCommand(0xA1U);
  OLED_WriteCommand(0xA6U);
  OLED_WriteCommand(0xA8U);
  OLED_WriteCommand(0x3FU);
  OLED_WriteCommand(0xA4U);
  OLED_WriteCommand(0xD3U);
  OLED_WriteCommand(0x00U);
  OLED_WriteCommand(0xD5U);
  OLED_WriteCommand(0xF0U);
  OLED_WriteCommand(0xD9U);
  OLED_WriteCommand(0x22U);
  OLED_WriteCommand(0xDAU);
  OLED_WriteCommand(0x12U);
  OLED_WriteCommand(0xDBU);
  OLED_WriteCommand(0x20U);
  OLED_WriteCommand(0x8DU);
  OLED_WriteCommand(0x14U);
  OLED_WriteCommand(0xAFU);
}

void OLED_Clear(void)
{
  for (uint8_t page = 0U; page < OLED_PAGES; ++page)
  {
    OLED_SetPos(0U, page);
    for (uint8_t col = 0U; col < OLED_WIDTH; ++col)
    {
      OLED_WriteData(0x00U);
    }
  }
}

void OLED_ShowString(uint8_t x, uint8_t y, const char *str)
{
  uint8_t pos_x = x;
  uint8_t pos_y = y;

  if (str == NULL)
  {
    return;
  }

  while (*str != '\0')
  {
    if (pos_x > 122U)
    {
      pos_x = 0U;
      ++pos_y;
      if (pos_y >= OLED_PAGES)
      {
        break;
      }
    }

    OLED_SetPos(pos_x, pos_y);
    const uint8_t *glyph = OLED_Font6x8(*str);
    for (uint8_t i = 0U; i < 6U; ++i)
    {
      OLED_WriteData(glyph[i]);
    }
    pos_x = (uint8_t)(pos_x + 6U);
    ++str;
  }
}

void OLED_ShowKey(uint8_t key)
{
  char buf[4] = {'S', '0', '\0', '\0'};

  if (key <= 9U)
  {
    buf[1] = (char)('0' + key);
    buf[2] = '\0';
  }
  else if (key <= 16U)
  {
    buf[1] = '1';
    buf[2] = (char)('0' + (key - 10U));
    buf[3] = '\0';
  }
  else
  {
    buf[1] = '0';
    buf[2] = '\0';
  }

  OLED_Clear();
  OLED_ShowString(0U, 0U, buf);
}

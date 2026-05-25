#include "keypad.h"

#include "main.h"

#define KEYPAD_ROWS 4U
#define KEYPAD_COLS 4U

static volatile uint8_t g_keypad_pending = 0U;//按键事件标志，1表示有按键事件待处理，0表示没有
static uint8_t g_keypad_locked = 0U;//按键锁定标志，1表示按键被锁定（即正在处理一个按键事件），0表示按键未锁定
static uint32_t g_keypad_last_tick = 0U;

static const uint16_t g_row_pins[KEYPAD_ROWS] = {
  ROW1_Pin,
  ROW2_Pin,
  ROW3_Pin,
  ROW4_Pin
};

static const uint16_t g_col_pins[KEYPAD_COLS] = {
  COL1_Pin,
  COL2_Pin,
  COL3_Pin,
  COL4_Pin
};

static const char g_keymap[KEYPAD_ROWS][KEYPAD_COLS] = {
  // {'1', '2', '3', 'A'},
  // {'4', '5', '6', 'B'},
  // {'7', '8', '9', 'C'},
  // {'*', '0', '#', 'D'}
  {'D','#','0','*'},
   {'C','9','8','7'},
   {'B','6','5','4'},
   {'A','3','2','1'}
};

//获取行引脚的掩码，用于一次性操作所有行引脚
static uint16_t Keypad_RowMask(void)
{
  return (uint16_t)(ROW1_Pin | ROW2_Pin | ROW3_Pin | ROW4_Pin);
}

//检查所有列引脚是否都处于高电平状态，返回1表示所有列都高，返回0表示至少有一列为低电平
static uint8_t Keypad_AllColsHigh(void)
{
  for (uint8_t col = 0U; col < KEYPAD_COLS; ++col)
  {
    if (HAL_GPIO_ReadPin(GPIOD, g_col_pins[col]) == GPIO_PIN_RESET)
    {
      return 0U;
    }
  }
  return 1U;
}

static int Keypad_Scan(void)
{
  const uint16_t row_mask = Keypad_RowMask();
  uint8_t col_low[KEYPAD_COLS] = {0U};
  uint8_t any_low = 0U;

  HAL_GPIO_WritePin(GPIOD, row_mask, GPIO_PIN_RESET);

  for (uint8_t col = 0U; col < KEYPAD_COLS; ++col)
  {
    if (HAL_GPIO_ReadPin(GPIOD, g_col_pins[col]) == GPIO_PIN_RESET)
    {
      col_low[col] = 1U;
      any_low = 1U;
    }
  }

  if (any_low == 0U)
  {
    return -1;
  }

  for (uint8_t col = 0U; col < KEYPAD_COLS; ++col)
  {
    if (col_low[col] == 0U)
    {
      continue;
    }

    for (uint8_t row = 0U; row < KEYPAD_ROWS; ++row)
    {
      HAL_GPIO_WritePin(GPIOD, row_mask, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOD, g_row_pins[row], GPIO_PIN_SET);

      for (volatile uint32_t d = 0U; d < 50U; ++d)
      {
        __NOP();
      }

      if (HAL_GPIO_ReadPin(GPIOD, g_col_pins[col]) == GPIO_PIN_SET)
      {
        HAL_GPIO_WritePin(GPIOD, row_mask, GPIO_PIN_RESET);
        return (int)g_keymap[row][col];
      }
    }
  }

  HAL_GPIO_WritePin(GPIOD, row_mask, GPIO_PIN_RESET);
  return -1;
}

void Keypad_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  HAL_GPIO_WritePin(GPIOD, Keypad_RowMask(), GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = COL1_Pin | COL2_Pin | COL3_Pin | COL4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

int Keypad_GetKey(void)
{
  const uint32_t now = HAL_GetTick();
  if ((now - g_keypad_last_tick) < 25U)
  {
    return -1;
  }

  if (g_keypad_locked != 0U)
  {
    if (Keypad_AllColsHigh() != 0U)
    {
      g_keypad_locked = 0U;
    }
  }

  if (g_keypad_pending == 0U)
  {
    return -1;
  }

  g_keypad_pending = 0U;

  if (g_keypad_locked != 0U)
  {
    return -1;
  }

  const int key = Keypad_Scan();
  if (key > 0)
  {
    g_keypad_locked = 1U;
    g_keypad_last_tick = now;
  }
  return key;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if ((GPIO_Pin == COL1_Pin) || (GPIO_Pin == COL2_Pin) ||
      (GPIO_Pin == COL3_Pin) || (GPIO_Pin == COL4_Pin))
  {
    g_keypad_pending = 1U;
  }
}

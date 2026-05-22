#include "keypad.h"

#include "main.h"

#define KEYPAD_ROWS 4U
#define KEYPAD_COLS 4U

static volatile uint8_t g_keypad_pending = 0U;
static uint8_t g_keypad_locked = 0U;

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

static uint16_t Keypad_RowMask(void)
{
  return (uint16_t)(ROW1_Pin | ROW2_Pin | ROW3_Pin | ROW4_Pin);
}

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
        return (int)(row * KEYPAD_COLS + col + 1U);
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

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdbool.h>
#include <stdio.h>
#include "keypad.h"
#include "lcd_driver.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define CTRL_PERIOD_MS               (1U)  //����ѭ�����ڣ�ms
#define CTRL_PERIOD_S                (0.001f)//�����ʾ����ѭ�����ڣ�?0.001s
//PWM��С/���ռ�ձ�?
#define DUTY_MIN                     (0.02f)
#define DUTY_MAX                     (0.95f)
//��ѹ�趨�����޺Ͳ���
#define VOUT_SET_MIN_CV              (200U)   /* 2.00V  */
#define VOUT_SET_MAX_CV              (3300U)  /* 33.00V */
#define VOUT_SET_STEP_CV             (2U)     /* 0.02V  */
//�����趨�����޺Ͳ���
#define IOUT_SET_MIN_DA              (1U)     /* 0.1A */
#define IOUT_SET_MAX_DA              (20U)    /* 2.0A */
#define IOUT_SET_STEP_DA             (1U)     /* 0.1A */
//ADC���̶�
#define ADC_FULL_SCALE               (4095.0f)
#define ADC_VREF                     (3.3f)
//����
/* ��ʵ��궨�޸�?: Vout = Vadc * VOLTAGE_SCALE */
#define VOLTAGE_SCALE                (13.0f)
/* ��ʵ��궨�޸�?: Iout = (Vadcx - CURRENT_OFFSET_V) * CURRENT_SCALE_A_PER_V */
#define CURRENT_SCALE_A_PER_V        (2.5f)
#define CURRENT_OFFSET_V             (0.0f)

#define IIR_ALPHA                    (0.3f)

#define CV_KP                        (0.100f)
#define CV_KI                        (10.0f)
#define CC_KP                        (0.0024f)
#define CC_KI                        (0.024f)
//软启�?: 从最小占空比到PI目标的斜坡时�?
#define SOFTSTART_MS                 (100U)   // 100ms斜坡
//设定值平滑变�?: 每控制周�?(1ms)设定值最大变化量(0.01V单位, 0.04V/cycle = 40V/s)
#define VOUT_SLEW_RATE_CV            (4U)     // 0.04V per 1ms, 单位0.01V
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

typedef enum
{
  MODE_CV = 0,
  MODE_CC = 1
} control_mode_t;  //����ģʽ����ѹ��CV���������CC��

typedef struct
{
  float kp;
  float ki;
  float integral;
  float out_min;
  float out_max;
} pi_ctrl_t;  //PI������������״̬����

typedef enum
{
  DISPLAY_NORMAL = 0,
  DISPLAY_EDIT,
  DISPLAY_MSG_CANCEL,
  DISPLAY_MSG_CONFIRM
} display_state_t;

static control_mode_t g_mode = MODE_CV;  //��ǰ����ģʽ

//��ѹĿ�����͵���Ŀ��������λ�ֱ���0.01V��0.1A����ʼֵ�ֱ���5.00V��1.0A
static uint16_t g_vset_cv = 500U; /* 5.00V */
static uint8_t g_iset_da = 10U;   /* 1.0A */
//�����ѹ�͵����Ĳ���ֵ���˲����ֵ
static float g_vout_real = 0.0f;
static float g_iout_real = 0.0f;
static float g_vout_filt = 0.0f;
static float g_iout_filt = 0.0f;
//当前目标占空比，初始化为�?小占空比（安全启动）
static float g_duty_cmd = DUTY_MIN;
//ADC原始数据
static uint16_t g_adc_raw_v = 0U;
static uint16_t g_adc_raw_i = 0U;
static volatile uint16_t g_adc_dma[2] = {0U, 0U};
//CV和CC控制器初始化: 积分从最小占空比�?�?
static pi_ctrl_t g_pi_cv = {CV_KP, CV_KI, DUTY_MIN, DUTY_MIN, DUTY_MAX};
static pi_ctrl_t g_pi_cc = {CC_KP, CC_KI, DUTY_MIN, DUTY_MIN, DUTY_MAX};
static volatile uint8_t g_uart_send_pending = 0U;
//软启动状�?
static uint8_t g_softstart_active = 0U;
static uint32_t g_softstart_tick = 0U;
//设定值平滑变�?: 实际用于PI的设定�??(初始与g_vset_cv�?�?)
static uint16_t g_vset_ramp = 500U;  /* 5.00V */

//�༭ģʽ��ر���?
static display_state_t g_display_state = DISPLAY_NORMAL;
static uint8_t g_edit_buf[6];     // �洢6λ��������: [V1,V2,Vf1,Vf2, I1,If1]
static uint8_t g_edit_idx = 0;    // ��ǰ����λ�� 0~5
static uint32_t g_msg_tick = 0;   // ������Ϣ��ʾ��ʱ

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

static float clampf(float x, float min_val, float max_val);
static void Power_ReadFeedback(void);
static void Power_ControlStep(float dt_s);
static void Power_ApplyDuty(float duty);
static void Power_HandleKeyboard(void);
static float PI_Update(pi_ctrl_t *pi, float setpoint, float feedback, float dt_s);
static void PI_Reset(pi_ctrl_t *pi, float preload);
static void Uart_SendSetpoints(void);
static void Oled_UpdateDisplay(void);
__weak int Keypad_GetKey(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//����
static float clampf(float x, float min_val, float max_val)  
{
  if (x < min_val)
  {
    return min_val;
  }
  if (x > max_val)
  {
    return max_val;
  }
  return x;
}
//PI���������ú�������pi->integral����ΪԤ����ֵpreload����ֹģʽ�л�ʱ�����������ͻ��?
static void PI_Reset(pi_ctrl_t *pi, float preload)
{
  pi->integral = preload;
}
//PI���������º����������趨ֵsetpoint������ֵfeedback��ʱ�䲽��dt_s�����µĿ������?
static float PI_Update(pi_ctrl_t *pi, float setpoint, float feedback, float dt_s)
{
  const float err = setpoint - feedback;  //���?
  const float p_term = pi->kp * err;  //������
  float out = p_term + pi->integral;  //��ʱ���?

  if ((out > pi->out_min) && (out < pi->out_max))
  {
    pi->integral += pi->ki * err * dt_s;  //�������δ����ʱ�Ÿ��»������ֹ���ֱ���?
  }
  //�������out
  pi->integral = clampf(pi->integral, pi->out_min, pi->out_max);
  out = p_term + pi->integral;
  out = clampf(out, pi->out_min, pi->out_max);
  return out;
}
//
static void Power_ReadFeedback(void)
{
  g_adc_raw_v = g_adc_dma[0];
  g_adc_raw_i = g_adc_dma[1];
  //根据ADC原始值计算实际电压和电流
  {
    const float vadc_v = ((float)g_adc_raw_v / ADC_FULL_SCALE) * ADC_VREF;
    const float vadc_i = ((float)g_adc_raw_i / ADC_FULL_SCALE) * ADC_VREF;

    g_vout_real = vadc_v * VOLTAGE_SCALE;
    g_iout_real = (vadc_i - CURRENT_OFFSET_V) * CURRENT_SCALE_A_PER_V;
    if (g_iout_real < 0.0f)
    {
      g_iout_real = 0.0f;
    }

    /* 电压: 标准IIR滤波 (硬件过采�?16x已在前端处理噪声) */
    g_vout_filt += IIR_ALPHA * (g_vout_real - g_vout_filt);
    /* 电流: 标准IIR滤波 */
    g_iout_filt += IIR_ALPHA * (g_iout_real - g_iout_filt);
  }
}
//���ݼ���õ���ռ�ձ�duty������TIM1ͨ��1�ıȽϼĴ���ֵ���Ӷ�����PWM���ռ�ձ�?
static void Power_ApplyDuty(float duty)
{
  const uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1U;//���ڶ�ȡ��ʱ���Զ���װ��ֵ
  uint32_t ccr = (uint32_t)(duty * (float)period);//����ռ�ձȼ���ȽϼĴ���ֵCCR

  if (ccr >= period)
  {
    ccr = period - 1U;
  }

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr);//����TIM1ͨ��1�ıȽϼĴ���ֵ������PWMռ�ձ�
}
//根据当前模式(CV或CC)使用对应PI控制器更新占空比，并执行软启�?
static void Power_ControlStep(float dt_s)
{
  /* 设定值平滑变�?: 每周期向目标靠近VOUT_SLEW_RATE_CV */
  {
    int16_t delta = (int16_t)g_vset_cv - (int16_t)g_vset_ramp;
    if (delta > (int16_t)VOUT_SLEW_RATE_CV)
    {
      g_vset_ramp += VOUT_SLEW_RATE_CV;
    }
    else if (delta < -(int16_t)VOUT_SLEW_RATE_CV)
    {
      g_vset_ramp -= VOUT_SLEW_RATE_CV;
    }
    else
    {
      g_vset_ramp = g_vset_cv;  // 直接到达，避免累积误�?
    }
  }

  const float vset = (float)g_vset_ramp * 0.01f;
  const float iset = (float)g_iset_da * 0.1f;

  if (g_mode == MODE_CV)
  {
    g_duty_cmd = PI_Update(&g_pi_cv, vset, g_vout_filt, dt_s);
  }
  else
  {
    g_duty_cmd = PI_Update(&g_pi_cc, iset, g_iout_filt, dt_s);
  }

  g_duty_cmd = clampf(g_duty_cmd, DUTY_MIN, DUTY_MAX);

  /* 软启�?: 在SOFTSTART_MS内从DUTY_MIN线�?�斜坡到PI目标 */
  if (g_softstart_active)
  {
    const uint32_t elapsed = HAL_GetTick() - g_softstart_tick;
    if (elapsed >= SOFTSTART_MS)
    {
      g_softstart_active = 0U;  // 斜坡完成
    }
    else
    {
      const float ramp = (float)elapsed / (float)SOFTSTART_MS;
      g_duty_cmd = DUTY_MIN + ramp * (g_duty_cmd - DUTY_MIN);
    }
  }

  Power_ApplyDuty(g_duty_cmd);
}

static void Uart_SendSetpoints(void)
{
  char msg[64];
  const uint16_t vset_cv = g_vset_cv;  // 显示用户设定的目标�??(平滑变化在内部对PI透明)
  const uint8_t iset_da = g_iset_da;
  const uint16_t v_int = (uint16_t)(vset_cv / 100U);
  const uint16_t v_frac = (uint16_t)(vset_cv % 100U);
  const uint8_t i_int = (uint8_t)(iset_da / 10U);
  const uint8_t i_frac = (uint8_t)(iset_da % 10U);
  const int len = snprintf(msg, sizeof(msg),
                           "Vset=%u.%02uV Iset=%u.%uA\r\n",
                           (unsigned)v_int, (unsigned)v_frac,
                           (unsigned)i_int, (unsigned)i_frac);
  if (len > 0)
  {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)msg, (uint16_t)len, 50U);
  }
}

/* ��ָ����λ����ʾһ���ı����Զ����Ǿ����� */
static void LCD_DisplayLine(uint16_t y, char *str)
{
  /* �ú�ɫ����������������ÿ��20���ظߣ� */
  LCD_FillArea(0, y, LCD_Width - 1, y + 19, LCD_BLACK);
  /* д�����ı� */
  LCD_SetColor(LCD_WHITE);
  LCD_DisplayString(0, y, str);
}

static void Oled_UpdateDisplay(void)
{
  char line0[32];
  char line1[32];
  char line2[32];
  char line3[32];

  /* ������Ϣ��ʾ��CANCEL / CONFIRM 1����Զ��˻�������? */
  if (g_display_state == DISPLAY_MSG_CANCEL)
  {
    if (HAL_GetTick() - g_msg_tick >= 1000U)
    {
      g_display_state = DISPLAY_NORMAL;
    }
  }
  else if (g_display_state == DISPLAY_MSG_CONFIRM)
  {
    if (HAL_GetTick() - g_msg_tick >= 1000U)
    {
      g_display_state = DISPLAY_NORMAL;
    }
  }

  /* �༭ģʽ����ʾ�༭���� */
  if (g_display_state == DISPLAY_EDIT)
  {
    char sv_buf[8];
    char si_buf[8];
    uint8_t i;

    /* ����SV�ַ���: xx.xxV�������벿�������֣�δ������'x' */
    for (i = 0; i < 4; i++)
    {
      if (i < g_edit_idx)
        sv_buf[i] = (char)('0' + g_edit_buf[i]);
      else
        sv_buf[i] = 'x';
    }
    sv_buf[4] = '\0';
    /* ����SI�ַ���: x.xA�������벿�������֣�δ������'x' */
    for (i = 0; i < 2; i++)
    {
      if ((i + 4) < g_edit_idx)
        si_buf[i] = (char)('0' + g_edit_buf[i + 4]);
      else
        si_buf[i] = 'x';
    }
    si_buf[2] = '\0';

    (void)snprintf(line0, sizeof(line0), "SV: %c%c.%c%cV  SI: %c.%cA",
                   sv_buf[0], sv_buf[1], sv_buf[2], sv_buf[3],
                   si_buf[0], si_buf[1]);
    (void)snprintf(line1, sizeof(line1), "D: CONFIRM");
    (void)snprintf(line2, sizeof(line2), "*: CANCEL");
    (void)snprintf(line3, sizeof(line3), "#: Redo");

    LCD_SetBackColor(LCD_BLACK);
    LCD_DisplayLine(100, line0);
    LCD_DisplayLine(120, line1);
    LCD_DisplayLine(140, line2);
    LCD_DisplayLine(160, line3);
    return;
  }

  /* ��ʾCANCEL��Ϣ */
  if (g_display_state == DISPLAY_MSG_CANCEL)
  {
    LCD_SetBackColor(LCD_BLACK);
    LCD_FillArea(0, 100, LCD_Width - 1, 179, LCD_BLACK);
    LCD_SetColor(LCD_RED);
    LCD_SetBackColor(LCD_BLACK);
    LCD_DisplayLine(130, (char *)"CANCEL");
    return;
  }

  /* ��ʾCONFIRM��Ϣ */
  if (g_display_state == DISPLAY_MSG_CONFIRM)
  {
    LCD_SetBackColor(LCD_BLACK);
    LCD_FillArea(0, 100, LCD_Width - 1, 179, LCD_BLACK);
    LCD_SetColor(LCD_GREEN);
    LCD_SetBackColor(LCD_BLACK);
    LCD_DisplayLine(130, (char *)"CONFIRM");
    return;
  }

  /* ========== 正常模式显示 ========== */
  const uint16_t vset_cv = g_vset_cv;  // 显示用户设定的目标�??(平滑变化在内部对PI透明)
  const uint8_t iset_da = g_iset_da;
  const uint16_t vset_int = (uint16_t)(vset_cv / 100U);
  const uint16_t vset_frac = (uint16_t)(vset_cv % 100U);
  const uint8_t iset_int = (uint8_t)(iset_da / 10U);
  const uint8_t iset_frac = (uint8_t)(iset_da % 10U);
  const uint16_t vout_cv = (uint16_t)(g_vout_filt * 100.0f + 0.5f);
  const uint16_t iout_da = (uint16_t)(g_iout_filt * 10.0f + 0.5f);
  const uint16_t vout_int = (uint16_t)(vout_cv / 100U);
  const uint16_t vout_frac = (uint16_t)(vout_cv % 100U);
  const uint16_t iout_int = (uint16_t)(iout_da / 10U);
  const uint16_t iout_frac = (uint16_t)(iout_da % 10U);
  const uint16_t adc_v = g_adc_raw_v;
  const uint16_t adc_i = g_adc_raw_i;
  const char mode_c0 = 'C';
  const char mode_c1 = (g_mode == MODE_CV) ? 'V' : 'C';

  (void)snprintf(line0, sizeof(line0), "%c%c SV:%02u.%02uV",
                 mode_c0, mode_c1, (unsigned)vset_int, (unsigned)vset_frac);
  (void)snprintf(line1, sizeof(line1), "SI:%u.%uA RI:%u.%uA",
                 (unsigned)iset_int, (unsigned)iset_frac,
                 (unsigned)iout_int, (unsigned)iout_frac);
  (void)snprintf(line2, sizeof(line2), "RV:%02u.%02uV", (unsigned)vout_int, (unsigned)vout_frac);
  (void)snprintf(line3, sizeof(line3), "A0:%04u A1:%04u",
                 (unsigned)adc_v, (unsigned)adc_i);

  LCD_SetBackColor(LCD_BLACK);
  LCD_DisplayLine(100, line0);
  LCD_DisplayLine(120, line1);
  LCD_DisplayLine(140, line2);
  LCD_DisplayLine(160, line3);
}
//��ȡ�����������������Ĭ��ʵ�ַ���?-1��ʾû�а��������£��û������������ļ����ض���ú�����ʵ��ʵ�ʵİ�����ȡ�߼�?
__weak int Keypad_GetKey(void)
{
  return -1;
}

//�ӱ༭�����������û����벢�����趨ֵ������Χ�޶���
static void Edit_ApplyValues(void)
{
  /* g_edit_buf: [V1,V2,Vf1,Vf2, I1,If1]
     ��ѹ: V1V2.Vf1Vf2 (��λ����+��λС��)
     ����: I1.If1 (һλ����+һλС��) */
  uint16_t new_vset = (uint16_t)(g_edit_buf[0] * 1000U + g_edit_buf[1] * 100U
                               + g_edit_buf[2] * 10U + g_edit_buf[3]);
  uint8_t new_iset = (uint8_t)(g_edit_buf[4] * 10U + g_edit_buf[5]);

  /* �޶���Χ */
  if (new_vset < VOUT_SET_MIN_CV) new_vset = VOUT_SET_MIN_CV;
  if (new_vset > VOUT_SET_MAX_CV) new_vset = VOUT_SET_MAX_CV;
  if (new_iset < IOUT_SET_MIN_DA) new_iset = IOUT_SET_MIN_DA;
  if (new_iset > IOUT_SET_MAX_DA) new_iset = IOUT_SET_MAX_DA;

  g_vset_cv = new_vset;
  g_iset_da = new_iset;
}

//����������룬���ݰ�����������ģʽ���趨ֵ�Ȳ���?
static void Power_HandleKeyboard(void)
{
  const int key = Keypad_GetKey();
  if (key < 0)
  {
    return;
  }

  if (key > 0)
  {
    HAL_GPIO_TogglePin(USER_KEY_GPIO_Port, USER_KEY_Pin);
  }

  /* �����CANCEL/CONFIRM��Ϣ��ʾ״̬���������а��� */
  if ((g_display_state == DISPLAY_MSG_CANCEL) ||
      (g_display_state == DISPLAY_MSG_CONFIRM))
  {
    return;
  }

  /* ---- �༭ģʽ�µİ������� ---- */
  if (g_display_state == DISPLAY_EDIT)
  {
    switch ((char)key)
    {
      case '0': case '1': case '2': case '3': case '4':
      case '5': case '6': case '7': case '8': case '9':
        if (g_edit_idx < 6)
        {
          g_edit_buf[g_edit_idx] = (uint8_t)((char)key - '0');
          g_edit_idx++;
        }
        break;

      case '#':   /* Redo: �������� */
        g_edit_idx = 0;
        break;

      case '*':   /* Cancel: ��ʾCANCEL���˻������� */
        g_display_state = DISPLAY_MSG_CANCEL;
        g_msg_tick = HAL_GetTick();
        break;

      case 'D':   /* Confirm: Ӧ���޸ģ���ʾCONFIRM���˻������� */
        if (g_edit_idx >= 6)
        {
          Edit_ApplyValues();
        }
        g_display_state = DISPLAY_MSG_CONFIRM;
        g_msg_tick = HAL_GetTick();
        break;

      default:
        break;
    }
    return;
  }

  /* ---- ����ģʽ���Ǳ༭���µİ������� ---- */
  switch ((char)key)
  {
    case 'A':
      g_mode = MODE_CV;
      PI_Reset(&g_pi_cv, g_duty_cmd);
      break;

    case 'B':
      g_mode = MODE_CC;
      PI_Reset(&g_pi_cc, g_duty_cmd);
      break;

    case 'C':   /* ����༭ģ�? */
      g_display_state = DISPLAY_EDIT;
      g_edit_idx = 0;
      break;

    case '2':
    case '+':
      if (g_mode == MODE_CV)
      {
        if (g_vset_cv <= (VOUT_SET_MAX_CV - VOUT_SET_STEP_CV))
        {
          g_vset_cv += VOUT_SET_STEP_CV;
        }
      }
      else
      {
        if (g_iset_da <= (IOUT_SET_MAX_DA - IOUT_SET_STEP_DA))
        {
          g_iset_da += IOUT_SET_STEP_DA;
        }
      }
      break;

    case '8':
    case '-':
      if (g_mode == MODE_CV)
      {
        if (g_vset_cv >= (VOUT_SET_MIN_CV + VOUT_SET_STEP_CV))
        {
          g_vset_cv -= VOUT_SET_STEP_CV;
        }
      }
      else
      {
        if (g_iset_da >= (IOUT_SET_MIN_DA + IOUT_SET_STEP_DA))
        {
          g_iset_da -= IOUT_SET_STEP_DA;
        }
      }
      break;

    default:
      break;
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  Keypad_Init();
  SPI_LCD_Init();

  /* LCD���ԣ���ʾ Hello World */
  LCD_SetBackColor(LCD_BLACK);
  LCD_Clear();
  LCD_SetColor(LCD_RED);
  LCD_DisplayString(0, 0, (char *)"Hello World!");
  LCD_SetColor(LCD_GREEN);
  LCD_DisplayString(0, 20, (char *)"LCD Test OK!");
  LCD_SetColor(LCD_BLUE);
  LCD_DisplayString(0, 40, (char *)"SPI1 PA5 PA7");
  LCD_SetColor(LCD_WHITE);
  LCD_DisplayString(0, 60, (char *)"PB0 BL PB1 DC");
  HAL_Delay(1000);

  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)g_adc_dma, 2U) != HAL_OK)
  {
    Error_Handler();
  }

  /* 【安全启动�?�PWM启动前先写入�?小占空比 */
  g_duty_cmd = DUTY_MIN;
  Power_ApplyDuty(g_duty_cmd);
  // 强制UEV将影子CCR(2%)传输到活动寄存器，防止第�?个PWM周期使用初始化�??50%
  TIM1->EGR = TIM_EGR_UG;
  PI_Reset(&g_pi_cv, DUTY_MIN);
  PI_Reset(&g_pi_cc, DUTY_MIN);

  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  /* 启动ADC后进行软启动 */
  g_softstart_active = 1U;
  g_softstart_tick = HAL_GetTick();
  g_vset_ramp = g_vset_cv;  // 初始化设定�?�陡坡与目标�?�?

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    static uint32_t last_tick = 0U;
    static uint32_t last_oled_tick = 0U;
    const uint32_t now = HAL_GetTick();

    if ((now - last_tick) >= CTRL_PERIOD_MS)
    {
      last_tick = now;

      Power_HandleKeyboard();
      Power_ReadFeedback();
      Power_ControlStep(CTRL_PERIOD_S);
    }

    if ((now - last_oled_tick) >= 200U)
    {
      last_oled_tick = now;
      Oled_UpdateDisplay();
    }

    if (g_uart_send_pending != 0U)
    {
      g_uart_send_pending = 0U;
      Uart_SendSetpoints();
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_32;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_5;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1699;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 850;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIMEx_EnableDeadTimePreload(&htim1);
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 24;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 17000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USER_KEY_GPIO_Port, USER_KEY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_BL_Pin|LCD_DC_Pin|LCD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, ROW1_Pin|ROW2_Pin|ROW3_Pin|ROW4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_KEY_Pin */
  GPIO_InitStruct.Pin = USER_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USER_KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_BL_Pin LCD_DC_Pin LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_BL_Pin|LCD_DC_Pin|LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ROW1_Pin ROW2_Pin ROW3_Pin ROW4_Pin */
  GPIO_InitStruct.Pin = ROW1_Pin|ROW2_Pin|ROW3_Pin|ROW4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : COL1_Pin COL2_Pin COL3_Pin COL4_Pin */
  GPIO_InitStruct.Pin = COL1_Pin|COL2_Pin|COL3_Pin|COL4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    g_uart_send_pending = 1U;
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

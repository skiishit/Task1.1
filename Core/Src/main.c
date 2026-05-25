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
#include "codetab.h"
#include "oled.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define CTRL_PERIOD_MS               (1U)  //控制循环周期，ms
#define CTRL_PERIOD_S                (0.001f)//用秒表示控制循环周期，0.001s
//PWM最小/最大占空比
#define DUTY_MIN                     (0.02f)
#define DUTY_MAX                     (0.95f)
//电压设定上下限和步长
#define VOUT_SET_MIN_CV              (200U)   /* 2.00V  */
#define VOUT_SET_MAX_CV              (3300U)  /* 33.00V */
#define VOUT_SET_STEP_CV             (2U)     /* 0.02V  */
//电流设定上下限和步长
#define IOUT_SET_MIN_DA              (1U)     /* 0.1A */
#define IOUT_SET_MAX_DA              (20U)    /* 2.0A */
#define IOUT_SET_STEP_DA             (1U)     /* 0.1A */
//ADC满刻度
#define ADC_FULL_SCALE               (4095.0f)
#define ADC_VREF                     (3.3f)
//？？
/* 按实物标定修改: Vout = Vadc * VOLTAGE_SCALE */
#define VOLTAGE_SCALE                (10.0f)
/* 按实物标定修改: Iout = (Vadcx - CURRENT_OFFSET_V) * CURRENT_SCALE_A_PER_V */
#define CURRENT_SCALE_A_PER_V        (1.0f)
#define CURRENT_OFFSET_V             (0.0f)

#define IIR_ALPHA                    (0.2f)

#define CV_KP                        (0.045f)
#define CV_KI                        (6.0f)
#define CC_KP                        (0.08f)
#define CC_KI                        (8.0f)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

typedef enum
{
  MODE_CV = 0,
  MODE_CC = 1
} control_mode_t;  //控制模式：恒压（CV）或恒流（CC）

typedef struct
{
  float kp;
  float ki;
  float integral;
  float out_min;
  float out_max;
} pi_ctrl_t;  //PI控制器参数和状态？？

static control_mode_t g_mode = MODE_CV;  //当前控制模式

//电压目标量和电流目标量，单位分别是0.01V和0.1A，初始值分别是5.00V和1.0A
static uint16_t g_vset_cv = 500U; /* 5.00V */
static uint8_t g_iset_da = 10U;   /* 1.0A */
//输出电压和电流的测量值和滤波后的值
static float g_vout_real = 0.0f;
static float g_iout_real = 0.0f;
static float g_vout_filt = 0.0f;
static float g_iout_filt = 0.0f;
//当前目标占空比，初始值为0.5（50%）
static float g_duty_cmd = 0.5f;
//ADC原始读数
static uint16_t g_adc_raw_v = 0U;
static uint16_t g_adc_raw_i = 0U;
//CV，CC控制器初始化
static pi_ctrl_t g_pi_cv = {CV_KP, CV_KI, 0.0f, DUTY_MIN, DUTY_MAX};
static pi_ctrl_t g_pi_cc = {CC_KP, CC_KI, 0.0f, DUTY_MIN, DUTY_MAX};
static volatile uint8_t g_uart_send_pending = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
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
static void Oled_WriteLine(uint8_t page, const char *text);
__weak int Keypad_GetKey(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//？？
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
//PI控制器重置函数，将pi->integral设置为预加载值preload，防止模式切换时积分项引起的突变
static void PI_Reset(pi_ctrl_t *pi, float preload)
{
  pi->integral = preload;
}
//PI控制器更新函数，根据设定值setpoint、反馈值feedback和时间步长dt_s计算新的控制输出
static float PI_Update(pi_ctrl_t *pi, float setpoint, float feedback, float dt_s)
{
  const float err = setpoint - feedback;  //误差
  const float p_term = pi->kp * err;  //比例项
  float out = p_term + pi->integral;  //临时输出

  if ((out > pi->out_min) && (out < pi->out_max))
  {
    pi->integral += pi->ki * err * dt_s;  //仅当输出未饱和时才更新积分项，防止积分饱和
  }
  //更新输出out
  pi->integral = clampf(pi->integral, pi->out_min, pi->out_max);
  out = p_term + pi->integral;
  out = clampf(out, pi->out_min, pi->out_max);
  return out;
}
//
static void Power_ReadFeedback(void)
{
  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    return;
  }
//轮询等待ADC转换完成，读取电压和电流的原始ADC值，存储在g_adc_raw_v和g_adc_raw_i中
  if (HAL_ADC_PollForConversion(&hadc1, 5U) != HAL_OK)
  {
    (void)HAL_ADC_Stop(&hadc1);
    return;
  }
  g_adc_raw_v = (uint16_t)HAL_ADC_GetValue(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 5U) != HAL_OK)
  {
    (void)HAL_ADC_Stop(&hadc1);
    return;
  }
  g_adc_raw_i = (uint16_t)HAL_ADC_GetValue(&hadc1);

  (void)HAL_ADC_Stop(&hadc1);
//根据ADC原始值计算实际电压和电流，并进行简单的IIR滤波，更新g_v/iout_real/filt
  {
    const float vadc_v = ((float)g_adc_raw_v / ADC_FULL_SCALE) * ADC_VREF;
    const float vadc_i = ((float)g_adc_raw_i / ADC_FULL_SCALE) * ADC_VREF;

    g_vout_real = vadc_v * VOLTAGE_SCALE;
    g_iout_real = (vadc_i - CURRENT_OFFSET_V) * CURRENT_SCALE_A_PER_V;
    if (g_iout_real < 0.0f)
    {
      g_iout_real = 0.0f;
    }

    g_vout_filt += IIR_ALPHA * (g_vout_real - g_vout_filt);
    g_iout_filt += IIR_ALPHA * (g_iout_real - g_iout_filt);
  }
}
//根据计算得到的占空比duty，更新TIM1通道1的比较寄存器值，从而调整PWM输出占空比
static void Power_ApplyDuty(float duty)
{
  const uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1U;//用于读取定时器自动重装载值
  uint32_t ccr = (uint32_t)(duty * (float)period);//根据占空比计算比较寄存器值CCR

  if (ccr >= period)
  {
    ccr = period - 1U;
  }

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr);//设置TIM1通道1的比较寄存器值，更新PWM占空比
}
//根据当前控制模式（CV或CC），使用对应的PI控制器计算新的占空比命令，并应用到PWM输出
static void Power_ControlStep(float dt_s)
{
  const float vset = (float)g_vset_cv * 0.01f;
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
  Power_ApplyDuty(g_duty_cmd);
}

static void Uart_SendSetpoints(void)
{
  char msg[64];
  const uint16_t vset_cv = g_vset_cv;
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

static void Oled_UpdateDisplay(void)
{
  char line0[17];
  char line1[17];
  char line2[17];
  char line3[17];
  const uint16_t vset_cv = g_vset_cv;
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

  (void)snprintf(line0, sizeof(line0), "SV:%02u.%02uV", (unsigned)vset_int, (unsigned)vset_frac);
  (void)snprintf(line1, sizeof(line1), "SI:%u.%uA", (unsigned)iset_int, (unsigned)iset_frac);
  (void)snprintf(line2, sizeof(line2), "RV:%02u.%02uV", (unsigned)vout_int, (unsigned)vout_frac);
  (void)snprintf(line3, sizeof(line3), "RI:%u.%uA", (unsigned)iout_int, (unsigned)iout_frac);

  Oled_WriteLine(0U, line0);
  Oled_WriteLine(2U, line1);
  Oled_WriteLine(4U, line2);
  Oled_WriteLine(6U, line3);
}

static void Oled_WriteLine(uint8_t page, const char *text)
{
  char buf[17];
  uint8_t i = 0U;

  for (i = 0U; i < 16U; ++i)
  {
    if ((text != NULL) && (text[i] != '\0'))
    {
      buf[i] = text[i];
    }
    else
    {
      buf[i] = ' ';
    }
  }
  buf[16] = '\0';
  OLED_ShowString(0U, page, buf);
}
//获取按键输入的弱函数，默认实现返回-1表示没有按键被按下，用户可以在其他文件中重定义该函数以实现实际的按键读取逻辑
__weak int Keypad_GetKey(void)
{
  return -1;
}
//处理键盘输入，根据按键调整控制模式、设定值等参数
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_I2C3_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  Keypad_Init();
  OLED_Init();
  OLED_CLS();
  OLED_ShowStr(0, 0, (unsigned char *)"OLED Init OK", 2);

  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }

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

  PI_Reset(&g_pi_cv, g_duty_cmd);
  PI_Reset(&g_pi_cc, g_duty_cmd);
  Power_ApplyDuty(g_duty_cmd);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C3
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x0010061A;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  sBreakDeadTimeConfig.DeadTime = 9;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USER_KEY_GPIO_Port, USER_KEY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, ROW1_Pin|ROW2_Pin|ROW3_Pin|ROW4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_KEY_Pin */
  GPIO_InitStruct.Pin = USER_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USER_KEY_GPIO_Port, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

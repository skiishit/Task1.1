/**
  ******************************************************************************
  * @file    lcd_driver.c
  * @brief   LCD驱动代码 (适配 PB0-BL, PB1-DC, PB2-CS, PA5-SCK, PA7-MOSI)
  ******************************************************************************
  */

#include "lcd_driver.h"
#include <stdio.h>
#include <string.h>

extern SPI_HandleTypeDef hspi1;
#define  LCD_SPI hspi1

static pFONT *LCD_AsciiFonts;		// 英文字体

// 缓冲区，用于显示字符
static uint16_t LCD_Buff[1024];

// LCD相关参数结构体
struct
{
	uint32_t Color;  			// LCD当前画笔颜色
	uint32_t BackColor;		// 背景色
	uint8_t  ShowNum_Mode;	// 数字显示模式
	uint8_t  Direction;		// 显示方向
	uint16_t Width;          // 屏幕像素长度
	uint16_t Height;         // 屏幕像素宽度	
	uint8_t  X_Offset;       // X坐标偏移
	uint8_t  Y_Offset;       // Y坐标偏移
} LCD;

/*******************************************************************************
 * SPI读写函数
 ******************************************************************************/
static uint8_t SPI_WriteData(uint8_t *data, uint16_t size)
{
	return HAL_SPI_Transmit(&LCD_SPI, data, size, 1000);
}

static void LCD_SPI_Send(uint8_t *data, uint16_t size)
{
	SPI_WriteData(data, size);
}

void LCD_WriteCommand(uint8_t lcd_command)
{
	LCD_DC_Command;
	LCD_SPI_Send(&lcd_command, 1);
	LCD_DC_Data;
}

void LCD_WriteData_8bit(uint8_t lcd_data)
{
	LCD_DC_Data;
	LCD_SPI_Send(&lcd_data, 1);
}

void LCD_WriteData_16bit(uint16_t lcd_data)
{
	uint8_t data[2] = {0};
	data[0] = lcd_data >> 8;
	data[1] = lcd_data;
	LCD_DC_Data;
	LCD_SPI_Send(data, 2);
}

void LCD_WriteBuff(uint16_t *DataBuff, uint16_t DataSize)
{
	uint32_t i;
	LCD_CS_L;
	for(i = 0; i < DataSize; i++)
	{
		LCD_WriteData_16bit(DataBuff[i]);
	}
	LCD_CS_H;
}

/*******************************************************************************
 * LCD初始化
 ******************************************************************************/
void SPI_LCD_Init(void)
{
	HAL_Delay(10);
	
	LCD_SetColor(LCD_WHITE);
	LCD_CS_L;
	
	LCD_WriteCommand(0x36);       // 显存访问控制
	LCD_WriteData_8bit(0x00);
	
	LCD_WriteCommand(0x3A);			// 接口像素格式
	LCD_WriteData_8bit(0x05);     // 16位色
	
	LCD_WriteCommand(0xB2);			
	LCD_WriteData_8bit(0x0C);
	LCD_WriteData_8bit(0x0C); 
	LCD_WriteData_8bit(0x00); 
	LCD_WriteData_8bit(0x33); 
	LCD_WriteData_8bit(0x33); 			
	
	LCD_WriteCommand(0xB7);		   // 栅极电压设置
	LCD_WriteData_8bit(0x35);
	
	LCD_WriteCommand(0xBB);			// 公共电压设置
	LCD_WriteData_8bit(0x19);
	
	LCD_WriteCommand(0xC0);
	LCD_WriteData_8bit(0x2C);
	
	LCD_WriteCommand(0xC2);
	LCD_WriteData_8bit(0x01);
	
	LCD_WriteCommand(0xC3);			// VRH电压
	LCD_WriteData_8bit(0x12);
	
	LCD_WriteCommand(0xC4);		   // VDV电压
	LCD_WriteData_8bit(0x20);
	
	LCD_WriteCommand(0xC6); 		// 帧率控制
	LCD_WriteData_8bit(0x0F);
	
	LCD_WriteCommand(0xD0);			// 电源控制
	LCD_WriteData_8bit(0xA4);
	LCD_WriteData_8bit(0xA1);
	
	LCD_WriteCommand(0xE0);       // 正极电压伽马值
	LCD_WriteData_8bit(0xD0);
	LCD_WriteData_8bit(0x04);
	LCD_WriteData_8bit(0x0D);
	LCD_WriteData_8bit(0x11);
	LCD_WriteData_8bit(0x13);
	LCD_WriteData_8bit(0x2B);
	LCD_WriteData_8bit(0x3F);
	LCD_WriteData_8bit(0x54);
	LCD_WriteData_8bit(0x4C);
	LCD_WriteData_8bit(0x18);
	LCD_WriteData_8bit(0x0D);
	LCD_WriteData_8bit(0x0B);
	LCD_WriteData_8bit(0x1F);
	LCD_WriteData_8bit(0x23);
	
	LCD_WriteCommand(0xE1);      // 负极电压伽马值
	LCD_WriteData_8bit(0xD0);
	LCD_WriteData_8bit(0x04);
	LCD_WriteData_8bit(0x0C);
	LCD_WriteData_8bit(0x11);
	LCD_WriteData_8bit(0x13);
	LCD_WriteData_8bit(0x2C);
	LCD_WriteData_8bit(0x3F);
	LCD_WriteData_8bit(0x44);
	LCD_WriteData_8bit(0x51);
	LCD_WriteData_8bit(0x2F);
	LCD_WriteData_8bit(0x1F);
	LCD_WriteData_8bit(0x1F);
	LCD_WriteData_8bit(0x20);
	LCD_WriteData_8bit(0x23);
	
	LCD_WriteCommand(0x21);       // 打开反显
	LCD_WriteCommand(0x11);       // 退出休眠
	HAL_Delay(120);
	
	LCD_WriteCommand(0x29);       // 打开显示
	
	while((LCD_SPI.Instance->SR & 0x0080) != RESET);
	LCD_CS_H;
	
	// 默认设置
	LCD_SetDirection(Direction_V);
	LCD_SetBackColor(LCD_BLACK);
	LCD_SetColor(LCD_WHITE);
	LCD_Clear();
	
	LCD_SetAsciiFont(&ASCII_Font16);   // 设置1608字体
	LCD_ShowNumMode(0);
	
	LCD_Backlight_ON;  // 打开背光
}

/*******************************************************************************
 * 坐标设置
 ******************************************************************************/
void LCD_SetAddress(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	LCD_CS_L;
	
	LCD_WriteCommand(0x2a);			// 列地址设置
	LCD_WriteData_16bit(x1 + LCD.X_Offset);
	LCD_WriteData_16bit(x2 + LCD.X_Offset);
	
	LCD_WriteCommand(0x2b);			// 行地址设置
	LCD_WriteData_16bit(y1 + LCD.Y_Offset);
	LCD_WriteData_16bit(y2 + LCD.Y_Offset);
	
	LCD_WriteCommand(0x2c);			// 开始写入显存
	
	while((LCD_SPI.Instance->SR & 0x0080) != RESET);
	LCD_CS_H;
}

/*******************************************************************************
 * 颜色设置
 ******************************************************************************/
void LCD_SetColor(uint32_t Color)
{
	uint16_t Red_Value, Green_Value, Blue_Value;
	Red_Value   = (uint16_t)((Color & 0x00F80000) >> 8);
	Green_Value = (uint16_t)((Color & 0x0000FC00) >> 5);
	Blue_Value  = (uint16_t)((Color & 0x000000F8) >> 3);
	LCD.Color = (uint16_t)(Red_Value | Green_Value | Blue_Value);
}

void LCD_SetBackColor(uint32_t Color)
{
	uint16_t Red_Value, Green_Value, Blue_Value;
	Red_Value   = (uint16_t)((Color & 0x00F80000) >> 8);
	Green_Value = (uint16_t)((Color & 0x0000FC00) >> 5);
	Blue_Value  = (uint16_t)((Color & 0x000000F8) >> 3);
	LCD.BackColor = (uint16_t)(Red_Value | Green_Value | Blue_Value);
}

/*******************************************************************************
 * 显示方向设置
 ******************************************************************************/
void LCD_SetDirection(uint8_t direction)
{
	LCD.Direction = direction;
	LCD_CS_L;
	
	if(direction == Direction_H)
	{
		LCD_WriteCommand(0x36);
		LCD_WriteData_8bit(0x70);
		LCD.X_Offset = 20;
		LCD.Y_Offset = 0;
		LCD.Width  = LCD_Height;
		LCD.Height = LCD_Width;
	}
	else if(direction == Direction_V)
	{
		LCD_WriteCommand(0x36);
		LCD_WriteData_8bit(0x00);
		LCD.X_Offset = 0;
		LCD.Y_Offset = 20;
		LCD.Width  = LCD_Width;
		LCD.Height = LCD_Height;
	}
	else if(direction == Direction_H_Flip)
	{
		LCD_WriteCommand(0x36);
		LCD_WriteData_8bit(0xA0);
		LCD.X_Offset = 20;
		LCD.Y_Offset = 0;
		LCD.Width  = LCD_Height;
		LCD.Height = LCD_Width;
	}
	else if(direction == Direction_V_Flip)
	{
		LCD_WriteCommand(0x36);
		LCD_WriteData_8bit(0xC0);
		LCD.X_Offset = 0;
		LCD.Y_Offset = 20;
		LCD.Width  = LCD_Width;
		LCD.Height = LCD_Height;
	}
	
	while((LCD_SPI.Instance->SR & 0x0080) != RESET);
	LCD_CS_H;
}

/*******************************************************************************
 * 字体设置
 ******************************************************************************/
void LCD_SetAsciiFont(pFONT *Asciifonts)
{
	LCD_AsciiFonts = Asciifonts;
}

void LCD_ShowNumMode(uint8_t mode)
{
	LCD.ShowNum_Mode = mode;
}

/*******************************************************************************
 * 清屏
 ******************************************************************************/
void LCD_Clear(void)
{
	uint32_t i;
	LCD_SetAddress(0, 0, LCD.Width - 1, LCD.Height - 1);
	LCD_CS_L;
	for(i = 0; i < LCD.Width * LCD.Height; i++)
	{
		LCD_WriteData_16bit(LCD.BackColor);
	}
	LCD_CS_H;
}

/*******************************************************************************
 * 填充矩形区域（消除闪烁用，仅填充指定区域）
 ******************************************************************************/
void LCD_FillArea(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint32_t color)
{
	uint32_t old_color = LCD.Color;
	uint16_t old_back = LCD.BackColor;
	uint32_t i, total;
	
	LCD_SetColor(color);
	LCD_SetAddress(x1, y1, x2, y2);
	total = (uint32_t)(x2 - x1 + 1) * (uint32_t)(y2 - y1 + 1);
	LCD_CS_L;
	for(i = 0; i < total; i++)
	{
		LCD_WriteData_16bit(LCD.Color);
	}
	LCD_CS_H;
	
	LCD.Color = old_color;
	LCD.BackColor = old_back;
}

/*******************************************************************************
 * 画点
 ******************************************************************************/
void LCD_DrawPoint(uint16_t x, uint16_t y, uint32_t color)
{
	uint32_t old_color = LCD.Color;
	LCD_SetColor(color);
	LCD_SetAddress(x, y, x, y);
	LCD_CS_L;
	LCD_WriteData_16bit(LCD.Color);
	while((LCD_SPI.Instance->SR & 0x0080) != RESET);
	LCD_CS_H;
	LCD.Color = old_color;
}

/*******************************************************************************
 * 显示单个ASCII字符
 ******************************************************************************/
void LCD_DisplayChar(uint16_t x, uint16_t y, uint8_t c)
{
	uint16_t index, counter, i = 0, w = 0;
	uint8_t disChar;
	
	c = c - 32; // 计算ASCII偏移
	
	LCD_CS_L;
	
	for(index = 0; index < LCD_AsciiFonts->Sizes; index++)
	{
		disChar = LCD_AsciiFonts->pTable[c * LCD_AsciiFonts->Sizes + index];
		for(counter = 0; counter < 8; counter++)
		{
			if(disChar & 0x01)
			{
				LCD_Buff[i] = LCD.Color;
			}
			else
			{
				LCD_Buff[i] = LCD.BackColor;
			}
			disChar >>= 1;
			i++;
			w++;
			if(w == LCD_AsciiFonts->Width)
			{
				w = 0;
				break;
			}
		}
	}
	LCD_SetAddress(x, y, x + LCD_AsciiFonts->Width - 1, y + LCD_AsciiFonts->Height - 1);
	LCD_WriteBuff(LCD_Buff, LCD_AsciiFonts->Width * LCD_AsciiFonts->Height);
}

/*******************************************************************************
 * 显示ASCII字符串
 ******************************************************************************/
void LCD_DisplayString(uint16_t x, uint16_t y, char *p)
{
	while((x < LCD.Width) && (*p != 0))
	{
		LCD_DisplayChar(x, y, *p);
		x += LCD_AsciiFonts->Width;
		p++;
	}
}

/*******************************************************************************
 * 显示整数
 ******************************************************************************/
void LCD_DisplayNumber(uint16_t x, uint16_t y, int32_t number, uint8_t len)
{
	char Number_Buffer[15];
	
	if(LCD.ShowNum_Mode == 0) // 多余位补0
	{
		sprintf(Number_Buffer, "%0.*d", len, number);
	}
	else // 多余位补空格
	{
		sprintf(Number_Buffer, "%*d", len, number);
	}
	
	LCD_DisplayString(x, y, (char *)Number_Buffer);
}

/*******************************************************************************
 * 画线 (Bresenham算法)
 ******************************************************************************/
#define ABS(X)  ((X) > 0 ? (X) : -(X))

void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
			yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
			curpixel = 0;
	
	deltax = ABS(x2 - x1);
	deltay = ABS(y2 - y1);
	x = x1;
	y = y1;
	
	if(x2 >= x1) { xinc1 = 1; xinc2 = 1; }
	else { xinc1 = -1; xinc2 = -1; }
	
	if(y2 >= y1) { yinc1 = 1; yinc2 = 1; }
	else { yinc1 = -1; yinc2 = -1; }
	
	if(deltax >= deltay)
	{
		xinc1 = 0;
		yinc2 = 0;
		den = deltax;
		num = deltax / 2;
		numpixels = deltax;
		for(curpixel = 0; curpixel <= numpixels; curpixel++)
		{
			LCD_DrawPoint(x, y, LCD.Color);
			num += deltay;
			if(num >= den)
			{
				num -= den;
				x += xinc1;
				y += yinc1;
			}
			x += xinc2;
			y += yinc2;
		}
	}
	else
	{
		xinc2 = 0;
		yinc1 = 0;
		den = deltay;
		num = deltay / 2;
		numpixels = deltay;
		for(curpixel = 0; curpixel <= numpixels; curpixel++)
		{
			LCD_DrawPoint(x, y, LCD.Color);
			num += deltax;
			if(num >= den)
			{
				num -= den;
				x += xinc1;
				y += yinc1;
			}
			x += xinc2;
			y += yinc2;
		}
	}
}

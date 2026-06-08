#ifndef __LCD_DRIVER_H
#define __LCD_DRIVER_H

#include "main.h"
#include "lcd_fonts.h"

/*----------------------------------------------- 参数宏 -------------------------------------------*/

#define LCD_Width     240		// LCD的像素长度
#define LCD_Height    280		// LCD的像素宽度

// 显示方向参数
#define	Direction_H				0					// LCD横屏显示
#define	Direction_H_Flip	   1					// LCD横屏显示,上下翻转
#define	Direction_V				2					// LCD竖屏显示 
#define	Direction_V_Flip	   3					// LCD竖屏显示,上下翻转 

/*--------------------------------------------------------- 控制宏 ---------------------------------------------------*/

// 片选控制 (PB2)
#define 	LCD_CS_H    		 GPIOB->BSRR = LCD_CS_Pin						// 片选拉高
#define 	LCD_CS_L     		 GPIOB->BSRR = (uint32_t)LCD_CS_Pin << 16U	// 片选拉低

// 数据/命令控制 (PB1)
#define	LCD_DC_Command		   HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET)	// 低电平，指令传输 
#define 	LCD_DC_Data		      HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET)	// 高电平，数据传输

// 背光控制 (PB0)
#define 	LCD_Backlight_ON      HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_SET)	// 高电平，开启背光
#define 	LCD_Backlight_OFF  	 HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_RESET)	// 低电平，关闭背光

/*---------------------------------------- 常用颜色 ------------------------------------------------------*/
#define 	LCD_WHITE       0xFFFFFF	 // 纯白色
#define 	LCD_BLACK       0x000000    // 纯黑色
#define 	LCD_BLUE        0x0000FF	 //	纯蓝色
#define 	LCD_GREEN       0x00FF00    //	纯绿色
#define 	LCD_RED         0xFF0000    //	纯红色

/*------------------------------------------------ 函数声明 ----------------------------------------------*/

void  SPI_LCD_Init(void);           // 液晶屏以及SPI初始化   
void  LCD_Clear(void);			       // 清屏函数
void  LCD_FillArea(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint32_t color);  // 用指定颜色填充矩形区域
void  LCD_SetAddress(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);	// 设置坐标		
void  LCD_SetColor(uint32_t Color); // 设置画笔颜色
void  LCD_SetBackColor(uint32_t Color);  // 设置背景颜色
void  LCD_SetDirection(uint8_t direction);  // 设置显示方向
void  LCD_DisplayChar(uint16_t x, uint16_t y, uint8_t c);	// 显示单个ASCII字符
void  LCD_DisplayString(uint16_t x, uint16_t y, char *p); // 显示ASCII字符串
void  LCD_SetAsciiFont(pFONT *fonts);	// 设置ASCII字体
void  LCD_ShowNumMode(uint8_t mode);  // 设置变量显示模式
void  LCD_DisplayNumber(uint16_t x, uint16_t y, int32_t number, uint8_t len); // 显示整数
void  LCD_DrawPoint(uint16_t x, uint16_t y, uint32_t color); // 画点
void  LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2); // 画线

#endif

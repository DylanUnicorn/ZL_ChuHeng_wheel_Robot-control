/**
 * ************************************************************************
 *
 * @file OLED_Function.h
 * @author
 * @brief OLED功能函数驱动头文件
 *
 * ************************************************************************
 * @copyright Copyright (c) 2024
 * ************************************************************************
 */
#ifndef _OLED_FUNCTION_H_
#define _OLED_FUNCTION_H_

#include "OLED_IIC_Config.h"

//字符串显示函数
void OLED_ShowStr(signed short int x, signed short int y, unsigned char ch[], unsigned char TextSize);
//中文汉字显示函数
void OLED_ShowChinese(signed short int x, signed short int y, unsigned char* ch);
//BMP图片显示函数
void OLED_ShowBMP(signed short int x0,signed short int y0,signed short int L,signed short int H,const unsigned char BMP[]);

#endif /* _OLED_FUNCTION_H_ */


#ifndef __OLED_H_
#define __OLED_H_

#include "main.h"

#define BIG_FONT_EN 16
#define SMALL_FONT_EN 8

void WriteCmd(void);
//向设备写控制命令
void OLED_WR_CMD(uint8_t cmd);
//向设备写数据
void OLED_WR_DATA(uint8_t data);
//初始化oled屏幕
void OLED_Init(void);
//清屏
void OLED_Clear(void);
//清行
void OLED_Clearrow(uint8_t i);
//开启OLED显示
void OLED_Display_On(void);
//关闭OLED显示
void OLED_Display_Off(void);
//设置光标
void OLED_Set_Pos(uint8_t x, uint8_t y);

void OLED_On(void);

//在指定位置显示一个字符
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t Char_Size);

//显示数字
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len,
                  uint8_t size2);

//显示一个字符号串
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t Char_Size);

//显示汉字
// hzk 用取模软件得出的数组
void OLED_ShowCHinese(uint8_t x, uint8_t y, uint8_t no);

#endif

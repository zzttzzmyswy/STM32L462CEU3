#ifndef __CRC16_H
#define __CRC16_H
#include "main.h"

uint16_t App_Tab_Get_CRC16( uint8_t * pucFrame, uint16_t usLen );              /* 查表计算得CRC */
uint16_t App_Calc_Get_CRC16( uint8_t *ptr,uint8_t len);                        /* 直接运算得CRC */

#endif


#ifndef __MODBUS_H
#define __MODBUS_H
#include "main.h"

HAL_StatusTypeDef Master_Rx_Data(uint8_t *pData, uint32_t dSize,
                                 enum Data_Type Data_Type_e);
void Master_Send(int add, uint16_t reg_add, uint16_t dSize);

#endif

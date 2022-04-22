#include "modbus.h"
#include "crc16.h"

extern UART_HandleTypeDef huart2;
extern float value_Data[4];
extern struct struct_data_from_r data_from_r[8];
extern char *Instantaneous_flow_unit_name[12];
extern char *Forward_flow_unit_name[4];
extern volatile uint32_t modbus_data_flag;

union FloatFig
{
    unsigned char FCB[4];
    float BCF;
};

/*
 * MODBUS��ʽ
 * [��ַ][������][��ʼ��ַ��][��ʼ��ַ��][�ܼĴ�������][�ܼĴ�������][CRC��][CRC��]
 * add : �ӻ���ַ
 * reg_add : ��Ҫ��ȡ�Ĵ����ĵ�ַ
 * dSize : ��Ҫ��ȡ�Ĵ�������Ŀ��һ���Ĵ���16λ�������ֽ�
 * ��������RS485����
 */
void Master_Send(int add, uint16_t reg_add, uint16_t dSize)
{
    static uint16_t CRC16Temp = 0;
    static uint8_t SendBuf[8];
    SendBuf[0] = add;                          //��ʼ��ַ
    SendBuf[1] = 0x04;                         //������
    SendBuf[2] = reg_add >> 8;                 //��ַ��λ
    SendBuf[3] = reg_add & 0xff;               //��ַ��λ
    SendBuf[4] = dSize >> 8;                   //�Ĵ���������λ
    SendBuf[5] = dSize & 0xff;                 //�Ĵ���������λ
    CRC16Temp = App_Tab_Get_CRC16(SendBuf, 6); //��ȡCRCУ��ֵ
    SendBuf[6] = CRC16Temp & 0xFF;             // CRC��λ ��λ����
    SendBuf[7] = (CRC16Temp >> 8);             // CRC��λ
    RS485_Set_TE();
    HAL_Delay(1);
    HAL_UART_Transmit(&huart2, SendBuf, 8, 0xffff);
    HAL_Delay(2);
    RS485_Set_RE();
}

/*
 * MODBUS��ʽ [��ַ][������][���ݳ���][����][CRC��][CRC��]
 * ö���� Data_Type_e:
 *      Data_Type_Instantaneous, ���ݳ���Ӧ����4
 *      Data_Type_Forward_flow, ���ݳ���Ӧ����8
 *      Data_Type_Instantaneous_flow_unit, ���ݳ���Ӧ����2
 *      Data_Type_Forward_flow_unit ���ݳ���Ӧ����2
 */
HAL_StatusTypeDef Master_Rx_Data(uint8_t *pData, uint32_t dSize,
                                 enum Data_Type Data_Type_e)
{
    uint16_t CRC16 = 0, CRC16Temp = 0;
    union FloatFig FloatChange;
    modbus_data_flag = 1;
    if (pData[0] > 8 && pData[0] < 1)
        return HAL_ERROR;
    if (pData[1] != 0x04)
        return HAL_ERROR;
    CRC16 = App_Tab_Get_CRC16(pData, dSize - 2);
    CRC16Temp = ((uint16_t)(pData[dSize - 1] << 8) | pData[dSize - 2]);
    /* CRCУ�� */
    if (CRC16 != CRC16Temp)
        return HAL_ERROR;
    switch (Data_Type_e)
    {
    case Data_Type_Instantaneous:
        /* ˲ʱ���� 4�ֽڴ�С float�� */
        if (((uint16_t)(pData[2] << 8 | pData[3])) != 0x04)
            return HAL_ERROR;
        FloatChange.FCB[3] = pData[4];
        FloatChange.FCB[2] = pData[5];
        FloatChange.FCB[1] = pData[6];
        FloatChange.FCB[0] = pData[7];
        data_from_r[pData[0] - 1].Instantaneous_flow = FloatChange.BCF;
        break;
    case Data_Type_Forward_flow: 
        /* �����ۼ����� 8�ֽڴ�С long+float�� */
        if (((uint16_t)(pData[2] << 8 | pData[3])) != 0x08)
            return HAL_ERROR;
        data_from_r[pData[0] - 1].Forward_flow_long =
            (uint32_t)(pData[4] << 24 | pData[5] << 16 | pData[6] << 8 |
                       pData[7]);
        FloatChange.FCB[3] = pData[8];
        FloatChange.FCB[2] = pData[9];
        FloatChange.FCB[1] = pData[10];
        FloatChange.FCB[0] = pData[11];
        data_from_r[pData[0] - 1].Forward_flow_float = FloatChange.BCF;
        break;
    case Data_Type_Instantaneous_flow_unit:
        /* ˲ʱ������λ 2�ֽڴ�С ö���� */
        if (((uint16_t)(pData[2] << 8 | pData[3])) != 0x02)
            return HAL_ERROR;
        data_from_r[pData[0] - 1].Instantaneous_flow_unit =
            Instantaneous_flow_unit_name[pData[5]];
        break;
    case Data_Type_Forward_flow_unit: 
        /* �����ۼ�������λ 2�ֽڴ�С ö���� */
        if (((uint16_t)(pData[2] << 8 | pData[3])) != 0x02)
            return HAL_ERROR;
        data_from_r[pData[0] - 1].Forward_flow_unit =
            Forward_flow_unit_name[pData[5]];
        break;
    default:
        return HAL_ERROR;
    }
    return HAL_OK;
}

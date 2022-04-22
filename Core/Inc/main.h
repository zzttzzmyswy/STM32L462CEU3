/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

    /* Private includes
     * ----------------------------------------------------------*/
    /* USER CODE BEGIN Includes */
#include "stdio.h"
    /* USER CODE END Includes */

    /* Exported types
     * ------------------------------------------------------------*/
    /* USER CODE BEGIN ET */

    /* USER CODE END ET */

    /* Exported constants
     * --------------------------------------------------------*/
    /* USER CODE BEGIN EC */

    /* USER CODE END EC */

    /* Exported macro
     * ------------------------------------------------------------*/
    /* USER CODE BEGIN EM */

    /* USER CODE END EM */

    /* Exported functions prototypes
     * ---------------------------------------------*/
    void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RS485_C_Pin GPIO_PIN_1
#define RS485_C_GPIO_Port GPIOA
#define SW1_Pin GPIO_PIN_5
#define SW1_GPIO_Port GPIOA
#define SW2_Pin GPIO_PIN_6
#define SW2_GPIO_Port GPIOA
#define SW3_Pin GPIO_PIN_7
#define SW3_GPIO_Port GPIOA
#define SW4_Pin GPIO_PIN_0
#define SW4_GPIO_Port GPIOB
#define LED0_Pin GPIO_PIN_1
#define LED0_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define BUTTON_SCAN(N)                                                         \
    {                                                                          \
        if (N == button_xd || N == button_flag)                                \
        {                                                                      \
            button_flag = N;                                                   \
        }                                                                      \
        else                                                                   \
        {                                                                      \
            button_xd = N;                                                     \
        }                                                                      \
    }
#define WAIT_MUDBUS(N)                                                          \
    {                                                                          \
        while (modbus_data_flag == 0)                                          \
            ;                                                                  \
        if (modbus_status != HAL_OK)                                           \
        {                                                                      \
            N++;                                                               \
        }                                                                      \
        modbus_data_flag = 0;                                                  \
        HAL_Delay(10);                                                         \
    }
#define RS485_Set_TE()                                                         \
    {                                                                          \
        HAL_GPIO_WritePin(RS485_C_GPIO_Port, RS485_C_Pin, GPIO_PIN_SET);       \
    }
#define RS485_Set_RE()                                                         \
    {                                                                          \
        HAL_GPIO_WritePin(RS485_C_GPIO_Port, RS485_C_Pin, GPIO_PIN_RESET);     \
    }

/* 接收环形缓存区大小 */
#define RX_BUFFER_SIZE 30

    struct struct_data_from_r
    {
        float Instantaneous_flow;      //瞬时流量
        long Forward_flow_long;        //累计流量整数
        float Forward_flow_float;      //累计流量小数
        char *Instantaneous_flow_unit; //瞬时流量单位
        char *Forward_flow_unit;       //累计流量单位
    };

    enum Data_Type
    {
        /* 瞬时流量地址：0x1010
        正向累积流量整数部分0x1018 小数部分0x101a
        瞬时流量单位0x1020
        累计流量单位0x1021 */
        Data_Type_Instantaneous = 0,
        Data_Type_Forward_flow,
        Data_Type_Instantaneous_flow_unit,
        Data_Type_Forward_flow_unit
    };

    void Delay_200us(void);
    /* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

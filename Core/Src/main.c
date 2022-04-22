/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "modbus.h"
#include "oled.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* 保证按钮扫描的时间 */
uint32_t button_time_count = 0;
/* 按钮标记 0-没有按下 1,2,3,4代表对应的按钮按下 */
uint32_t button_flag = 0;

/* modbus数据解析成功的标志，HAL_OK为解析正常，HAL_ERROR为解析异常 */
HAL_StatusTypeDef modbus_status = HAL_ERROR;

/* ↓↓↓↓↓ moubus底层不定长数据接收，使用DMA环形缓存区+软件双缓存 ↓↓↓↓↓ */
uint8_t aRXBufferUser[RX_BUFFER_SIZE]; // DMA复制目标环形空间地址
uint8_t aRXBufferA[RX_BUFFER_SIZE];    //用户空间A
uint8_t aRXBufferB[RX_BUFFER_SIZE];    //用户空间B
volatile uint32_t uwNbReceivedChars;   //收到数据的量
uint8_t *pBufferReadyForUser;          //用于用户解析的空间指针
uint8_t *pBufferReadyForReception;     //用于备用缓存区的空间指针
/* ↑↑↑↑↑ moubus底层不定长数据接收，使用DMA环形缓存区+软件双缓存 ↑↑↑↑↑ */

/* modbus数据收取成功标志，需要读完复位 */
volatile uint32_t modbus_data_flag = 0;

/* 数据存放内存空间，结构体数组，下标为从机地址 */
struct struct_data_from_r data_from_r[8];

/* 主循环通知中断解析数据类型 */
enum Data_Type Data_Type_e;

/* 单位字符串，用于显示 */
char *Instantaneous_flow_unit_name[12] = {"L/S",  "L/M", "L/H", "M3/S", "M3/M",
                                          "M3/H", "T/S", "T/M", "T/H"};
char *Forward_flow_unit_name[4] = {"L", "M3", "T", "USG"};

/* 显示从机下标,0代表显示主页面 */
uint32_t show_addr_flag = 0;

uint16_t send_cmd[] = {0x1010, 0x1018, 0x1020, 0x1021};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM6_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int stdout_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
    return ch;
}

void oled_Init(void)
{
    //初始化oled屏幕
    OLED_Init();
    //开启OLED显示
    OLED_Display_On();
    //清屏
    OLED_Clear();
}

void Bsp_Init(void)
{
    HAL_TIM_Base_Start_IT(&htim6);
    /* OLED显示屏初始化 */
    /* 喂狗 */
    HAL_IWDG_Refresh(&hiwdg);
    oled_Init();
    /* 喂狗 */
    HAL_IWDG_Refresh(&hiwdg);
    /* 接收模式 */
    RS485_Set_RE();
    /* 为软件双缓存区指针赋值 */
    pBufferReadyForReception = aRXBufferA;
    pBufferReadyForUser = aRXBufferB;
    /* 接收的数据字节复位 */
    uwNbReceivedChars = 0;
    // 初始化RS485串口为DMA接收模式，且带有可编程空闲检测中断
    if (HAL_OK !=
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, aRXBufferUser, RX_BUFFER_SIZE))
    {
        Error_Handler();
    }
    // 3.5 个字符时间设置为空闲检测时间
    HAL_UART_ReceiverTimeout_Config(&huart2, 10 + 10 + 10 + 5);
}

void oled_show_inf(void)
{
    struct struct_data_from_r *pData;
    char tempc[36];
    if (show_addr_flag == 0)
    {
        /* 张泽韬潘锦涛课设 */
        /* 可读数据瞬时流量 */
        /* 以及正向累计流量 */
        /* 按钮一可切换从机 */
        for (int k = 0; k < 4; k++)
            for (int i = 0; i < 8; i++)
                OLED_ShowCHinese(16 * i, 16 * k, i + 8 * k);
    }
    else
    {
        pData = data_from_r + show_addr_flag - 1;
        memset(tempc, 0, 36);
        OLED_Clear();
        /* 第一行：瞬时流量 [单位] [从机地址] */
        OLED_ShowCHinese(0, 0, 12);
        OLED_ShowCHinese(16, 0, 13);
        OLED_ShowCHinese(16 * 2, 0, 14);
        OLED_ShowCHinese(16 * 3, 0, 15);
        OLED_ShowString(16 * 4 + 8, 0,
                        (uint8_t *)pData->Instantaneous_flow_unit, BIG_FONT_EN);
        OLED_ShowNum(16 * 7 + 8, 0, show_addr_flag, 1, BIG_FONT_EN);
        /* 第二行和第三行：6x8字体大小显示瞬时流量 */
        sprintf(tempc, "  %.6f\0", pData->Instantaneous_flow);
        tempc[35] = '\0';
        OLED_ShowString(0, 16 * 1, (uint8_t *)tempc, SMALL_FONT_EN);
        /* 第四行：正向累计流量 [单位] */
        OLED_ShowCHinese(0, 16 * 2, 18);
        OLED_ShowCHinese(16 * 1, 16 * 2, 19);
        OLED_ShowCHinese(16 * 2, 16 * 2, 20);
        OLED_ShowCHinese(16 * 3, 16 * 2, 21);
        OLED_ShowCHinese(16 * 4, 16 * 2, 22);
        OLED_ShowCHinese(16 * 5, 16 * 2, 23);
        OLED_ShowString(16 * 6 + 8, 0, (uint8_t *)pData->Forward_flow_unit,
                        BIG_FONT_EN);
        /* 第五行和第六行：6x8大小显示正向累计流量 */
        sprintf(tempc, "  %.6lf\0",
                ((double)pData->Forward_flow_long +
                 (double)pData->Forward_flow_float));
        tempc[35] = '\0';
        OLED_ShowString(0, 16 * 3, (uint8_t *)tempc, SMALL_FONT_EN);
    }
}

/* 扫描按钮 */
void Scan_Button(void)
{
    static uint32_t button_xd = 0;
    if (GPIO_PIN_SET == HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin))
        BUTTON_SCAN(1)
    else if (GPIO_PIN_SET == HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin))
        BUTTON_SCAN(2)
    else if (GPIO_PIN_SET == HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin))
        BUTTON_SCAN(3)
    else if (GPIO_PIN_SET == HAL_GPIO_ReadPin(SW4_GPIO_Port, SW4_Pin))
        BUTTON_SCAN(4)
}

/* 定时器回调 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim6)
    {
        /* 1kHz */
        if (button_time_count == 20)
        {
            /* 50Hz */
            button_time_count = 0;
            Scan_Button();
        }
        button_time_count += 1;
    }
}

/* 串口不定时收取数据空闲中断处理回调 */
void UserDataTreatment(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    /*
     * This function might be called in any of the following interrupt contexts
     * :
     *  - DMA TC and HT events
     *  - UART IDLE line event
     *
     * pData and Size defines the buffer where received data have been copied,
     * in order to be processed. During this processing of already received
     * data, reception is still ongoing.
     *
     */
    /*
     *此函数可以在以下任何中断上下文中调用：
     *-DMA TC和HT事件
     *-UART空闲线路事件
     *
     *pData和Size定义复制接收到的数据以进行处理的缓冲区。
     *在处理已接收数据的过程中，接收仍在进行。
     *
     */
    modbus_status = Master_Rx_Data(pData, Size, Data_Type_e);
#if 0
    uint8_t *pBuff = pData;
    uint8_t i;
    /* Implementation of loopback is on purpose implemented in direct register access,
        in order to be able to echo received characters as fast as they are received.
        Wait for TC flag to be raised at end of transmit is then removed, only TXE is checked */
    /*环回的实现是在直接寄存器访问中故意实现的，
    以便能够以接收字符的速度回显接收到的字符。
    等待TC标志在传输结束时升起，然后移除，只检查TXE*/
    for (i = 0; i < Size; i++)
    {
        while (!(__HAL_UART_GET_FLAG(huart, UART_FLAG_TXE)))
        {
        }
        huart->Instance->TDR = *pBuff;
        pBuff++;
    }
#endif
}

/* 串口接收事件回调 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    static uint8_t old_pos = 0;
    uint8_t *ptemp;
    uint8_t i;

    /* Check if number of received data in recpetion buffer has changed */
    /*检查recpetion缓冲区中接收的数据数量是否已更改*/
    if (Size != old_pos)
    {
        /* Check if position of index in reception buffer has simply be
           increased of if end of buffer has been reached */
        /*检查接收缓冲器中的索引位置是否只是增加了
            如果已到达缓冲区的末尾*/
        if (Size > old_pos)
        {
            /* Current position is higher than previous one */
            uwNbReceivedChars = Size - old_pos;
            /* Copy received data in "User" buffer for evacuation */
            /*将接收到的数据复制到“用户”缓冲区以进行疏散*/
            for (i = 0; i < uwNbReceivedChars; i++)
            {
                pBufferReadyForUser[i] = aRXBufferUser[old_pos + i];
            }
        }
        else
        {
            /* Current position is lower than previous one : end of buffer has
             * been reached */
            /* First copy data from current position till end of buffer */
            /*当前位置低于前一个位置：已到达缓冲区末端*/
            /*首先从当前位置复制数据，直到缓冲区结束*/
            uwNbReceivedChars = RX_BUFFER_SIZE - old_pos;
            /* Copy received data in "User" buffer for evacuation */
            /*将接收到的数据复制到“用户”缓冲区以进行疏散*/
            for (i = 0; i < uwNbReceivedChars; i++)
            {
                pBufferReadyForUser[i] = aRXBufferUser[old_pos + i];
            }
            /* Check and continue with beginning of buffer */
            /*检查并继续缓冲区的开始*/
            if (Size > 0)
            {
                for (i = 0; i < Size; i++)
                {
                    pBufferReadyForUser[uwNbReceivedChars + i] =
                        aRXBufferUser[i];
                }
                uwNbReceivedChars += Size;
            }
        }
        /* Process received data that has been extracted from Rx User buffer */
        /*处理从Rx用户缓冲区提取的接收数据*/
        UserDataTreatment(huart, pBufferReadyForUser, uwNbReceivedChars);

        /* Swap buffers for next bytes to be processed */
        /*为下一个要处理的字节交换缓冲区*/
        ptemp = pBufferReadyForUser;
        pBufferReadyForUser = pBufferReadyForReception;
        pBufferReadyForReception = ptemp;
    }
    /* Update old_pos as new reference of position in User Rx buffer that
       indicates position to which data have been processed */
    /*将旧的_pos更新为用户接收缓冲区中位置的新参考
        指示数据已处理到的位置*/
    old_pos = Size;
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

    /* MCU
     * Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the
     * Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_DMA_Init();
    MX_TIM6_Init();
    MX_IWDG_Init();
    /* USER CODE BEGIN 2 */
    Bsp_Init();
    uint32_t error_count = 0, error_count_old = 0;
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    /* 喂狗 */
    HAL_IWDG_Refresh(&hiwdg);
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        error_count_old = error_count;
        /* 查询八个从机 */
        for (int i = 1; i <= 8; i++)
        {
            /* 查询四个量 */
            Data_Type_e = Data_Type_Instantaneous;
            Master_Send(i, send_cmd[Data_Type_e], 2);
            WAIT_MUDBUS(error_count);
            Data_Type_e = Data_Type_Forward_flow;
            Master_Send(i, send_cmd[Data_Type_e], 4);
            WAIT_MUDBUS(error_count);
            Data_Type_e = Data_Type_Instantaneous_flow_unit;
            Master_Send(i, send_cmd[Data_Type_e], 1);
            WAIT_MUDBUS(error_count);
            Data_Type_e = Data_Type_Forward_flow_unit;
            Master_Send(i, send_cmd[Data_Type_e], 1);
            WAIT_MUDBUS(error_count);
        }
        /* 如果查询八个从机的四个量都没有错误，则错误计数归0 */
        if (error_count_old == error_count)
            error_count = 0;
        /* 如果错误数量足够多，就进入错误函数，等待狗复位 */
        if (error_count >= 100)
            Error_Handler();
        /* 获取按钮当前按下的信息 */
        switch (button_flag)
        {
        case 0: /* 未按下按钮 */
            break;
        case 1: /* 按钮1 */
            show_addr_flag = show_addr_flag == 8 ? 0 : show_addr_flag + 1;
            button_flag = 0;
            break;
        case 2: /* 按钮2 */
            break;
        case 3: /* 按钮3 */
            break;
        case 4: /* 按钮4 */
            break;
        }
        /* 显示屏根据需要显示的内容进行刷新 */
        oled_show_inf();
        /* 输出获取的信息 */
        for (uint32_t i; i < 8; i++)
            printf("从机:%d 瞬时流量:%.6f %s 正向累计流量:%.6lf %s\r\n", i + 1,
                   data_from_r[i].Instantaneous_flow,
                   data_from_r[i].Instantaneous_flow_unit,
                   ((double)data_from_r[i].Forward_flow_long +
                    (double)data_from_r[i].Forward_flow_float),
                   data_from_r[i].Forward_flow_unit);
        /* 延时半秒 */
        HAL_Delay(500);
        /* 喂狗 */
        HAL_IWDG_Refresh(&hiwdg);
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
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 10;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }

    /** Enables the Clock Security System
     */
    HAL_RCC_EnableCSS();
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x10909CEC;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Analogue filter
     */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Digital filter
     */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief IWDG Initialization Function
 * @param None
 * @retval None
 */
static void MX_IWDG_Init(void)
{

    /* USER CODE BEGIN IWDG_Init 0 */

    /* USER CODE END IWDG_Init 0 */

    /* USER CODE BEGIN IWDG_Init 1 */

    /* USER CODE END IWDG_Init 1 */
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
    hiwdg.Init.Window = 4000;
    hiwdg.Init.Reload = 4000;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN IWDG_Init 2 */

    /* USER CODE END IWDG_Init 2 */
}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void)
{

    /* USER CODE BEGIN TIM6_Init 0 */

    /* USER CODE END TIM6_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM6_Init 1 */

    /* USER CODE END TIM6_Init 1 */
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 80 - 1;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 1000;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_OnePulse_Init(&htim6, TIM_OPMODE_SINGLE) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM6_Init 2 */

    /* USER CODE END TIM6_Init 2 */
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
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 9600;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Channel6_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
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
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(RS485_C_GPIO_Port, RS485_C_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin : RS485_C_Pin */
    GPIO_InitStruct.Pin = RS485_C_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RS485_C_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : SW1_Pin SW2_Pin SW3_Pin */
    GPIO_InitStruct.Pin = SW1_Pin | SW2_Pin | SW3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : SW4_Pin */
    GPIO_InitStruct.Pin = SW4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(SW4_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : LED0_Pin */
    GPIO_InitStruct.Pin = LED0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED0_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state
     */
    /* 禁止全局中断 */
    __disable_irq();
    while (1)
    {
        HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
        HAL_UART_Transmit(&huart1, "Error_Handler\r\n",
                          sizeof("Error_Handler\r\n") - 1, 0xfff);
        HAL_Delay(200);
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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
    /* User can add his own implementation to report the file name and line
       number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
       file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

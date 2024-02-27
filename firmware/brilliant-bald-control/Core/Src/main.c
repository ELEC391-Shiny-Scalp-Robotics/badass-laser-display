/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 Charles Surianto.
 * All rights reserved.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
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

I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
DMA_HandleTypeDef hdma_tim4_ch1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint16_t WsBuffer[48];
UARTStruct Uart1;
FSMStateTypeDef FSMState = INIT;
bool FSMIsParsing = FALSE;
bool CommandReady = FALSE;
bool MoveTimedOut = FALSE;
LaserPosStruct LaserPos = {0.0, 0.0};
double Kp = 0;
double Kd = 0;
uint16_t HomingSpeed = 400;
MotorStruct MotorX = {0, 0, 0, 0};
MotorStruct MotorY = {0, 0, 0, 0};

const uint8_t COS256[256] = {
    255, 255, 255, 255, 254, 254, 254, 253, 253, 252, 251, 250, 249, 249, 248, 246,
    245, 244, 243, 241, 240, 238, 237, 235, 233, 232, 230, 228, 226, 224, 222, 220,
    218, 215, 213, 211, 208, 206, 203, 201, 198, 195, 193, 190, 187, 185, 182, 179,
    176, 173, 170, 167, 164, 161, 158, 155, 152, 149, 146, 143, 140, 136, 133, 130,
    127, 124, 121, 118, 114, 111, 108, 105, 102, 99, 96, 93, 90, 87, 84, 81,
    78, 75, 72, 69, 67, 64, 61, 59, 56, 53, 51, 48, 46, 43, 41, 39,
    36, 34, 32, 30, 28, 26, 24, 22, 21, 19, 17, 16, 14, 13, 11, 10,
    9, 8, 6, 5, 5, 4, 3, 2, 1, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 3, 4, 5, 5, 6, 8,
    9, 10, 11, 13, 14, 16, 17, 19, 21, 22, 24, 26, 28, 30, 32, 34,
    36, 39, 41, 43, 46, 48, 51, 53, 56, 59, 61, 64, 67, 69, 72, 75,
    78, 81, 84, 87, 90, 93, 96, 99, 102, 105, 108, 111, 114, 118, 121, 124,
    127, 130, 133, 136, 140, 143, 146, 149, 152, 155, 158, 161, 164, 167, 170, 173,
    176, 179, 182, 185, 187, 190, 193, 195, 198, 201, 203, 206, 208, 211, 213, 215,
    218, 220, 222, 224, 226, 228, 230, 232, 233, 235, 237, 238, 240, 241, 243, 244,
    245, 246, 248, 249, 249, 250, 251, 252, 253, 253, 254, 254, 254, 255, 255, 255};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void ParseCommand(void);
bool StringStartsWith(const char *str, const char *prefix);
void WsSet(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
void WsSetAll(uint8_t r, uint8_t g, uint8_t b);
void EEPROMRead(uint8_t regAddr, uint8_t *pData, uint8_t size);
void EEPROMWrite(uint8_t regAddr, uint8_t *pData, uint8_t size);
void ParamsRead(void);
void ParamsWrite(void);
void MotorSetSpeed(AxisTypeDef axis, int16_t speed);
void LaserTurn(LaserStateTypeDef state);
void LaserSetPos(double x, double y);
void LaserLineTo(double x, double y, uint16_t steps);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
    MX_DMA_Init();
    MX_TIM1_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    MX_USART1_UART_Init();
    MX_I2C3_Init();
    MX_TIM7_Init();
    MX_TIM6_Init();
    /* USER CODE BEGIN 2 */
    HAL_TIM_Encoder_Start(&HTIM_ENCX, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&HTIM_ENCY, TIM_CHANNEL_ALL);

    HAL_TIM_PWM_Start(&HTIM_MOT, TIM_CHANNEL_MOT_XA);
    HAL_TIM_PWM_Start(&HTIM_MOT, TIM_CHANNEL_MOT_XB);
    HAL_TIM_PWM_Start(&HTIM_MOT, TIM_CHANNEL_MOT_YA);
    HAL_TIM_PWM_Start(&HTIM_MOT, TIM_CHANNEL_MOT_YB);

    HAL_UART_Receive_IT(&huart1, &Uart1.rxByte, 1);

    {
        uint8_t configByte;
        EEPROMRead(CONFIG_BYTE_ADDR, &configByte, 1);

        if (configByte || HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_SET)
        {
            FSMState = TEST;
        }
    }
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        if (FSMState == TEST)
        {
            SerialPrint("test mode\n");

            while (FSMState == TEST)
            {
                ParseCommand();
            }
        }

        if (FSMState == INIT)
        {
            SerialPrint("init\n");

            // stop control loop
            HAL_TIM_Base_Stop_IT(&HTIM_CTRL);

            // load eeprom for Kp, Kd, offset, homing speed
            // EEPROMRead(...);

            // home motors
            // MotorSetSpeed(X, HomingSpeed);
            // wait for Stop
            // MotorSetSpeed(X, 0);
            // __HAL_TIM_SET_COUNTER(&HTIM_ENCX, MotorX.offset);
            // ...
            // MotorSetSpeed(Y, 0);
            // __HAL_TIM_SET_COUNTER(&HTIM_ENCY, -MotorY.offset);
            // reset motor structs

            // start control loop
            // HAL_TIM_Base_Start_IT(&HTIM_CTRL);

            FSMState = SERIAL;
        }

        if (FSMState == SERIAL)
        {
            SerialPrint("serial mode\n");

            while (FSMState == SERIAL)
            {
                ParseCommand();
            }
        }

        if (FSMState == MEMORY_SQUARE)
        {
            SerialPrint("square mode\n");

            while (FSMState == MEMORY_SQUARE)
            {
                // draw square

                ParseCommand();
            }
        }

        if (FSMState == RB)
        {
            while (FSMState == RB)
            {
                for (int j = 0; j < 2; j++)
                {
                    WsSetAll(0, 0, 0);
                    for (int i = 0; i < 2; i++)
                    {
                        WsSet(j, 0, 0, 0);
                        WsShow();
                        HAL_Delay(50);
                        WsSet(j, 30, 0, 0);
                        WsShow();
                        HAL_Delay(50);
                    }
                    HAL_Delay(100);
                    WsSetAll(0, 0, 0);
                    for (int i = 0; i < 2; i++)
                    {
                        WsSet(j, 0, 0, 0);
                        WsShow();
                        HAL_Delay(50);
                        WsSet(j, 0, 0, 30);
                        WsShow();
                        HAL_Delay(50);
                    }
                    HAL_Delay(100);
                }
                ParseCommand();
            }
        }

        if (FSMState == RGB)
        {
            while (FSMState == RGB)
            {
                for (int i = 0; i < 256; i++)
                {
                    WsSetAll(COS256[(uint8_t)i] >> 4, COS256[(uint8_t)(i - 256 / 3)] >> 4, COS256[(uint8_t)(i - 256 * 2 / 3)] >> 4);
                    WsShow();
                    HAL_Delay(10);
                }
                ParseCommand();
            }
        }
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
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

    /** Supply configuration update enable
     */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    /** Configure the main internal regulator output voltage
     */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
    {
    }

    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
    {
    }

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 5;
    RCC_OscInitStruct.PLL.PLLN = 192;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 20;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
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
    hi2c3.Init.Timing = 0x00B03FDB;
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

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    /* USER CODE BEGIN TIM1_Init 1 */

    /* USER CODE END TIM1_Init 1 */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 4800 - 1;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM2;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter = 0;
    sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    sBreakDeadTimeConfig.Break2Filter = 0;
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
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

    /* USER CODE BEGIN TIM3_Init 0 */

    /* USER CODE END TIM3_Init 0 */

    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 65535;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0;
    if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM3_Init 2 */

    /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

    /* USER CODE BEGIN TIM4_Init 0 */

    /* USER CODE END TIM4_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM4_Init 1 */

    /* USER CODE END TIM4_Init 1 */
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 2 - 1;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 150 - 1;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM4_Init 2 */

    /* USER CODE END TIM4_Init 2 */
    HAL_TIM_MspPostInit(&htim4);
}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void)
{

    /* USER CODE BEGIN TIM5_Init 0 */

    /* USER CODE END TIM5_Init 0 */

    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM5_Init 1 */

    /* USER CODE END TIM5_Init 1 */
    htim5.Instance = TIM5;
    htim5.Init.Prescaler = 0;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 65535;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0;
    if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM5_Init 2 */

    /* USER CODE END TIM5_Init 2 */
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
    htim6.Init.Prescaler = 24000 - 1;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 65535;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM6_Init 2 */

    /* USER CODE END TIM6_Init 2 */
}

/**
 * @brief TIM7 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM7_Init(void)
{

    /* USER CODE BEGIN TIM7_Init 0 */

    /* USER CODE END TIM7_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM7_Init 1 */

    /* USER CODE END TIM7_Init 1 */
    htim7.Instance = TIM7;
    htim7.Init.Prescaler = 0;
    htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim7.Init.Period = 48000 - 1;
    htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM7_Init 2 */

    /* USER CODE END TIM7_Init 2 */
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
    huart1.Init.BaudRate = 1000000;
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
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Stream0_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, LED3_Pin | LED2_Pin | LED1_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : KEY3_Pin KEY1_Pin KEY2_Pin */
    GPIO_InitStruct.Pin = KEY3_Pin | KEY1_Pin | KEY2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pins : PA2 PA3 */
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : LASER_Pin */
    GPIO_InitStruct.Pin = LASER_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LASER_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PA11 PA12 */
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : LED4_Pin */
    GPIO_InitStruct.Pin = LED4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED4_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : LED3_Pin LED2_Pin LED1_Pin */
    GPIO_InitStruct.Pin = LED3_Pin | LED2_Pin | LED1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &HTIM_CTRL)
    {
        // HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);

        // control loop

        // int16_t encoderx = (int16_t)__HAL_TIM_GET_COUNTER(&HTIM_ENCX);
        // int16_t encodery = (int16_t)__HAL_TIM_GET_COUNTER(&HTIM_ENCY);

        // HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &HTIM_WS)
    {
        __HAL_TIM_SET_COMPARE(&HTIM_WS, TIM_CHANNEL_WS, 0);
        HAL_TIM_PWM_Stop_DMA(&HTIM_WS, TIM_CHANNEL_WS);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        if (HAL_UART_Receive_IT(&huart1, &Uart1.rxByte, 1) == HAL_OK)
        {
            if (!FSMIsParsing)
            {
                if (Uart1.rxByte == '\n' || Uart1.rxByte == '\r' || Uart1.rxCounter >= sizeof(Uart1.rxBuffer) - 2)
                {
                    Uart1.rxBuffer[Uart1.rxCounter] = 0;
                    Uart1.rxBuffer[Uart1.rxCounter + 1] = 0;
                    Uart1.rxCounter = 0;
                    CommandReady = TRUE;
                }
                else
                {
                    Uart1.rxBuffer[Uart1.rxCounter++] = Uart1.rxByte;
                }
            }
        }
        else
        {
            HAL_UART_AbortReceive(&huart1);
            HAL_UART_Receive_IT(&huart1, &Uart1.rxByte, 1);
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
    }
}

void ParseCommand(void)
{
    ParseStatusTypeDef status = OK;

    if (CommandReady)
    {
        FSMIsParsing = TRUE;

        if (StringStartsWith((char *)Uart1.rxBuffer, "move"))
        {
            double x, y;
            uint16_t steps;
            if (sscanf((char *)&Uart1.rxBuffer[sizeof("move")], "%lf %lf %hu", &x, &y, &steps) == 3)
            {
                LaserLineTo(x, y, steps);
                if (MoveTimedOut)
                {
                    status = TIMEDOUT;
                }
            }
            else
            {
                status = INVALID_ARGUMENT;
            }
        }
        else if (StringStartsWith((char *)Uart1.rxBuffer, "laser"))
        {
            char *pBuffer = (char *)(&Uart1.rxBuffer[sizeof("laser")]);

            if (StringStartsWith(pBuffer, "on"))
            {
                LaserTurn(ON);
            }
            else if (StringStartsWith(pBuffer, "off"))
            {
                LaserTurn(OFF);
            }
            else
            {
                status = INVALID_ARGUMENT;
            }
        }
        else if (StringStartsWith((char *)Uart1.rxBuffer, "motor"))
        {
            char *pBuffer = (char *)(&Uart1.rxBuffer[sizeof("motor")]);

            if (*pBuffer == 'x')
            {
                int16_t speed;
                if (sscanf(&pBuffer[sizeof("x")], "%hd", &speed) == 1)
                {
                    MotorSetSpeed(X, speed);
                }
                else
                {
                    status = INVALID_ARGUMENT;
                }
            }
            else if (*pBuffer == 'y')
            {
                int16_t speed;
                if (sscanf(&pBuffer[sizeof("y")], "%hd", &speed) == 1)
                {
                    MotorSetSpeed(Y, speed);
                }
                else
                {
                    status = INVALID_ARGUMENT;
                }
            }
            else
            {
                status = INVALID_ARGUMENT;
            }
        }
        else if (StringStartsWith((char *)Uart1.rxBuffer, "led"))
        {
            uint16_t index, state;
            if (sscanf((char *)&Uart1.rxBuffer[sizeof("led")], "%hu %hu", &index, &state) == 2)
            {
                switch (index)
                {
                case 1:
                    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
                    break;
                case 2:
                    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
                    break;
                case 3:
                    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
                    break;
                case 4:
                    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
                    break;
                }
            }
            else
            {
                status = INVALID_ARGUMENT;
            }
        }
        else if (StringStartsWith((char *)Uart1.rxBuffer, "rgb"))
        {
            uint16_t index, r, g, b;
            if (sscanf((char *)&Uart1.rxBuffer[sizeof("rgb")], "%hu %hu %hu %hu", &index, &r, &g, &b) == 4)
            {
                WsSet(index, r, g, b);
                WsShow();
            }
            else
            {
                status = INVALID_ARGUMENT;
            }
        }
        else if (StringStartsWith((char *)Uart1.rxBuffer, "param"))
        {
            char *pBuffer = (char *)(&Uart1.rxBuffer[sizeof("param")]);

            if (StringStartsWith(pBuffer, "load"))
            {
                ParamsRead();
            }
            else if (StringStartsWith(pBuffer, "save"))
            {
                ParamsWrite();
            }
            else if (StringStartsWith(pBuffer, "kp"))
            {
                if (sscanf(&pBuffer[sizeof("kp")], "%lf", &Kp) == 1)
                {
                    SerialPrint("kp <- %lf\n", Kp);
                }
                else
                {
                    SerialPrint("kp: %lf\n", Kp);
                }
                status = SILENT;
            }
            else if (StringStartsWith(pBuffer, "kd"))
            {
                if (sscanf(&pBuffer[sizeof("kd")], "%lf", &Kd) == 1)
                {
                    SerialPrint("kd <- %lf\n", Kd);
                }
                else
                {
                    SerialPrint("kd: %lf\n", Kd);
                }
                status = SILENT;
            }
            else if (StringStartsWith(pBuffer, "offset"))
            {
                pBuffer += sizeof("offset");

                if (StringStartsWith(pBuffer, "x"))
                {
                    if (sscanf(&pBuffer[sizeof("x")], "%hd", &MotorX.homeOffset) == 1)
                    {
                        SerialPrint("offset x <- %hd\n", MotorX.homeOffset);
                    }
                    else
                    {
                        SerialPrint("offset x: %hd\n", MotorX.homeOffset);
                    }
                    status = SILENT;
                }
                else if (StringStartsWith(pBuffer, "y"))
                {
                    if (sscanf(&pBuffer[sizeof("y")], "%hd", &MotorY.homeOffset) == 1)
                    {
                        SerialPrint("offset y <- %hd\n", MotorY.homeOffset);
                    }
                    else
                    {
                        SerialPrint("offset y: %hd\n", MotorY.homeOffset);
                    }
                    status = SILENT;
                }
                else
                {
                    status = INVALID_ARGUMENT;
                }
            }
            else if (StringStartsWith(pBuffer, "homing"))
            {
                pBuffer += sizeof("homing");

                if (sscanf(pBuffer, "%hu", &HomingSpeed) == 1)
                {
                    SerialPrint("homing <- %hu\n", HomingSpeed);
                }
                else
                {
                    SerialPrint("homing: %hu\n", HomingSpeed);
                }
                status = SILENT;
            }
            else
            {
                SerialPrint("kp: %lf\nkd: %lf\nhoming speed: %hu\noffset x: %hd\noffset y: %hd\n", Kp, Kd, HomingSpeed, MotorX.homeOffset, MotorY.homeOffset);
                status = SILENT;
            }
        }
        else if (StringStartsWith((char *)Uart1.rxBuffer, "rom"))
        {
            char *pBuffer = (char *)(&Uart1.rxBuffer[sizeof("rom")]);

            if (StringStartsWith(pBuffer, "dump"))
            {
                Uart1.txCounter = 0;
                uint8_t data[16];

                for (int i = 0; i < 16; i++)
                {
                    EEPROMRead(i * 16, data, 16);
                    Uart1.txCounter += sprintf((char *)&Uart1.txBuffer[Uart1.txCounter], "%02x:", i * 16);

                    for (int j = 0; j < 16; j++)
                    {
                        Uart1.txCounter += sprintf((char *)&Uart1.txBuffer[Uart1.txCounter], " %02x", data[j]);
                    }
                    Uart1.txCounter += sprintf((char *)&Uart1.txBuffer[Uart1.txCounter], "\n");
                }

                HAL_UART_Transmit(&huart1, Uart1.txBuffer, Uart1.txCounter, UART_TIMEOUT);
                status = SILENT;
            }
            else if (StringStartsWith(pBuffer, "write"))
            {
                int addr, wdata;
                if (sscanf(&pBuffer[sizeof("write")], "%x %x", &addr, &wdata) == 2)
                {
                    uint8_t data = (uint8_t)wdata;
                    EEPROMWrite((uint8_t)addr, &data, 1);
                }
                else
                {
                    status = INVALID_ARGUMENT;
                }
            }
            else if (StringStartsWith(pBuffer, "read"))
            {
                int addr;
                if (sscanf(&pBuffer[sizeof("read")], "%x", &addr) == 1)
                {
                    uint8_t data;
                    EEPROMRead((uint8_t)addr, &data, 1);

                    SerialPrint("%02x: %02x\n", addr, data);
                    status = SILENT;
                }
                else
                {
                    status = INVALID_ARGUMENT;
                }
            }
            else
            {
                status = INVALID_ARGUMENT;
            }
        }
        else if (StringStartsWith((char *)Uart1.rxBuffer, "mode"))
        {
            char *pBuffer = (char *)(&Uart1.rxBuffer[sizeof("mode")]);

            if (StringStartsWith(pBuffer, "test"))
            {
                FSMState = TEST;
            }
            else if (StringStartsWith(pBuffer, "init"))
            {
                FSMState = INIT;
            }
            else if (StringStartsWith(pBuffer, "serial"))
            {
                FSMState = SERIAL;
            }
            else if (StringStartsWith(pBuffer, "rb"))
            {
                FSMState = RB;
            }
            else if (StringStartsWith(pBuffer, "rgb"))
            {
                FSMState = RGB;
            }
            else
            {
                status = INVALID_ARGUMENT;
            }
        }
        else if (StringStartsWith((char *)Uart1.rxBuffer, "control"))
        {
            char *pBuffer = (char *)(&Uart1.rxBuffer[sizeof("control")]);

            if (StringStartsWith(pBuffer, "on"))
            {
                HAL_TIM_Base_Start_IT(&HTIM_CTRL);
            }
            else if (StringStartsWith(pBuffer, "off"))
            {
                HAL_TIM_Base_Stop_IT(&HTIM_CTRL);
            }
            else
            {
                status = INVALID_ARGUMENT;
            }
        }
        else if (StringStartsWith((char *)Uart1.rxBuffer, "hardware reset 69420"))
        {
            NVIC_SystemReset();
        }
        // else if (StringStartsWith((char *)Uart1.rxBuffer, "help"))
        // {
        //     SerialPrint("move <x: double> <y: double> <steps: int>\n");
        //     SerialPrint("laser <on/off>\n");
        //     SerialPrint("led <1-4> <0/1>\n");
        //     SerialPrint("rgb <0-1> <r> <g> <b>\n");
        //     SerialPrint("rom dump\n");
        //     SerialPrint("rom write <addr: hex> <data: hex>\n");
        //     SerialPrint("rom read <addr: hex>\n");
        //     SerialPrint("mode test\n");
        //     SerialPrint("mode init\n");
        //     SerialPrint("mode serial\n");
        //     SerialPrint("mode rb\n");
        //     SerialPrint("mode rgb\n");
        //     SerialPrint("hardware reset <code>\n");
        //     status = SILENT;
        // }
        else
        {
            status = INVALID_COMMAND;
        }

        switch (status)
        {
        case OK:
            SerialPrint("ok\n");
            break;
        case SILENT:
            break;
        case INVALID_COMMAND:
            SerialPrint("invalid command\n");
            break;
        case INVALID_ARGUMENT:
            SerialPrint("invalid argument\n");
            break;
        case TIMEDOUT:
            SerialPrint("timed out\n");
            break;
        }

        CommandReady = FALSE;
        FSMIsParsing = FALSE;
    }
}

bool StringStartsWith(const char *str, const char *prefix)
{
    for (uint8_t i = 0; prefix[i] != 0; i++)
    {
        if (str[i] != prefix[i])
            return FALSE;
    }
    return TRUE;
}

void WsSet(uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
    index *= 24;
    uint8_t i = 0, mask;
    for (mask = 0x80; mask > 0; i++, mask >>= 1)
    {
        if (g & mask)
            WsBuffer[index + i] = WS2812_PULSE_1;
        else
            WsBuffer[index + i] = WS2812_PULSE_0;
    }
    for (mask = 0x80; mask > 0; i++, mask >>= 1)
    {
        if (r & mask)
            WsBuffer[index + i] = WS2812_PULSE_1;
        else
            WsBuffer[index + i] = WS2812_PULSE_0;
    }
    for (mask = 0x80; mask > 0; i++, mask >>= 1)
    {
        if (b & mask)
            WsBuffer[index + i] = WS2812_PULSE_1;
        else
            WsBuffer[index + i] = WS2812_PULSE_0;
    }
}

void WsSetAll(uint8_t r, uint8_t g, uint8_t b)
{
    WsSet(0, r, g, b);
    WsSet(1, r, g, b);
}

void EEPROMRead(uint8_t regAddr, uint8_t *pData, uint8_t size)
{
    HAL_I2C_Mem_Read(&hi2c3, EEPROM_ADDR << 1, regAddr, I2C_MEMADD_SIZE_8BIT, pData, size, 1000);
}

void EEPROMWrite(uint8_t regAddr, uint8_t *pData, uint8_t size)
{
    HAL_I2C_Mem_Write(&hi2c3, EEPROM_ADDR << 1, regAddr, I2C_MEMADD_SIZE_8BIT, pData, size, 1000);
}

/**
 * eeprom map:
 *  0.. 7: Kp (double)
 *  8..15: Kd (double)
 * 16..17: HomingSpeed (uint16_t)
 * 18..19: MotorX.homeOffset (int16_t)
 * 20..21: MotorY.homeOffset (int16_t)
 */

void ParamsRead(void)
{
    uint8_t readBuffer[22];

    EEPROMRead(0, readBuffer, 22);

    for (int i = 0; i < 8; i++)
    {
        ((uint8_t *)&Kp)[i] = readBuffer[i];
        ((uint8_t *)&Kd)[i] = readBuffer[i + 8];
    }

    for (int i = 0; i < 2; i++)
    {
        ((uint8_t *)&HomingSpeed)[i] = readBuffer[i + 16];
        ((uint8_t *)&MotorX.homeOffset)[i] = readBuffer[i + 2 + 16];
        ((uint8_t *)&MotorY.homeOffset)[i] = readBuffer[i + 4 + 16];
    }
}

void ParamsWrite(void)
{
    uint8_t pageBuffer[16];

    for (int i = 0; i < 8; i++)
    {
        pageBuffer[i] = ((uint8_t *)&Kp)[i];
        pageBuffer[i + 8] = ((uint8_t *)&Kd)[i];
    }
    EEPROMWrite(0, pageBuffer, 16);
    HAL_Delay(5);

    for (int i = 0; i < 2; i++)
    {
        pageBuffer[i] = ((uint8_t *)&HomingSpeed)[i];
        pageBuffer[i + 2] = ((uint8_t *)&MotorX.homeOffset)[i];
        pageBuffer[i + 4] = ((uint8_t *)&MotorY.homeOffset)[i];
    }
    EEPROMWrite(16, pageBuffer, 6);
}

void MotorSetSpeed(AxisTypeDef axis, int16_t speed)
{
    uint16_t pulseA, pulseB;

    if (speed >= 0)
    {
        pulseA = speed;
        pulseB = 0;
    }
    else
    {
        pulseA = 0;
        pulseB = -speed;
    }

    switch (axis)
    {
    case X:
        __HAL_TIM_SET_COMPARE(&HTIM_MOT, TIM_CHANNEL_MOT_XA, pulseA);
        __HAL_TIM_SET_COMPARE(&HTIM_MOT, TIM_CHANNEL_MOT_XB, pulseB);
        break;
    case Y:
        __HAL_TIM_SET_COMPARE(&HTIM_MOT, TIM_CHANNEL_MOT_YA, pulseA);
        __HAL_TIM_SET_COMPARE(&HTIM_MOT, TIM_CHANNEL_MOT_YB, pulseB);
        break;
    }
}

void LaserTurn(LaserStateTypeDef state)
{
    if (state == ON)
    {
        HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);
    }
    else if (state == OFF)
    {
        HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
    }
}

void LaserSetPos(double x, double y)
{
    // calculate required encoder values
    MotorX.target = (int16_t)(atan2(x, NORM_DIST) * STEPS_PER_RAD);
    MotorY.target = (int16_t)(atan2(y, NORM_DIST) * STEPS_PER_RAD);

    int16_t xEncoder, yEncoder;
    double xError, yError;

    // wait for error to be within threshold or time out
    do
    {
        xEncoder = (int16_t)__HAL_TIM_GET_COUNTER(&HTIM_ENCX);
        yEncoder = (int16_t)__HAL_TIM_GET_COUNTER(&HTIM_ENCY);

        LaserPos.x = tan(xEncoder * RAD_PER_STEPS) * NORM_DIST;
        LaserPos.y = tan(yEncoder * RAD_PER_STEPS) * NORM_DIST;

        xError = x - LaserPos.x;
        yError = y - LaserPos.y;

        *(uint64_t *)&xError = *(uint64_t *)&xError & 0x7FFFFFFFFFFFFFFF;
        *(uint64_t *)&yError = *(uint64_t *)&yError & 0x7FFFFFFFFFFFFFFF;

    } while (__HAL_TIM_GET_COUNTER(&htim6) < TIMEOUT_PERIOD && (xError > ERROR_THRESHOLD || yError > ERROR_THRESHOLD));
}

void LaserLineTo(double x, double y, uint16_t steps)
{
    LaserPosStruct startPos = LaserPos;
    double xDistance = x - startPos.x;
    double yDistance = y - startPos.y;

    // start time out timer
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    HAL_TIM_Base_Start(&htim6);

    // interpolate between start and end position
    for (int i = 1; i <= steps && __HAL_TIM_GET_COUNTER(&htim6) < TIMEOUT_PERIOD; i++)
    {
        LaserSetPos(startPos.x + (double)i / steps * xDistance, startPos.y + (double)i / steps * yDistance);
    }
    HAL_TIM_Base_Stop(&htim6);

    if (__HAL_TIM_GET_COUNTER(&htim6) >= TIMEOUT_PERIOD)
    {
        MoveTimedOut = TRUE;
    }
    else
    {
        MoveTimedOut = FALSE;
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
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

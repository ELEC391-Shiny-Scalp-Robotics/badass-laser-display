/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 Charles Surianto.
 * All rights reserved.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct UARTStruct
{
    uint8_t rxByte;
    uint16_t rxCounter;
    uint8_t rxBuffer[1024];
    uint16_t txCounter;
    uint8_t txBuffer[1024];
} UARTStruct;

typedef struct LaserPosStruct
{
    double x;
    double y;
} LaserPosStruct;

typedef struct MotorStruct
{
    int16_t target;
    int16_t error;
    int16_t lastError;
    int16_t homeOffset;
} MotorStruct;

typedef enum AxisTypeDef
{
    X,
    Y
} AxisTypeDef;

typedef enum LaserStateTypeDef
{
    OFF,
    ON
} LaserStateTypeDef;

typedef enum bool
{
    FALSE,
    TRUE
} bool;

typedef enum FSMStateTypeDef
{
    TEST,
    SERIAL,
    MEMORY_SQUARE,
    MEMORY_TRIANGLE,
    MEMORY_CUBE,
    MEMORY_PATTERN4,
    MEMORY_PATTERN5,
    RGB,
    RB
} FSMStateTypeDef;

typedef enum ParseStatusTypeDef
{
    OK,
    SILENT,
    INVALID_COMMAND,
    INVALID_ARGUMENT,
    TIMEDOUT
} ParseStatusTypeDef;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY3_Pin GPIO_PIN_2
#define KEY3_GPIO_Port GPIOE
#define LASER_Pin GPIO_PIN_12
#define LASER_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_15
#define LED4_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_10
#define LED3_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_11
#define LED2_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_12
#define LED1_GPIO_Port GPIOC
#define KEY1_Pin GPIO_PIN_0
#define KEY1_GPIO_Port GPIOE
#define KEY2_Pin GPIO_PIN_1
#define KEY2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
#define HTIM_ENCX htim5
#define HTIM_ENCY htim3
#define HTIM_MOT htim1
#define HTIM_WS htim4
#define HTIM_CTRL htim7

#define TIM_CHANNEL_WS TIM_CHANNEL_1
#define TIM_CHANNEL_MOT_XA TIM_CHANNEL_1
#define TIM_CHANNEL_MOT_XB TIM_CHANNEL_2
#define TIM_CHANNEL_MOT_YA TIM_CHANNEL_3
#define TIM_CHANNEL_MOT_YB TIM_CHANNEL_4

#define EEPROM_ADDR 0x50
#define CONFIG_BYTE_ADDR 0xff
#define WS2812_PULSE_1 100
#define WS2812_PULSE_0 50
#define MOT_PWM_PERIOD 4800
#define UART_TIMEOUT 250
#define ENCODER_RANGE 510 // ~15 degrees
#define ENCODER_CPR 12240
#define NORM_DIST 218.852233578 // 1 / tan(2pi * range / cpr)
#define RAD_PER_STEPS 5.1333213294E-4
#define STEPS_PER_RAD 1948.05650344
#define ERROR_THRESHOLD 0.01
#define TIMEOUT_PERIOD 10000

#define WsShow() HAL_TIM_PWM_Start_DMA(&HTIM_WS, TIM_CHANNEL_WS, (uint32_t *)WsBuffer, sizeof(WsBuffer))
#define SerialPrint(str, ...)                                                          \
    Uart1.txCounter = sprintf((char *)Uart1.txBuffer, str __VA_OPT__(, ) __VA_ARGS__); \
    HAL_UART_Transmit(&huart1, Uart1.txBuffer, Uart1.txCounter, UART_TIMEOUT);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

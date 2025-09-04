/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <stdint.h>
#include <string.h>
#include "stm32l4xx_nucleo.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* Frame level protocol defines */
#define FRAME_START_BYTE        0xFA
#define FRAME_PAYLOAD_START     0xFB
#define FRAME_STOP_BYTE         0xFE
#define FRAME_PAYLOAD_OFFSET    5
#define FRAME_TAIL_SIZE         2
#define FRAME_HEADER_LSB_IDX    1   // LSB of length
#define FRAME_HEADER_MSB_IDX    2   // MSB of length
#define FRAME_HEADER_CRC_OFFSET 3
#define FRAME_HEADER_SBD_OFFSET 4

/* Protocol Message Types */
#define PROTO_MSG_TYPE_QUERY    0x0B
#define PROTO_MSG_TYPE_STREAM   0x0C
#define PROTO_MSG_TYPE_RESPONSE 0x16
#define PROTO_MSG_TYPE_ERROR    0x21

#define PROTO_MSG_NAME_OFFSET     2
#define QUEUE_MSG_SIZE          128
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

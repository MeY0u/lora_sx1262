/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32u5xx_hal.h"
#include "stm32u5xx_nucleo.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef struct {
    uint8_t buffer[2];
    uint8_t buffer_length;
    int8_t rssi;		// average rssi
    int8_t snr;
    bool new_data;
    int8_t TxCounter;
}LoraRxInfo_t;
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
#define SKU_WAKEUP_Pin GPIO_PIN_3
#define SKU_WAKEUP_GPIO_Port GPIOF
#define SX_LED_RX_Pin GPIO_PIN_0
#define SX_LED_RX_GPIO_Port GPIOC
#define SX_LED_TX_Pin GPIO_PIN_1
#define SX_LED_TX_GPIO_Port GPIOC
#define VBUS_SENSE_Pin GPIO_PIN_2
#define VBUS_SENSE_GPIO_Port GPIOC
#define SX_Mode_SX126X_Pin GPIO_PIN_3
#define SX_Mode_SX126X_GPIO_Port GPIOC
#define SX_Mode_FRx_Pin GPIO_PIN_2
#define SX_Mode_FRx_GPIO_Port GPIOA
#define SX_RESET_Pin GPIO_PIN_3
#define SX_RESET_GPIO_Port GPIOA
#define SX_SPI1_SCK_Pin GPIO_PIN_5
#define SX_SPI1_SCK_GPIO_Port GPIOA
#define SX_SPI1_MISO_Pin GPIO_PIN_6
#define SX_SPI1_MISO_GPIO_Port GPIOA
#define SX_SPI1_MOSI_Pin GPIO_PIN_7
#define SX_SPI1_MOSI_GPIO_Port GPIOA
#define SX_OPT_Pin GPIO_PIN_0
#define SX_OPT_GPIO_Port GPIOB
#define SX_ANT_SW_Pin GPIO_PIN_12
#define SX_ANT_SW_GPIO_Port GPIOF
#define SX_SPI1_CS_Pin GPIO_PIN_13
#define SX_SPI1_CS_GPIO_Port GPIOF
#define SX_DIO1_Pin GPIO_PIN_11
#define SX_DIO1_GPIO_Port GPIOE
#define SX_DIO1_EXTI_IRQn EXTI11_IRQn
#define SX_BUSY_Pin GPIO_PIN_13
#define SX_BUSY_GPIO_Port GPIOE
#define UCPD_FLT_Pin GPIO_PIN_14
#define UCPD_FLT_GPIO_Port GPIOB
#define SKU_RESET_Pin GPIO_PIN_8
#define SKU_RESET_GPIO_Port GPIOC
#define SKU_IRQ_Pin GPIO_PIN_9
#define SKU_IRQ_GPIO_Port GPIOC
#define SKU_SPI3_SCK_Pin GPIO_PIN_10
#define SKU_SPI3_SCK_GPIO_Port GPIOC
#define SKU_SPI3_MISO_Pin GPIO_PIN_11
#define SKU_SPI3_MISO_GPIO_Port GPIOC
#define SKU_SPI3_MOSI_Pin GPIO_PIN_12
#define SKU_SPI3_MOSI_GPIO_Port GPIOC
#define SKU_CS_Pin GPIO_PIN_2
#define SKU_CS_GPIO_Port GPIOD
#define UCPD_DBn_Pin GPIO_PIN_5
#define UCPD_DBn_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

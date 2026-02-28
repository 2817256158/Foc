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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define UO_Pin GPIO_PIN_0
#define UO_GPIO_Port GPIOC
#define VO_Pin GPIO_PIN_1
#define VO_GPIO_Port GPIOC
#define WO_Pin GPIO_PIN_2
#define WO_GPIO_Port GPIOC
#define NTC1_Pin GPIO_PIN_3
#define NTC1_GPIO_Port GPIOC
#define IU_Pin GPIO_PIN_0
#define IU_GPIO_Port GPIOA
#define IV_Pin GPIO_PIN_1
#define IV_GPIO_Port GPIOA
#define IW_Pin GPIO_PIN_2
#define IW_GPIO_Port GPIOA
#define R_IN_Pin GPIO_PIN_3
#define R_IN_GPIO_Port GPIOA
#define IBUS_Pin GPIO_PIN_5
#define IBUS_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_1
#define SPI1_CS_GPIO_Port GPIOB
#define BUTTON4_Pin GPIO_PIN_12
#define BUTTON4_GPIO_Port GPIOB
#define BUTTON3_Pin GPIO_PIN_6
#define BUTTON3_GPIO_Port GPIOC
#define BUTTON2_Pin GPIO_PIN_7
#define BUTTON2_GPIO_Port GPIOC
#define TFT_CS_Pin GPIO_PIN_15
#define TFT_CS_GPIO_Port GPIOA
#define TFT_RES_Pin GPIO_PIN_11
#define TFT_RES_GPIO_Port GPIOC
#define TFT_DC_Pin GPIO_PIN_2
#define TFT_DC_GPIO_Port GPIOD
#define TFT_LEDA_Pin GPIO_PIN_6
#define TFT_LEDA_GPIO_Port GPIOB
#define BUTON1_Pin GPIO_PIN_7
#define BUTON1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

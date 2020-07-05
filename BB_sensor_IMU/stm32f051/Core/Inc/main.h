/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bbsensor_utilites.h"
#include <string.h>
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
void BB_Delay_bytick(uint32_t Delay);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

typedef int32_t fix16_t;
//tick defines
#define DELAY_UNIT_1MS(delay) \
        (BBSYSFRAC == FRAC1MS ?(delay):(delay*2))
#define DELAY_UNIT_500US(delay) \
        (BBSYSFRAC == FRAC500US ?(delay):(delay/2))
#define FRAC500US     (500U)
#define FRAC1MS       (1000U)
#define BBSYSFRAC     FRAC1MS

//debug defines
#define USE_FULL_ASSERT 1
//BB sensor define
#define ICM_SERIF_TYPE ICM426XX_UI_SPI4
#define BB_UART_BRT (512000)
#define I2CTIMEOUT (0x1000)
#define SPITIMEOUT (100)

#define ICM42605_I2CADDRESS   (0x69)   // Address of ICM42605 when ADO = HIGH
#define ICM42605_SPI_CS_PORT  GPIOA
#define ICM42605_SPI_CS_PIN   GPIO_PIN_4
#define BB_DUMMYBYTE 0xBB


#ifndef INV_ABS
#define INV_ABS(x) (((x) < 0) ? -(x) : (x))
#endif //INV_ABS

#define BBINVST 1
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

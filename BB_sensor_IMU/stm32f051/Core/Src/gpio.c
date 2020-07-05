/**
  ******************************************************************************
  * File Name          : gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
/* USER CODE BEGIN 0 */
//extern BB_INV_BOARD_SPI_DEVICE spi_sel_device;
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 as SPI CS*/
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 for debug*/
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 for debug*/
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* USER CODE BEGIN 2 */
int BB_SPI_CS_ENABLE(BB_INV_BOARD_SPI_DEVICE spi_sel_device)
{
  switch (spi_sel_device) {
    case BB_SPI_NONE:
      BB_MSG(BB_MSG_LEVEL_ERROR, "BB_SPI_NONE SPI CS ENABLE FAIL.%s Line %d",__FILE__,__LINE__);
      return -1;
    case BB_ICM42605:
      HAL_GPIO_WritePin(ICM42605_SPI_CS_PORT, ICM42605_SPI_CS_PIN, GPIO_PIN_RESET);
      return 0;
      /***
    case BB_ICM42605:
      HAL_GPIO_WritePin(ICM42605_SPI_CS_PORT, ICM42605_SPI_CS_PIN, GPIO_PIN_RESET);
      return 0;
      ***/
    case BB_SPI_MAX:
      BB_MSG(BB_MSG_LEVEL_ERROR, "BB_SPI_MAX SPI CS ENABLE FAIL.%s Line %d",__FILE__,__LINE__);
      return -1;
    default:
      return -1;
  }
}

int BB_SPI_CS_DISABLE(BB_INV_BOARD_SPI_DEVICE spi_sel_device)
{
  switch (spi_sel_device) {
    case BB_SPI_NONE:
      BB_MSG(BB_MSG_LEVEL_ERROR, "BB_SPI_NONE SPI CS DISABLE FAIL.%s Line %d",__FILE__,__LINE__);
      return -1;
    case BB_ICM42605:
      HAL_GPIO_WritePin(ICM42605_SPI_CS_PORT, ICM42605_SPI_CS_PIN, GPIO_PIN_SET);
      return 0;
      /***
    case BB_ICM42605:
      HAL_GPIO_WritePin(ICM42605_SPI_CS_PORT, ICM42605_SPI_CS_PIN, GPIO_PIN_RESET);
      return 0;
      ***/
    case BB_SPI_MAX:
      BB_MSG(BB_MSG_LEVEL_ERROR, "SPI CS DISABLE FAIL.%s Line %d",__FILE__,__LINE__);
      return -1;
    default:
      return -1;
  }
}

/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

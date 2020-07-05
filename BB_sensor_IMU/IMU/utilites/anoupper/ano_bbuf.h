/**
  ******************************************************************************
  * @file    IMU/algorithm/inc/sensor_algorithm.h
  * @author  Zhang Xiaodong
  * @version V0.1.0
  * @date    2020/5
  * @brief   sensor algorithm header file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2020 BeyondBits</center></h2>
  *
  *
  ******************************************************************************
 */

#include "main.h"

//void send_ANO(uint8_t anoid, int16_t data1, int16_t data2, int16_t data3);

void send_ANO_acc(int32_t *acc);
void trans_Q16_send(fix16_t *acc);

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT BeyondBits *****END OF FILE****/

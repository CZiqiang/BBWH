/**
  ******************************************************************************
  * @file    IMU/algorithm/inc/sensor_algorithm.h
  * @author  Zhang Xiaodong
  * @version V0.1.0
  * @date    11-April-2020
  * @brief   sensor algorithm header file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2020 BeyondBits</center></h2>
  *
  *
  ******************************************************************************
 */

#ifndef __SENSORALGOR_H
#define __SENSORALGOR_H


#define CALISAMPLECNT 50  //calibration sample count


//define the RAW Date read from sensor
typedef struct {
  int16_t xraw;
  int16_t yraw;
  int16_t zraw;
} ThreeAxisDate_TypeDef;


//Accel and Gyro Raw date define
ThreeAxisDate_TypeDef* pAcc_Rawdate; 
ThreeAxisDate_TypeDef* pAcc_cali;

ThreeAxisDate_TypeDef* pCyro_Rawdate;
ThreeAxisDate_TypeDef* pGyro_cali;


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT BeyondBits *****END OF FILE****/

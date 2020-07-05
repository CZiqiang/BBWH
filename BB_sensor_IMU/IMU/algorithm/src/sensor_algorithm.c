/**
  ******************************************************************************
  * @file    IMU/algorithm/src/sensor_algorithm.c
  * @author  Zhang Xiaodong
  * @version V0.1.0
  * @date    11-April-2020
  * @brief   sensor utilities for BBIMUv1.0
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2020 BeyondBits</center></h2>
  *
  *
  ******************************************************************************
 */

 /* Includes ------------------------------------------------------------------*/
#include "sensor_algorithm.h"

/** @addtogroup 
  * @{
  */

/** @addtogroup 
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static __IO uint32_t TimingDelay;

/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
  
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Cal the Zero bias of Accelerometer
  * @param  index:              date count index
            axraw,ayraw,azraw:  Date read from the sensor
            enable:         1 :   count calibration
                                else: set calibration value to 0
  * @retval 0: calibration is ongoing
            1: count calibration done
            2: Dont do calibration ,just set cali to ZERO
  */
uint8_t Acc_Get_Zerobias(uint16_t index, 
                    ThreeAxisRawDate_TypeDef* pAcc_Rawdate,
                    ThreeAxisDate_TypeDef* pAcc_cali;
                    bool enable)
{
  static u8 i=0;
  float normalize;
  static int32_t axsum=0;
  static int32_t aysum=0;
  static int32_t azsum=0;
  
  float ax,ay,az;
  
  if(enable)
    {
      if(index == CALISAMPLECNT)
      {
        ax=axsum/float(CALISAMPLECNT);
        ay=aysum/float(CALISAMPLECNT);
        az=azsum/float(CALISAMPLECNT);

        //normalize = 1/sqrt(ax^2+ay^2+az^2)
        //Q_rsqrt() should be modified to improve performance
        normalize=Q_rsqrt(ax*ax+ay*ay+az*az);
        ax*=normalize;
        ay*=normalize;
        az*=normalize;

        //Update calibration val
        ax_cali += -ax;
        ay_cali += -ay;
        az_cali += 1-az;
        axsum = 0;
        aysum = 0;
        azsum = 0;
        return 1;  
      }
      else
      {
        axsum += pAcc_Rawdate->xraw;
        aysum += pAcc_Rawdate->yraw;
        azsum += pAcc_Rawdate->zraw;
        retun 0;
      }
  
    }
  else//donnt do calibration
  {
    ax_cali =0;
    ay_cali =0;
    az_cali =0;
    retun 2;
  }
 
}
/**
  * @brief  Cal the Zero bias of Gyroscope
  * @param  index:              date count index
            gxraw,gyraw,gzraw:  Date read from the sensor
            enableCali:         1 :   count calibration
                                else: set calibration value to 0
  * @retval 0: calibration is ongoing
            1: count calibration done
            2: Dont do calibration ,just set cali to ZERO
  */
uint8_t Gyro_Get_Zerobias(uint16_t index, 
                      ThreeAxisRawDate_TypeDef* pAcc_Rawdate,
                    ThreeAxisDate_TypeDef* pAcc_cali;
                    uint16_t gxraw,uint16_t gyraw,uint16_t gyraw,bool enable)
{
  static u8 i=0;
  static s32 gxcount=0;
  static s32 gycount=0;
  static s32 gzcount=0;
  if(enable!=1)
  {
    i=0;
    gxcount=0;gycount=0;gzcount=0;
    return 0;
  }
  else if(i<CALISAMPLECNT)
  {
    gxcount+=gx;gycount+=gy;gzcount+=gz;
    i++;
    return 0;
  }
  else
  {
    gxcount/=CALISAMPLECNT;gycount/=CALISAMPLECNT;gzcount/=CALISAMPLECNT;
    gx_cali+=gxcount;
    gy_cali+=gycount;
    gz_cali+=gzcount;
    i=0;
    gxcount=0;gycount=0;gzcount=0;
    return 1;
  }
}

 /**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT BeyondBits *****END OF FILE****/

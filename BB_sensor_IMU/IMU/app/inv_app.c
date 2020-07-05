/*****************************************************************************
 *
 * @file:	D:\prj\git-version\BB_sensor_IMU\IMU\app\inv_app.c
 * @author: Xiaodong Zhang
 * @Email:	sacntumz@foxmail.com;zhangxiaodong@beyondbit.com
 * @date:	2020/06
 * @brief: Applicate for inv

******************************************************************************
*
* Revision History:
*
* Rev.  Date        Who       		Changes
* 1     2020/06   Xiaodong Zhang  		New Created.
******************************************************************************

******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2020 BeyondBit</center></h2>
  *
  *
******************************************************************************/

 
/* Includes ------------------------------------------------------------------*/
#include "inv_app.h"


#include "icm42605SelfTest.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
//Self-Test codes include begin


//self-test codes include end

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static struct inv_icm426xx icm_driver;
/* Private function prototypes -----------------------------------------------*/


/* Private functions ---------------------------------------------------------*/
 

/** @addtogroup 
  * @{
  */

/** @addtogroup 
  * @{
  */


/**
  * @brief  Setup Inv device
  * @param  MemAddress Internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval BB_StatusTypeDef status 
 */ 
BB_StatusTypeDef SetupInvDevice(struct inv_icm426xx_serif * icm_serif)
{
  BB_StatusTypeDef status = BB_NO_ERROR;
  uint8_t who_am_i;
   
  /* Initialize device */
  BB_MSG(BB_MSG_LEVEL_DEBUG, "Initialize Icm426xx...");
  status = inv_icm426xx_init(&icm_driver, icm_serif, NULL);
  if(status != BB_NO_ERROR) {
    BB_MSG(BB_MSG_LEVEL_ERROR, "Fail to initialize Icm426xx.");
    return status;
  }
  BB_MSG(BB_MSG_LEVEL_DEBUG, "...Init Success. RC: %d",status); 

  /* Check WHOAMI */
  BB_MSG(BB_MSG_LEVEL_DEBUG, "Check Icm426xx WhoAmI...");
  
  status = inv_icm426xx_get_who_am_i(&icm_driver, &who_am_i);
  if(status != BB_NO_ERROR) {
    BB_MSG(BB_MSG_LEVEL_ERROR, "Fail when read Icm426xx whoami value.ErCode:%d",status);
    return status;
  }
  
  if(who_am_i != ICM_WHOAMI) {
    BB_MSG(BB_MSG_LEVEL_ERROR, "Bad WHOAMI value. Got 0x%02x (expected: 0x%02x)", who_am_i, ICM_WHOAMI);
    return BB_INV_ERROR;
  }
  BB_MSG(BB_MSG_LEVEL_DEBUG, "...Check WhoAmI Success");
  
  BB_MSG(BB_MSG_LEVEL_DEBUG, "Config Accel&Gyro");
  status |=  inv_icm426xx_ag_config(&icm_driver, 
                      ICM426XX_PWR_MGMT_0_ACCEL_MODE_LN,
                      ICM426XX_PWR_MGMT_0_GYRO_MODE_LN,
                      ICM426XX_ACCEL_CONFIG0_FS_SEL_2g,
                      ICM426XX_GYRO_CONFIG0_FS_SEL_2000dps,
                      ICM426XX_ACCEL_CONFIG0_ODR_200_HZ,
                      ICM426XX_GYRO_CONFIG0_ODR_200_HZ);
  if(status != BB_NO_ERROR) {
    BB_MSG(BB_MSG_LEVEL_ERROR, "Accel&Gyro Config Fail:%d",status);
    return status;
  }

  return status;
}


void RunSelfTestInv(void)
{
	BB_StatusTypeDef status = BB_NO_ERROR;
	uint8_t st_result = 0;

	status = inv_icm426xx_run_selftest(&icm_driver, &st_result);
	BB_MSG(BB_MSG_LEVEL_INFO, "Selftest :%x",st_result);

	if (status != 0) {
		BB_MSG(BB_MSG_LEVEL_ERROR, "An error occured while running selftest");
	} else {
		/* Check for GYR success (1 << 0) and ACC success (1 << 1) */
		if (st_result & 0x1)
			BB_MSG(BB_MSG_LEVEL_INFO, "Gyro Selftest PASS");
		else
			BB_MSG(BB_MSG_LEVEL_INFO, "Gyro Selftest FAIL");
		
		if (st_result & 0x2)
			BB_MSG(BB_MSG_LEVEL_INFO, "Accel Selftest PASS");
		else
			BB_MSG(BB_MSG_LEVEL_INFO, "Accel Selftest FAIL");
	} 
}

/**
  * @brief  
  * @param              
  * @retval 
 */ 
void GetInvBias(void)
{
	uint8_t rc = 0;
	int raw_bias[6];

	/* Get Low Noise / Low Power bias computed by self-tests scaled by 2^16 */
	BB_MSG(BB_MSG_LEVEL_INFO, "Getting ST bias");
	rc =inv_icm426xx_get_st_bias(&icm_driver, raw_bias);

	if ( rc == 1)
    BB_MSG(BB_MSG_LEVEL_ERROR, "!Get bias while Selftest didn't run!");
	else if(rc == 0)
    {
      BB_MSG(BB_MSG_LEVEL_INFO, "GYR LN bias (dps): x=%f, y=%f, z=%f",
        (float)(raw_bias[0]) / (float)(1 << 16), (float)(raw_bias[1]) / (float)(1 << 16), (float)(raw_bias[2]) / (float)(1 << 16));
      BB_MSG(BB_MSG_LEVEL_INFO, "ACC LN bias (g): x=%f, y=%f, z=%f",
        (float)(raw_bias[0 + 3] / (float)(1 << 16)), (float)(raw_bias[1 + 3] / (float)(1 << 16)), (float)(raw_bias[2 + 3] / (float)(1 << 16)));
    }
    
}

 
/**
  * @}
  */

/**
  * @}
  */



/*****END OF FILE****/

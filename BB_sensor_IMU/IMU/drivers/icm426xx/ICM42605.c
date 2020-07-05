/*****************************************************************************
 *
 * @file:	D:\prj\BBSensor\IMU\drivers\ICM42605.c
 * @author: Xiaodong Zhang
 * @Email:	sacntumz@foxmail.com;zhangxiaodong@beyondbit.com
 * @date:	2020/05
 * @brief: ICM42605 hal functions

******************************************************************************
*
* Revision History:
*
* Rev.  Date        Who       		Changes
* 1     2020/05   Xiaodong Zhang  		New Created.
* 2     2020/06   Xiaodong Zhang      Add more functions.
******************************************************************************

******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2020 BeyondBit</center></h2>
  *
  *
******************************************************************************/

 
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "ICM42605.h"

#include "spi.h"
#include "i2c.h"
#include "gpio.h"
/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/
/*define ICM interface*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Just a handy variable to handle the icm426xx object */



/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static BB_StatusTypeDef inv_icm426xx_configure_serial_interface(struct inv_icm426xx * s);
static int inv_icm426xx_init_hardware_from_ui(struct inv_icm426xx * s);

/** @addtogroup ICM42605_Interface
  * @{
  */

/**
  * @brief  Set icm42605 register bank, this function only can be used after inv_icm426xx_init
  * @param  :inv_icm426xx pointer
  *          bank pointer
  * @retval BB_StatusTypeDef
 */ 
BB_StatusTypeDef inv_icm426xx_set_reg_bank(struct inv_icm426xx * s, uint8_t bank)
{
  BB_StatusTypeDef status = BB_NO_ERROR;
  struct inv_icm426xx_transport *t = (struct inv_icm426xx_transport *)s;

  if(s->current_reg_bank == bank)
    return status;
  else
  {
    s->current_reg_bank = bank;
    return t->serif.write_reg(&(t->serif), MPUREG_REG_BANK_SEL, &bank ,1);
  }
}
 

/**
  * @brief  icm42605 reg read
  *         -support single read burst read by set rlen
  *         -support burst read with address self-increasing
  * @param  :
  *         - inv_icm426xx_serif * serif: serials inteface struct pointer for icm42605.
  *         - uint8_t reg:reg address val
  *         - uint8_t* rbuffer: read buffer pointer to store the read-out data
  *         - uint16_t rlen: bytes that should be read out from the reg address
  * @retval : BB_Status
*/ 
 
BB_StatusTypeDef inv_io_hal_read_reg(struct inv_icm426xx_serif * serif, uint8_t reg, uint8_t * rbuffer, uint8_t rlen)
{
  HAL_StatusTypeDef hal_status;
  uint16_t spitransize=0;
  uint16_t i = 1;
  switch (serif->serif_type) {

    case ICM426XX_UI_SPI4:
      spitransize = rlen + 1;
      //full fill txbuffer with regval 
      //support burst read
      if(spitransize > ICM4_REGRDMAX )//rlen is bigger than max read
        return BB_ERROR_SIZE;

//      serif->Txbuff[0] = reg | 0x80;

      for(i=0;i<rlen;i++)
      {
        serif->Txbuff[i] = (reg + i) | 0x80;
      }
      serif->Txbuff[rlen] = BB_DUMMYBYTE;
      
      // memset(&serif->Txbuff[1], BB_DUMMYBYTE, 1);

      hal_status = inv_spi_master_rw_register(BB_ICM42605, &serif->Txbuff[0], &serif->Rxbuff[0],spitransize);
      
      if(hal_status == HAL_OK) 
        {
          memcpy(rbuffer,&serif->Rxbuff[1], rlen);
          return BB_NO_ERROR;
        }
      else if (hal_status == HAL_ERROR)
        {
          BB_MSG(BB_MSG_LEVEL_ERROR,"SPI HAL ERROR. %s Line:%d",__FILE__,__LINE__);
          return BB_ERROR_HW;
        }
      else if (hal_status == HAL_BUSY)
        {
          BB_MSG(BB_MSG_LEVEL_ERROR,"SPI HAL BUSY.  %s Line:%d",__FILE__,__LINE__);      
          return BB_ERROR_HW;
        }
      else if (hal_status == HAL_TIMEOUT)
        {
          BB_MSG(BB_MSG_LEVEL_ERROR,"SPI HAL TIMEOUT. %s Line:%d",__FILE__,__LINE__);
          return BB_ERROR_TIMEOUT;
        }

    case ICM426XX_UI_I2C:
      hal_status = inv_i2c_readData((ICM42605_I2CADDRESS<<1),reg,rbuffer,rlen);
      if(hal_status == HAL_OK) 
          return BB_NO_ERROR;
      else if (hal_status == HAL_ERROR)
        {
          BB_MSG(BB_MSG_LEVEL_ERROR,"I2C HAL ERROR. %s Line:%d",__FILE__,__LINE__);
          return BB_ERROR_HW;
        }
      else if (hal_status == HAL_BUSY)
        {
          BB_MSG(BB_MSG_LEVEL_ERROR,"I2C HAL BUSY.  %s Line:%d",__FILE__,__LINE__);      
          return BB_ERROR_HW;
        }
      else if (hal_status == HAL_TIMEOUT)
        {
          BB_MSG(BB_MSG_LEVEL_ERROR,"I2C HAL TIMEOUT. %s Line:%d",__FILE__,__LINE__);
          return BB_ERROR_TIMEOUT;
        }
    default:
      return BB_ERROR_IO;
  }
  
}



/**
  * @brief  icm42605 reg write , support single read and burst read by set wlen
  * @param  :
  *           inv_icm426xx_serif * serif: serials inteface struct pointer for icm42605.
  * @retval 
 */ 
BB_StatusTypeDef inv_io_hal_write_reg(struct inv_icm426xx_serif * serif, uint8_t reg, uint8_t * wbuffer, uint16_t wlen)
{
  HAL_StatusTypeDef hal_status;
  uint16_t transize=0;
  uint16_t i=1;
  uint32_t index;

  switch (serif->serif_type) 
  {
    case ICM426XX_UI_SPI4:
      transize = wlen<<1;

      //if(transize > (ICM4_REGWRMAX-1))
      //return BB_ERROR_SIZE;

      //full fill txbuffer with regval 
      //support burst read
      if(transize > ICM4_REGWRMAX )//wlen is bigger than max write buffer length
        return BB_ERROR_SIZE;
      
      serif->Txbuff[0] = reg;
      serif->Txbuff[1] = *wbuffer;

      if(transize > 2)//write more than one byte from one register address
      {
        for(i=1;i<wlen;i++)
          {
            index = i<<1;//i * 2
            serif->Txbuff[index] = serif->Txbuff[index-2]+1;//formate reg address
            serif->Txbuff[index+1] = *(wbuffer+i);//formate reg write val
          }
      }
      
      hal_status = inv_spi_master_rw_register(BB_ICM42605, &serif->Txbuff[0], &serif->Rxbuff[0],transize);

      if(hal_status == HAL_OK)
        return BB_NO_ERROR;
      else if (hal_status == HAL_ERROR)
        {
          BB_MSG(BB_MSG_LEVEL_ERROR,"SPI HAL ERROR. %s Line:%d",__FILE__,__LINE__);
          return BB_ERROR_HW;
        }
      else if (hal_status == HAL_BUSY)
        {
          BB_MSG(BB_MSG_LEVEL_ERROR,"SPI HAL BUSY. %s Line:%d",__FILE__,__LINE__);      
          return BB_ERROR_HW;
        }
      else if (hal_status == HAL_TIMEOUT)
        {
          BB_MSG(BB_MSG_LEVEL_ERROR,"SPI HAL TIMEOUT.%s Line:%d",__FILE__,__LINE__);
          return BB_ERROR_TIMEOUT;
        }
    
    case ICM426XX_UI_I2C:
      hal_status = inv_i2c_writeData((ICM42605_I2CADDRESS<<1),reg,wbuffer,wlen);

      if(hal_status == HAL_OK)
        return BB_NO_ERROR;
      else if (hal_status == HAL_ERROR)
        {
          BB_MSG(BB_MSG_LEVEL_ERROR,"I2C HAL ERROR. %s Line:%d",__FILE__,__LINE__);
          return BB_ERROR_HW;
        }
      else if (hal_status == HAL_BUSY)
        {
          BB_MSG(BB_MSG_LEVEL_ERROR,"I2C HAL BUSY.  %s Line:%d",__FILE__,__LINE__);      
          return BB_ERROR_HW;
        }
      else if (hal_status == HAL_TIMEOUT)
        {
          BB_MSG(BB_MSG_LEVEL_ERROR,"I2C HAL TIMEOUT. %s Line:%d",__FILE__,__LINE__);
          return BB_ERROR_TIMEOUT;
        }

    default:
      return BB_ERROR_IO;
  }
}

/**
  * @brief :Get icm426xx ID
  * @param :
  *     struct inv_icm426xx * s: struct pointer for icm42605.
  *     uint8_t * who_am_i:pointer to store read out id
  * @retval: status
 */ 
BB_StatusTypeDef inv_icm426xx_get_who_am_i(struct inv_icm426xx * s, uint8_t * who_am_i)
{
  struct inv_icm426xx_transport *t = (struct inv_icm426xx_transport *)s;

  return t->serif.read_reg(&(t->serif), MPUREG_WHO_AM_I, who_am_i ,1);
}

/**
  * @brief :This function was designed to init icm42605 after power up
  *       ,so 300ms delay is needed to make sure icm42605 is stablized.
  *       1. Init inv_icm426xx st;
  *       2. link inv_icm426xx_serif st to inv_icm426xx
  *       3. set default value for inv_icm426xx st
  *       4. set default value for inv_icm426xx st
  *       5. set register according to the serif define
  *       6. Accel & gyro work mode, odr and fsr set
  * @param :
  *     struct inv_icm426xx * s: struct pointer for icm42605.
  *     struct inv_icm426xx_serif * serif: struct pointer for icm426 IF
  *     void (*sensor_event_cb)(inv_icm426xx_sensor_event_t * event):
  *         callback function point
  * @retval: status
 */ 
BB_StatusTypeDef inv_icm426xx_init(struct inv_icm426xx * s, struct inv_icm426xx_serif * serif, void (*sensor_event_cb)(inv_icm426xx_sensor_event_t * event))
{
	BB_StatusTypeDef status = BB_NO_ERROR;

	memset(s, 0, sizeof(*s));
	
	s->transport.serif = *serif;
	
	/* Wait some time for ICM to be properly supplied */
	HAL_Delay(300);

  /* First data are noisy after enabling sensor
   * This variable keeps track of gyro start time. Set to UINT32_MAX at init 
   */
  s->gyro_start_time_us = UINT32_MAX;
  /* First data are noisy after enabling sensor
   * This variable keeps track of accel start time. Set to UINT32_MAX at init 
   */
  s->accel_start_time_us = UINT32_MAX;

  s->current_reg_bank = 0XFF;

	/* Gyro power-off to power-on transition can cause ring down issue
   * This variable keeps track of timestamp when gyro is power off. Set to UINT32_MAX at init
   */
  s->gyro_power_off_tmst = UINT32_MAX;

	if((status |= inv_icm426xx_configure_serial_interface(s)) != 0 )
		return status;

	/* Register the callback to be executed each time inv_icm426xx_get_data_from_fifo extracts 
	 * a packet from fifo or inv_icm426xx_get_data_from_registers read data 
	 */
  s->sensor_event_cb = sensor_event_cb;
	
	/* initialize hardware */
	status |= inv_icm426xx_init_hardware_from_ui(s);
	

//  status |=  inv_icm426xx_ag_config(s, 
//                      ICM426XX_PWR_MGMT_0_ACCEL_MODE_LN,
//                      ICM426XX_PWR_MGMT_0_GYRO_MODE_LN,
//                      ICM426XX_ACCEL_CONFIG0_FS_SEL_2g,
//                      ICM426XX_GYRO_CONFIG0_FS_SEL_2000dps,
//                      ICM426XX_ACCEL_CONFIG0_ODR_200_HZ,
//                      ICM426XX_GYRO_CONFIG0_ODR_200_HZ);

	return status;
}

/**
  * @brief Set accel fsr&freq
  * @param :
  * - inv_icm426xx * s
  * - ICM426XX_ACCEL_CONFIG0_FS_SEL_t acc_fsr_g
  * - ICM426XX_ACCEL_CONFIG0_ODR_t acc_freq
  * @retval status
*/ 
BB_StatusTypeDef inv_icm426xx_set_accel_fsr_freq(struct inv_icm426xx * s, 
              ICM426XX_ACCEL_CONFIG0_FS_SEL_t acc_fsr_g,
              ICM426XX_ACCEL_CONFIG0_ODR_t acc_freq)
{

  //assert_param(acc_fsr_g == ICM426XX_ACCEL_CONFIG0_FS_DONTOUCH);

  struct inv_icm426xx_transport *t = (struct inv_icm426xx_transport *)s;

  BB_StatusTypeDef status = BB_NO_ERROR;
  uint8_t temp_reg;

  status |= inv_icm426xx_set_reg_bank(s,0);

  status |= t->serif.read_reg(&(t->serif),MPUREG_ACCEL_CONFIG0,&temp_reg,1);
  if(acc_fsr_g != ICM426XX_ACCEL_CONFIG0_FS_DONTOUCH)
  {  
    temp_reg &= (uint8_t)~BIT_ACCEL_CONFIG0_FS_SEL_MASK;
    temp_reg |= (uint8_t)acc_fsr_g;
  }

  if(acc_freq != ICM426XX_ACCEL_CONFIG0_ODR_DONTOUCH)
    {
      temp_reg &= (uint8_t)~BIT_ACCEL_CONFIG0_ODR_MASK;
      temp_reg |= (uint8_t)acc_freq;
    }
  status |=  t->serif.write_reg(&(t->serif), MPUREG_ACCEL_CONFIG0, &temp_reg ,1);
  return status;
}


/**
  * @brief Set gyro fsr&freq
  * @param :
  * - inv_icm426xx * s
  * - ICM426XX_GYRO_CONFIG0_FS_SEL_t gyro_fsr_dps,
  * - ICM426XX_GYRO_CONFIG0_ODR_t gyr_freq
  * @retval status
*/ 
BB_StatusTypeDef inv_icm426xx_set_gyro_fsr_freq(struct inv_icm426xx * s,
            ICM426XX_GYRO_CONFIG0_FS_SEL_t gyro_fsr_dps,
            ICM426XX_GYRO_CONFIG0_ODR_t gyr_freq)
{

  //assert_param(gyro_fsr_dps == ICM426XX_GYRO_CONFIG0_FS_DONTOUCH);

  struct inv_icm426xx_transport *t = (struct inv_icm426xx_transport *)s;

  BB_StatusTypeDef status = BB_NO_ERROR;
  uint8_t temp_reg;

  status |= inv_icm426xx_set_reg_bank(s,0);

  status |= t->serif.read_reg(&(t->serif),MPUREG_GYRO_CONFIG0,&temp_reg,1);

  if(gyro_fsr_dps != ICM426XX_GYRO_CONFIG0_FS_DONTOUCH)
    {
      temp_reg &= (uint8_t)~BIT_GYRO_CONFIG0_FS_SEL_MASK;
      temp_reg |= (uint8_t)gyro_fsr_dps;
    }

  if(gyr_freq != ICM426XX_GYRO_CONFIG0_ODR_DONTOUCH)
  {
    temp_reg &= (uint8_t)~BIT_GYRO_CONFIG0_ODR_MASK;
    temp_reg |= (uint8_t)gyr_freq;
  }
  status |= t->serif.write_reg(&(t->serif), MPUREG_GYRO_CONFIG0, &temp_reg ,1);

  return status;
}

/**
  * @brief:Set accel low noise mode
  * @param :
  * - inv_icm426xx * s
  * @retval: status
*/ 
BB_StatusTypeDef inv_icm426xx_enable_accel_low_noise_mode(struct inv_icm426xx * s)
{
  struct inv_icm426xx_transport *t = (struct inv_icm426xx_transport *)s;

  BB_StatusTypeDef status = BB_NO_ERROR;
  uint8_t data;
  uint8_t pwr_mgmt0_reg;
  status |= inv_icm426xx_set_reg_bank(s,0);

  /* Restore filter BW settings */
  status |= t->serif.read_reg(&(t->serif), MPUREG_ACCEL_GYRO_CONFIG0, &data, 1);
  data &= (uint8_t)~BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_MASK;
  data |= s->avg_bw_setting.acc_ln_bw;
  status |= t->serif.write_reg(&(t->serif), MPUREG_ACCEL_GYRO_CONFIG0,&data, 1);
  
  /* Enable/Switch the accelerometer in/to low noise mode */
  status |= t->serif.read_reg(&(t->serif), MPUREG_PWR_MGMT_0, &pwr_mgmt0_reg, 1);
  pwr_mgmt0_reg &= (uint8_t)~BIT_PWR_MGMT_0_ACCEL_MODE_MASK;
  pwr_mgmt0_reg |= (uint8_t)ICM426XX_PWR_MGMT_0_ACCEL_MODE_LN;
  status |= t->serif.write_reg(&(t->serif), MPUREG_PWR_MGMT_0, &pwr_mgmt0_reg, 1);
  s->accel_status = ICM426XX_PWR_MGMT_0_ACCEL_MODE_LN;
  HAL_Delay(1);//only 200us delay is needed

  return status;
}

/**
  * @brief:enable accel low power mode
  * @param :
  * - inv_icm426xx * s
  * @retval: status
*/ 
BB_StatusTypeDef inv_icm426xx_enable_accel_low_power_mode(struct inv_icm426xx * s)
{
 struct inv_icm426xx_transport *t = (struct inv_icm426xx_transport *)s;

  BB_StatusTypeDef status = BB_NO_ERROR;
  uint8_t data;
  status |= inv_icm426xx_set_reg_bank(s,0);
  /* Restore filter averaging settings */
  status |= t->serif.read_reg(&(t->serif), MPUREG_ACCEL_GYRO_CONFIG0, &data, 1);
  data &= (uint8_t)~BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_MASK;
  data |= s->avg_bw_setting.acc_lp_avg;
  status |= t->serif.write_reg(&(t->serif), MPUREG_ACCEL_GYRO_CONFIG0,&data, 1);
  
  /* Enable/Switch the accelerometer in/to low power mode */
  status |= t->serif.read_reg(&(t->serif), MPUREG_PWR_MGMT_0, &data, 1);
  data &= ~BIT_PWR_MGMT_0_ACCEL_MODE_MASK;
  data |= ICM426XX_PWR_MGMT_0_ACCEL_MODE_LP;
  status |= t->serif.write_reg(&(t->serif), MPUREG_PWR_MGMT_0,&data, 1);
  s->accel_status = ICM426XX_PWR_MGMT_0_ACCEL_MODE_LP;
  HAL_Delay(1);//only 200us delay is needed
  
  return status;
}

/**
  * @brief enable gyro ln mode
  * @param :
  * - inv_icm426xx * s
  * @retval : status
*/ 
BB_StatusTypeDef inv_icm426xx_enable_gyro_low_noise_mode(struct inv_icm426xx * s)
{
  struct inv_icm426xx_transport *t = (struct inv_icm426xx_transport *)s;
  
  BB_StatusTypeDef status = BB_NO_ERROR;
  uint8_t data;
  uint8_t pwr_mngt_0_reg;
  //uint64_t current_time;
  uint64_t current_tick;

  /* Powering the gyroscope on immediately after powering it off can result in device failure. 
   * The gyroscope proof mass can continue vibrating after it has been powered off, 
   * and powering it back on immediately can result in unpredictable proof mass movement.
   * After powering the gyroscope off, a period of > 150ms should be allowed to elapse before it is powered back on. */
  if (s->gyro_power_off_tmst != UINT32_MAX) {
    current_tick = HAL_GetTick();
    /* Handle rollover */
    if (current_tick <= s->gyro_power_off_tmst)
      current_tick += UINT32_MAX;
    /* If 150 ms are not elapsed since power-off error is returned */
    if ((current_tick - s->gyro_power_off_tmst) <= (150))
      {
        BB_MSG(BB_MSG_LEVEL_ERROR,"150ms should be wait after power off GYRO");
            return BB_ERROR_HW;
      }
  }
  
  status |= inv_icm426xx_set_reg_bank(s,0);
  /* Restore filter BW settings */
  status |= t->serif.read_reg(&(t->serif), MPUREG_ACCEL_GYRO_CONFIG0, &data, 1);;
  data &= (uint8_t)~BIT_GYRO_ACCEL_CONFIG0_GYRO_FILT_MASK;
  data |= s->avg_bw_setting.gyr_ln_bw;
  status |= t->serif.write_reg(&(t->serif), MPUREG_ACCEL_GYRO_CONFIG0, &data, 1);

  /* Enable/Switch the gyroscope in/to low noise mode */
  status |= t->serif.read_reg(&(t->serif), MPUREG_PWR_MGMT_0, &pwr_mngt_0_reg, 1);
  pwr_mngt_0_reg &= (uint8_t)~BIT_PWR_MGMT_0_GYRO_MODE_MASK;
  pwr_mngt_0_reg |= (uint8_t)ICM426XX_PWR_MGMT_0_GYRO_MODE_LN;
  status |= t->serif.write_reg(&(t->serif), MPUREG_PWR_MGMT_0, &pwr_mngt_0_reg, 1);
  s->gyro_status = ICM426XX_PWR_MGMT_0_GYRO_MODE_LN;
  HAL_Delay(1);//only need 200us

  return status;
}

/**
  * @brief config accel&gyro configure
  * @param :
  * - inv_icm426xx * s
  * - const ICM426XX_ACCEL_CONFIG0_ODR_t frequency
  * @retval 
*/ 
BB_StatusTypeDef inv_icm426xx_ag_config(struct inv_icm426xx * s, 
                      ICM426XX_PWR_MGMT_0_ACCEL_MODE_t accel_mode,
                      ICM426XX_PWR_MGMT_0_GYRO_MODE_t gyro_mode,
                      ICM426XX_ACCEL_CONFIG0_FS_SEL_t acc_fsr_g,
                      ICM426XX_GYRO_CONFIG0_FS_SEL_t gyro_fsr_dps,
                      ICM426XX_ACCEL_CONFIG0_ODR_t acc_freq,
                      ICM426XX_GYRO_CONFIG0_ODR_t gyr_freq)
{
  //donnt support accel off in this function
  //only support gyro LN mode in this function
  assert_param((accel_mode == ICM426XX_PWR_MGMT_0_ACCEL_MODE_OFF) \
  ||(gyro_mode != ICM426XX_PWR_MGMT_0_GYRO_MODE_LN));

//  struct inv_icm426xx_transport *t = (struct inv_icm426xx_transport *)s;

  BB_StatusTypeDef status = BB_NO_ERROR;

 //set accel fsr&freq
  status |= inv_icm426xx_set_accel_fsr_freq(s,acc_fsr_g,acc_freq);

  //set gyro fsr&freq
  status |= inv_icm426xx_set_gyro_fsr_freq(s,gyro_fsr_dps,gyr_freq);

  if(accel_mode == ICM426XX_PWR_MGMT_0_ACCEL_MODE_LN)//LN mode
    status |= inv_icm426xx_enable_accel_low_noise_mode(s);
  else if(accel_mode == ICM426XX_PWR_MGMT_0_ACCEL_MODE_LP)//LP mode
    status |= inv_icm426xx_enable_accel_low_power_mode(s);
  
  status |= inv_icm426xx_enable_gyro_low_noise_mode(s);
  
  return status;
}

/**
  * @brief:inv_icm426xx_device_reset
  * @param:struct inv_icm426xx * s     
  * @retval :BB_StatusTypeDef status
 */
BB_StatusTypeDef inv_icm426xx_device_reset(struct inv_icm426xx * s)
{
  BB_StatusTypeDef status = BB_NO_ERROR;
  uint8_t data;
  uint8_t intf_cfg4_reg, intf_cfg6_reg;
  struct inv_icm426xx_transport *t = (struct inv_icm426xx_transport *)s;
  
  BB_MSG(BB_MSG_LEVEL_DEBUG, "Reset Icm426xx...");
  /* Set memory bank 1 */
  status |= inv_icm426xx_set_reg_bank(s, 1);
  /* save registers necessary to perform soft reset while still keeping communication link alive */
  status |= t->serif.read_reg(&(t->serif), MPUREG_INTF_CONFIG4_B1, &intf_cfg4_reg, 1); // AUX SPI and AP SPI fields
  status |= t->serif.read_reg(&(t->serif), MPUREG_INTF_CONFIG6_B1, &intf_cfg6_reg, 1);
  /* Set memory bank 0 */
  status |= inv_icm426xx_set_reg_bank(s, 0);
  
  /* Reset the internal registers and restores the default settings.
   * The bit automatically clears to 0 once the reset is done.
   * Since soft-reset will trigger OTP reload, SPI mode (bit4) does not need saving
   */
  data = ICM426XX_DEVICE_CONFIG_RESET_EN;
  status |= t->serif.write_reg(&(t->serif), MPUREG_DEVICE_CONFIG, &data, 1);
  if(status)
    return status;

  /* Wait 1ms for soft reset to be effective before trying to perform any further read */
  HAL_Delay(1);

  status |= inv_icm426xx_set_reg_bank(s, 1);
  status |=  t->serif.write_reg(&(t->serif), MPUREG_INTF_CONFIG4_B1, &intf_cfg4_reg, 1);
  status |= inv_icm426xx_set_reg_bank(s, 0);

  /* Check the Int Reset Done bit */
  status |= t->serif.read_reg(&(t->serif), MPUREG_INT_STATUS, &data, 1);
  if (0 == (data & BIT_INT_STATUS_RESET_DONE)) {
    BB_MSG(BB_MSG_LEVEL_ERROR,"Read Reset Done bit error:0x%2x",data);
    return BB_ERROR_UNEXPECTED;
  }

  if(status != BB_NO_ERROR){
    BB_MSG(BB_MSG_LEVEL_ERROR,"!!!!!!!!!\r\nRest FAIL: 0x%2x \r\n!!!!!!!!!",status);
    return status;
  }
  status |= inv_icm426xx_set_reg_bank(s, 1);
  status |= t->serif.write_reg(&(t->serif),  MPUREG_INTF_CONFIG6_B1, &intf_cfg6_reg, 1);

/* Configure FSYNC on INT2=pin 9 */
//  status |= t->serif.read_reg(&(t->serif), MPUREG_INTF_CONFIG5, 1, &d_B1ata);
//  data &= (uint8_t)~BIT_INTF_CONFIG5_GPIO_PAD_SEL_MASK;
//  data |= (1 << BIT_INTF_CONFIG5_GPIO_PAD_SEL_POS);
//  status |= t->serif.write_reg(&(t->serif), MPUREG_INTF_CONFIG5_B1, 1, &data);
//  status |= inv_icm426xx_set_reg_bank(s, 0);

  /* Read and set endianess for further processing */
  status |= inv_icm426xx_set_reg_bank(s, 0);
  status |= t->serif.read_reg(&(t->serif), MPUREG_INTF_CONFIG0, &data, 1);
  s->endianess_data = data & BIT_DATA_ENDIAN_MASK;

  //if(s->transport.serif.serif_type == ICM426XX_UI_I3C){
  //  status |= s->transport.serif.configure((struct inv_icm426xx_serif *)s);
  //}
  BB_MSG(BB_MSG_LEVEL_DEBUG, "...DONE");
  return status;
}


/**
  * @brief:configure fifo 
  *   - fifo_config enable： data are comming from fifo and Interrupt is configured on fifo watermark
  *   - fifo_config disable： data are comming from registers and Interrupt is configured on Data ready
on Fifo Watermark
  * @param:
  *       - struct inv_icm426xx * s     
  *       - INV_ICM426XX_FIFO_CONFIG_t fifo_config
  * @retval :BB_StatusTypeDef status
 */
BB_StatusTypeDef inv_icm426xx_configure_fifo(struct inv_icm426xx * s, INV_ICM426XX_FIFO_CONFIG_t fifo_config)
{
  BB_StatusTypeDef status = BB_NO_ERROR;
  uint8_t data;
  inv_icm426xx_interrupt_parameter_t config_int = {(inv_icm426xx_interrupt_value)0};
  struct inv_icm426xx_transport *t = (struct inv_icm426xx_transport *)s;

  s->fifo_is_used = fifo_config;
  
  switch (fifo_config) {
  
    case INV_ICM426XX_FIFO_ENABLED :
      // Configure:
      // - FIFO record mode ,while FIFO count unit is PACKET 
      // - FIFO_MODE : stop on full mode :drop the data when the FIFO overflows
      // - Timestamp is logged in FIFO// maybe comment in future by zxd
      // - Little Endian fifo_count
      //
      status |= t->serif.read_reg(&(t->serif), MPUREG_INTF_CONFIG0, &data,1);
      data |= (uint8_t)ICM426XX_INTF_CONFIG0_FIFO_COUNT_REC_RECORD;//fifo cnt in packet
      data &= (uint8_t)~BIT_FIFO_COUNT_ENDIAN_MASK; // set to little endian
      status |= t->serif.write_reg(&(t->serif), MPUREG_INTF_CONFIG0,&data,1);
      data = (uint8_t)ICM426XX_FIFO_CONFIG_MODE_STOP_ON_FULL;//set to STOP_ON_FULL
      status |= t->serif.write_reg(&(t->serif), MPUREG_FIFO_CONFIG, &data,1);
      
      status |= t->serif.read_reg(&(t->serif), MPUREG_TMST_CONFIG,  &data,1);
      data |= ICM426XX_TMST_CONFIG_TMST_EN;//Enabale Time Stamp register
      status |= t->serif.write_reg(&(t->serif), MPUREG_TMST_CONFIG, &data,1);


      status |= t->serif.read_reg(&(t->serif), MPUREG_FIFO_CONFIG1, &data, 1);
      //data |= (BIT_FIFO_CONFIG1_GYRO_MASK | BIT_FIFO_CONFIG1_ACCEL_MASK | BIT_FIFO_CONFIG1_TEMP_MASK | BIT_FIFO_CONFIG1_TMST_FSYNC_MASK);

       
      //Enable temperature sensor packets to FIFO
      //Enable gyro sensor packets to FIFO
      //Enable accel sensor packets to FIFO
      //by OR with MASK!!!! WTF coding style.
      //But if you want to use ICM426XX_FIFO_CONFIG1_GYRO_EN or xxx_EN, you must add (uint8_t)before these value to coerce the type!!!
      
      data |= (BIT_FIFO_CONFIG1_GYRO_MASK | BIT_FIFO_CONFIG1_ACCEL_MASK | BIT_FIFO_CONFIG1_TEMP_MASK | BIT_FIFO_CONFIG1_TMST_FSYNC_MASK);
      //Set fifo_wm_int_w generating condition : fifo_wm_int_w generated when counter >= threshold 
      data |= (uint8_t)ICM426XX_FIFO_CONFIG1_WM_GT_TH_EN;
      status |= t->serif.write_reg(&(t->serif), MPUREG_FIFO_CONFIG1, &data,1);
      /* Configure FIFO watermark so that INT is triggered for each packet */
      data = 0x1;
      status |= t->serif.write_reg(&(t->serif), MPUREG_FIFO_CONFIG2, &data,1);

      // Disable Data Ready Interrupt 
      status |= inv_icm426xx_get_config_int1(s, &config_int);
      config_int.INV_ICM426XX_UI_DRDY = INV_ICM426XX_DISABLE;
      status |= inv_icm426xx_set_config_int1(s, &config_int);
      //for debug
      status |= inv_icm426xx_get_config_int1(s, &config_int);
      break;
    
    case INV_ICM426XX_FIFO_DISABLED :
      /* make sure FIFO is disabled */
      data = ICM426XX_FIFO_CONFIG_MODE_BYPASS;
      status |= t->serif.write_reg(&(t->serif), MPUREG_FIFO_CONFIG, &data,1);
      
      /* restart and reset FIFO configuration */
      status |= t->serif.read_reg(&(t->serif), MPUREG_FIFO_CONFIG1, &data,1);
      data &= (uint8_t)~(BIT_FIFO_CONFIG1_GYRO_MASK | BIT_FIFO_CONFIG1_ACCEL_MASK);
      data |= (BIT_FIFO_CONFIG1_TEMP_MASK | BIT_FIFO_CONFIG1_TMST_FSYNC_MASK);
      status |= t->serif.write_reg(&(t->serif), MPUREG_FIFO_CONFIG1,&data,1);
      
      /* Enable Data Ready Interrupt */
      status |= inv_icm426xx_get_config_int1(s, &config_int);
      config_int.INV_ICM426XX_UI_DRDY = INV_ICM426XX_ENABLE;
      status |= inv_icm426xx_set_config_int1(s, &config_int);
      break;

    default :
      status = BB_INV_ERROR;
  }
  return status;
}

BB_StatusTypeDef inv_icm426xx_set_config_int1(struct inv_icm426xx * s, inv_icm426xx_interrupt_parameter_t * interrupt_to_configure)
{
  BB_StatusTypeDef status = BB_NO_ERROR;
  uint8_t data[3] = {0};
  struct inv_icm426xx_transport *t = (struct inv_icm426xx_transport *)s;

  status |= t->serif.read_reg(&(t->serif), MPUREG_INT_SOURCE0, data, 2); /* burst read int_source0/int_source1 */
  status |= inv_icm426xx_set_reg_bank(s, 4);
  status |= t->serif.read_reg(&(t->serif), MPUREG_INT_SOURCE6_B4, &data[2], 1); /* switch to bank4 for int_source6 */

  /* Set INT_SOURCE0 bits */
  data[0] &= (uint8_t)~(BIT_INT_SOURCE0_UI_FSYNC_INT1_EN 
      | BIT_INT_SOURCE0_UI_DRDY_INT1_EN 
      | BIT_INT_SOURCE0_FIFO_THS_INT1_EN 
      | BIT_INT_SOURCE0_FIFO_FULL_INT1_EN);
  data[0] |= ((interrupt_to_configure->INV_ICM426XX_UI_FSYNC != 0)  << BIT_INT_UI_FSYNC_INT_EN_POS);
  data[0] |= ((interrupt_to_configure->INV_ICM426XX_UI_DRDY != 0)   << BIT_INT_UI_DRDY_INT_EN_POS);
  data[0] |= ((interrupt_to_configure->INV_ICM426XX_FIFO_THS != 0)  << BIT_INT_FIFO_THS_INT_EN_POS);
  data[0] |= ((interrupt_to_configure->INV_ICM426XX_FIFO_FULL != 0) << BIT_INT_FIFO_FULL_INT_EN_POS);

  /* Set INT_SOURCE1 bits */
  data[1] &= (uint8_t)~(BIT_INT_SOURCE1_SMD_INT1_EN 
      | BIT_INT_SOURCE1_WOM_X_INT1_EN 
      | BIT_INT_SOURCE1_WOM_Y_INT1_EN 
      | BIT_INT_SOURCE1_WOM_Z_INT1_EN);
  data[1] |= ((interrupt_to_configure->INV_ICM426XX_SMD   != 0) << BIT_INT_SMD_INT_EN_POS);
  data[1] |= ((interrupt_to_configure->INV_ICM426XX_WOM_X != 0) << BIT_INT_WOM_X_INT_EN_POS);
  data[1] |= ((interrupt_to_configure->INV_ICM426XX_WOM_Y != 0) << BIT_INT_WOM_Y_INT_EN_POS);
  data[1] |= ((interrupt_to_configure->INV_ICM426XX_WOM_Z != 0) << BIT_INT_WOM_Z_INT_EN_POS);

  /* Set INT_SOURCE6 bits */
  data[2] &= (uint8_t)~(BIT_INT_SOURCE6_STEP_DET_INT1_EN
      | BIT_INT_SOURCE6_STEP_CNT_OVFL_INT1_EN
      | BIT_INT_SOURCE6_TILT_DET_INT1_EN
      | BIT_INT_SOURCE6_SLEEP_DET_INT1_EN
      | BIT_INT_SOURCE6_WAKE_DET_INT1_EN
      | BIT_INT_SOURCE6_TAP_DET_INT1_EN);
  data[2] |= ((interrupt_to_configure->INV_ICM426XX_STEP_DET != 0)      << BIT_INT_STEP_DET_INT_EN_POS);
  data[2] |= ((interrupt_to_configure->INV_ICM426XX_STEP_CNT_OVFL != 0) << BIT_INT_STEP_CNT_OVFL_INT_EN_POS);
  data[2] |= ((interrupt_to_configure->INV_ICM426XX_TILT_DET != 0)      << BIT_INT_TILT_DET_INT_EN_POS);
  data[2] |= ((interrupt_to_configure->INV_ICM426XX_SLEEP_DET != 0)     << BIT_INT_SLEEP_DET_INT_EN_POS);
  data[2] |= ((interrupt_to_configure->INV_ICM426XX_WAKE_DET != 0)      << BIT_INT_WAKE_DET_INT_EN_POS);
  data[2] |= ((interrupt_to_configure->INV_ICM426XX_TAP_DET != 0)       << BIT_INT_TAP_DET_INT_EN_POS);
  
  status |= t->serif.write_reg(&(t->serif), MPUREG_INT_SOURCE6_B4, &data[2], 1); /* start with int_source6 since we are still in bank4 */
  status |= inv_icm426xx_set_reg_bank(s, 0);
  status |= t->serif.write_reg(&(t->serif), MPUREG_INT_SOURCE0, data, 2); /* burst write int_source0/int_source1 */

  return status;
}


BB_StatusTypeDef inv_icm426xx_get_config_int1(struct inv_icm426xx * s, inv_icm426xx_interrupt_parameter_t * interrupt_to_configure)
{
  BB_StatusTypeDef status = BB_NO_ERROR;
  struct inv_icm426xx_transport *t = (struct inv_icm426xx_transport *)s;

  uint8_t data[3] = {0};

  status |= t->serif.read_reg(&(t->serif), MPUREG_INT_SOURCE0, data, 2); /* burst read int_source0/int_source1 */
  status |= inv_icm426xx_set_reg_bank(s, 4);
  status |= t->serif.read_reg(&(t->serif), MPUREG_INT_SOURCE6_B4, &data[2], 1); /* int_source6 */
  status |= inv_icm426xx_set_reg_bank(s, 0);

  /* Handles INT_SOURCE0 bits */
  interrupt_to_configure->INV_ICM426XX_UI_FSYNC  = (inv_icm426xx_interrupt_value) ((data[0] & BIT_INT_SOURCE0_UI_FSYNC_INT1_EN)  >> BIT_INT_UI_FSYNC_INT_EN_POS);
  interrupt_to_configure->INV_ICM426XX_UI_DRDY   = (inv_icm426xx_interrupt_value) ((data[0] & BIT_INT_SOURCE0_UI_DRDY_INT1_EN)   >> BIT_INT_UI_DRDY_INT_EN_POS);
  interrupt_to_configure->INV_ICM426XX_FIFO_THS  = (inv_icm426xx_interrupt_value) ((data[0] & BIT_INT_SOURCE0_FIFO_THS_INT1_EN)  >> BIT_INT_FIFO_THS_INT_EN_POS);
  interrupt_to_configure->INV_ICM426XX_FIFO_FULL = (inv_icm426xx_interrupt_value) ((data[0] & BIT_INT_SOURCE0_FIFO_FULL_INT1_EN) >> BIT_INT_FIFO_FULL_INT_EN_POS);

  /* Handles INT_SOURCE1 bits */
  interrupt_to_configure->INV_ICM426XX_SMD   = (inv_icm426xx_interrupt_value) ((data[1] & BIT_INT_SOURCE1_SMD_INT1_EN )   >> BIT_INT_SMD_INT_EN_POS);
  interrupt_to_configure->INV_ICM426XX_WOM_X = (inv_icm426xx_interrupt_value) ((data[1] & BIT_INT_SOURCE1_WOM_X_INT1_EN ) >> BIT_INT_WOM_X_INT_EN_POS);
  interrupt_to_configure->INV_ICM426XX_WOM_Y = (inv_icm426xx_interrupt_value) ((data[1] & BIT_INT_SOURCE1_WOM_Y_INT1_EN ) >> BIT_INT_WOM_Y_INT_EN_POS);
  interrupt_to_configure->INV_ICM426XX_WOM_Z = (inv_icm426xx_interrupt_value) ((data[1] & BIT_INT_SOURCE1_WOM_Z_INT1_EN ) >> BIT_INT_WOM_Z_INT_EN_POS);

  /* Handles INT_SOURCE6 bits */
  interrupt_to_configure->INV_ICM426XX_STEP_DET      = (inv_icm426xx_interrupt_value) ((data[2] & BIT_INT_SOURCE6_STEP_DET_INT1_EN)      >> BIT_INT_STEP_DET_INT_EN_POS);
  interrupt_to_configure->INV_ICM426XX_STEP_CNT_OVFL = (inv_icm426xx_interrupt_value) ((data[2] & BIT_INT_SOURCE6_STEP_CNT_OVFL_INT1_EN) >> BIT_INT_STEP_CNT_OVFL_INT_EN_POS);
  interrupt_to_configure->INV_ICM426XX_TILT_DET      = (inv_icm426xx_interrupt_value) ((data[2] & BIT_INT_SOURCE6_TILT_DET_INT1_EN)      >> BIT_INT_TILT_DET_INT_EN_POS);
  interrupt_to_configure->INV_ICM426XX_SLEEP_DET     = (inv_icm426xx_interrupt_value) ((data[2] & BIT_INT_SOURCE6_SLEEP_DET_INT1_EN)     >> BIT_INT_SLEEP_DET_INT_EN_POS);
  interrupt_to_configure->INV_ICM426XX_WAKE_DET      = (inv_icm426xx_interrupt_value) ((data[2] & BIT_INT_SOURCE6_WAKE_DET_INT1_EN)      >> BIT_INT_WAKE_DET_INT_EN_POS);
  interrupt_to_configure->INV_ICM426XX_TAP_DET       = (inv_icm426xx_interrupt_value) ((data[2] & BIT_INT_SOURCE6_TAP_DET_INT1_EN)       >> BIT_INT_TAP_DET_INT_EN_POS);

  return status;
}

/**
  * @brief: icm42605 serial IF register set
  * @param:struct inv_icm426xx * s             
  * @retval 
 */ 
static BB_StatusTypeDef inv_icm426xx_configure_serial_interface(struct inv_icm426xx * s)
{
  BB_StatusTypeDef status = BB_NO_ERROR;
  uint8_t value;
  
  struct inv_icm426xx_transport *t = (struct inv_icm426xx_transport *)s;
  /* Set memory bank 1 */
  status |= inv_icm426xx_set_reg_bank(s, 1);
  switch(s->transport.serif.serif_type) {
  
    case ICM426XX_UI_I2C:
      /* Enable I2C 50ns spike filtering */
      //int inv_io_hal_read_reg(struct inv_icm426xx_serif * serif, uint8_t reg, uint8_t * rbuffer, uint16_t rlen)

      status |= t->serif.read_reg(&(t->serif), MPUREG_INTF_CONFIG6_B1, &value ,1);
      value &= (uint8_t)~(BIT_INTF_CONFIG6_I3C_SDR_EN_MASK | BIT_INTF_CONFIG6_I3C_DDR_EN_MASK);
      status |= t->serif.write_reg(&(t->serif), MPUREG_INTF_CONFIG6_B1, &value ,1);
      break;
      
    case ICM426XX_UI_I3C:
      /* Enable In Band Interrupt for I3C UI interface and associated payload byte and assign dynamic address */
      status |= t->serif.read_reg(&(t->serif), MPUREG_INTF_CONFIG6_B1, &value ,1);
      value |= (BIT_INTF_CONFIG6_I3C_IBI_BYTE_EN_MASK | BIT_INTF_CONFIG6_I3C_IBI_EN_MASK);
      status |= t->serif.write_reg(&(t->serif), MPUREG_INTF_CONFIG6_B1, &value ,1);
      status |= t->serif.configure((struct inv_icm426xx_serif *)s);
      break;
      
    case ICM426XX_UI_SPI4:
      value = ICM426XX_INTF_CONFIG4_AP_SPI4W;
      status |= t->serif.write_reg(&(t->serif), MPUREG_INTF_CONFIG4_B1, &value ,1);
      break;
      
    default:
      status |= BB_ERROR_BAD_ARG;
  }

  /* Set memory bank 0 */
  
  status |= inv_icm426xx_set_reg_bank(s, 0);
  
  return status;
}




/**
  * @brief:initialize the device using the following hardware settings
  *     • gyroscope fsr = 2000dps
  *     • accelerometer fsr = 2g
  *     • //set timestamp resolution to 16us//seems we donnt need this? zxd
  *     • set interrupt source
  *        - UI_DRDY,FIFO_THS,WOM_X,WOM_Y,WOM_Z for INT1
  *        - mcu donnt support i3c,IBI interrupt donnt use
  *     • enable FIFO mechanism with the following configuration:
  *        – FIFO record mode i.e FIFO count unit is packet
  *        – FIFO snapshot mode i.e drop the data when the FIFO overflows
  *        – Timestamp is logged in FIFO
  *        – Little Endian fifo_count and fifo_data
  *        – generate FIFO threshold interrupt when packet count reaches FIFO watermark
  *        – set FIFO watermark to 1 packet
  *        – enable temperature and timestamp data to go to FIFO 
  * @param:struct inv_icm426xx * s     
  * @retval :BB_StatusTypeDef status
 */
static int inv_icm426xx_init_hardware_from_ui(struct inv_icm426xx * s)
{

  BB_StatusTypeDef status = BB_NO_ERROR;
  struct inv_icm426xx_transport *t = (struct inv_icm426xx_transport *)s;

  uint8_t data;
  uint8_t wom_threshold[3];
  uint8_t gyro_cfg_0_reg, accel_cfg_0_reg, tmst_cfg_reg; 
  
  inv_icm426xx_interrupt_parameter_t config_int = {
    .INV_ICM426XX_UI_FSYNC      = INV_ICM426XX_DISABLE,
    .INV_ICM426XX_UI_DRDY       = INV_ICM426XX_ENABLE,   
    .INV_ICM426XX_FIFO_THS      = INV_ICM426XX_ENABLE,    
    .INV_ICM426XX_FIFO_FULL     = INV_ICM426XX_DISABLE,   
    .INV_ICM426XX_SMD           = INV_ICM426XX_DISABLE,    
    .INV_ICM426XX_WOM_X         = INV_ICM426XX_ENABLE,    
    .INV_ICM426XX_WOM_Y         = INV_ICM426XX_ENABLE,    
    .INV_ICM426XX_WOM_Z         = INV_ICM426XX_ENABLE,    
    .INV_ICM426XX_STEP_DET      = INV_ICM426XX_DISABLE,    
    .INV_ICM426XX_STEP_CNT_OVFL = INV_ICM426XX_DISABLE,    
    .INV_ICM426XX_TILT_DET      = INV_ICM426XX_DISABLE,    
    .INV_ICM426XX_SLEEP_DET     = INV_ICM426XX_DISABLE,  
    .INV_ICM426XX_WAKE_DET      = INV_ICM426XX_DISABLE,  
    .INV_ICM426XX_TAP_DET       = INV_ICM426XX_DISABLE,  
  };

  status |= inv_icm426xx_device_reset(s);


  /* Setup Accel&Gyro properties */
  status |= t->serif.read_reg(&(t->serif), MPUREG_GYRO_CONFIG0, &gyro_cfg_0_reg ,1);
  status |= t->serif.read_reg(&(t->serif), MPUREG_ACCEL_CONFIG0,&accel_cfg_0_reg, 1);
  gyro_cfg_0_reg &= (uint8_t)~BIT_GYRO_CONFIG0_FS_SEL_MASK;
  gyro_cfg_0_reg |= (uint8_t)ICM426XX_GYRO_CONFIG0_FS_SEL_2000dps;
  accel_cfg_0_reg &= (uint8_t)~BIT_ACCEL_CONFIG0_FS_SEL_MASK;
  accel_cfg_0_reg |= (uint8_t)ICM426XX_ACCEL_CONFIG0_FS_SEL_2g;
  status |= t->serif.write_reg(&(t->serif), MPUREG_GYRO_CONFIG0, &gyro_cfg_0_reg, 1);
  status |= t->serif.write_reg(&(t->serif), MPUREG_ACCEL_CONFIG0,&accel_cfg_0_reg, 1);
  
  /* make sure FIFO is disabled */
  data = ICM426XX_FIFO_CONFIG_MODE_BYPASS;
  status |=  t->serif.write_reg(&(t->serif), MPUREG_FIFO_CONFIG, &data, 1);
  
  /* Deactivate FSYNC by default */
  status |= t->serif.read_reg(&(t->serif), MPUREG_FSYNC_CONFIG, &data, 1);
  data &= (uint8_t)~BIT_FSYNC_CONFIG_UI_SEL_MASK;
  status |= t->serif.write_reg(&(t->serif), MPUREG_FSYNC_CONFIG, &data, 1);
  
  status |= t->serif.read_reg(&(t->serif), MPUREG_TMST_CONFIG,&tmst_cfg_reg, 1);
  tmst_cfg_reg &= (uint8_t)~BIT_TMST_CONFIG_TMST_FSYNC_MASK; // == ICM426XX_FSYNC_CONFIG_UI_SEL_NO
  status |= t->serif.write_reg(&(t->serif),MPUREG_TMST_CONFIG,&tmst_cfg_reg, 1);
  
  /* Set default timestamp resolution 16us (Mobile use cases) */
  //comment by zxd
  //status |= inv_icm426xx_configure_timestamp_resolution(s, ICM426XX_TMST_CONFIG_RESOL_16us);
  
  status |= inv_icm426xx_configure_fifo(s, INV_ICM426XX_FIFO_ENABLED);
  
  status |= t->serif.read_reg(&(t->serif),  MPUREG_INT_CONFIG,&data,1);
  /* Enable push pull on INT1 to avoid moving in Test Mode after a soft reset */
  data |= (uint8_t)ICM426XX_INT_CONFIG_INT1_DRIVE_CIRCUIT_PP;
  /* Configure the INT1 interrupt pulse as active high */
  data |= (uint8_t)ICM426XX_INT_CONFIG_INT1_POLARITY_HIGH;
  status |= t->serif.write_reg(&(t->serif),MPUREG_INT_CONFIG, &data, 1);
  
  /* Config interrupt source
  the interrupt sources is defined in inv_icm426xx_interrupt_parameter_t config_int */
  status |= inv_icm426xx_set_config_int1(s,&config_int);
  //config_int.INV_ICM426XX_UI_DRDY  = INV_ICM426XX_ENABLE;
  //config_int.INV_ICM426XX_FIFO_THS = INV_ICM426XX_DISABLE;
  //status |= inv_icm426xx_set_config_ibi(s,&config_int);

  /* 
  See datasheet P81, INT_CONFIG1
  * "User should change setting to 0 from default setting of 1, for proper INT1 and INT2 pin operation"
  * Set the ASY_RESET_DISABLE bit to 0 (async enabled) in order to chop Tpulse as soon as interrupt status is read
  * Guideline is to set the ASY_RESET_DISABLE bit to 0 in pulse mode
  * No effect in latch mode 
  */
  status |= t->serif.read_reg(&(t->serif), MPUREG_INT_CONFIG1, &data, 1);
  data &= (uint8_t)~BIT_INT_CONFIG1_ASY_RST_MASK;
  data |= (uint8_t)ICM426XX_INT_CONFIG1_ASY_RST_ENABLED;
  status |= t->serif.write_reg(&(t->serif), MPUREG_INT_CONFIG1, &data, 1);

  /* Set the UI filter order to 2 for both gyro and accel */
  status |= t->serif.read_reg(&(t->serif), MPUREG_GYRO_CONFIG1, &data, 1);
  data &= (uint8_t)~BIT_GYRO_CONFIG1_GYRO_UI_FILT_ORD_MASK;
  data |= (uint8_t)ICM426XX_GYRO_CONFIG_GYRO_UI_FILT_ORD_2ND_ORDER;
  status |= t->serif.write_reg(&(t->serif), MPUREG_GYRO_CONFIG1, &data, 1);
  status |= t->serif.read_reg(&(t->serif), MPUREG_ACCEL_CONFIG1, &data, 1);
  data &= (uint8_t)~BIT_ACCEL_CONFIG1_ACCEL_UI_FILT_ORD_MASK;
  data |= (uint8_t)ICM426XX_ACCEL_CONFIG_ACCEL_UI_FILT_ORD_2ND_ORDER;
  status |= t->serif.write_reg(&(t->serif), MPUREG_ACCEL_CONFIG1, &data, 1);
  
  /* 
  FIFO packets are 16bit format by default
  icm42605 is not support high resolution fifo mode
  */
  //status |= inv_icm426xx_disable_high_resolution_fifo(s);
  
  /* 
    configure SMD_CONFIG
    WOM mode :compare current sample with the previous sample 
    SMD_MODE :keep default, disable
    WOM_INT_MODE: when all axis exceed 52 mg 
  */
  status |= inv_icm426xx_set_reg_bank(s, 4); /* Set memory bank 4 */
  wom_threshold[0] = ICM426XX_DEFAULT_WOM_THS_MG; /* Set X threshold */
  wom_threshold[1] = ICM426XX_DEFAULT_WOM_THS_MG; /* Set Y threshold */
  wom_threshold[2] = ICM426XX_DEFAULT_WOM_THS_MG; /* Set Z threshold */
  status |= t->serif.write_reg(&(t->serif), MPUREG_ACCEL_WOM_X_THR_B4, &wom_threshold[0], sizeof(wom_threshold));
  
  status |= inv_icm426xx_set_reg_bank(s, 0); /* Set memory bank 0 */
  data = ((uint8_t)ICM426XX_SMD_CONFIG_WOM_INT_MODE_ANDED) | ((uint8_t)ICM426XX_SMD_CONFIG_WOM_MODE_CMP_PREV);
  status |= t->serif.write_reg(&(t->serif), MPUREG_SMD_CONFIG, &data, 1);

  /* by default, set IIR filter BW to ODR/4 for LN, 16x averaging for GLP, 16x averaging for ALP */
  s->avg_bw_setting.acc_ln_bw = (uint8_t)ICM426XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_4;
  s->avg_bw_setting.gyr_ln_bw = (uint8_t)ICM426XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_4;
  s->avg_bw_setting.acc_lp_avg = (uint8_t)ICM426XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_AVG_16;

  /* Reset self-test result variable*/
  s->st_result = 0;

  return status;
}



/**
  * @}
  */



/*****END OF FILE****/


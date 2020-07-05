/*****************************************************************************
 *
 * @file:	D:\prj\git-version\BB_sensor_IMU\IMU\drivers\icm426xx\icm42605SelfTest.c
 * @author: Xiaodong Zhang
 * @Email:	sacntumz@foxmail.com;zhangxiaodong@beyondbit.com
 * @date:	2020/06
 * @brief:  File including icm42605 

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
#include <math.h>
#include "ICM42605.h"
#include "icm42605SelfTest.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

	#define ST_GYRO_FSR             ICM426XX_GYRO_CONFIG0_FS_SEL_250dps
	#define ST_GYRO_ODR             ICM426XX_ACCEL_CONFIG0_ODR_200_HZ
	#define ST_GYRO_UI_FILT_ORD_IND ICM426XX_GYRO_CONFIG_GYRO_UI_FILT_ORD_3RD_ORDER
	#define ST_GYRO_UI_FILT_BW_IND  ICM426XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_10
	
	#define ST_ACCEL_FSR             ICM426XX_ACCEL_CONFIG0_FS_SEL_4g
	#define ST_ACCEL_ODR             ICM426XX_ACCEL_CONFIG0_ODR_200_HZ
	#define ST_ACCEL_UI_FILT_ORD_IND ICM426XX_ACCEL_CONFIG_ACCEL_UI_FILT_ORD_3RD_ORDER
	#define ST_ACCEL_UI_FILT_BW_IND  ICM426XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_10
	
	/* Pass/Fail criteria */
	#define MIN_RATIO_GYRO         0.5f /* expected ratio greater than 0.5 */ 
	#define MIN_ST_GYRO_DPS        60   /* expected values greater than 60dps */
	#define MAX_ST_GYRO_OFFSET_DPS 20   /* expected offset less than 20 dps */
	
	#define MIN_RATIO_GYRO  0.5f /* expected ratio greater than 0.5 */ 
	#define MAX_RATIO_GYRO  1.5f /* expected ratio lower than 1.5 */ 
	#define MIN_ST_ACCEL_MG 225  /* expected values in [225mgee;675mgee] */
	#define MAX_ST_ACCEL_MG 675

	#define INV_ST_OTP_EQUATION(FS, ST_code) (uint32_t)((2620/pow(2,3-FS))*pow(1.01, ST_code-1)+0.5)

	enum inv_icm426xx_sensor_on_mask {
		INV_ICM426XX_SENSOR_ON_MASK_ACCEL = (1L<<INV_ICM426XX_SENSOR_ACCEL),
		INV_ICM426XX_SENSOR_ON_MASK_GYRO  = (1L<<INV_ICM426XX_SENSOR_GYRO),
	};

	struct recover_regs {
		/* bank 0 */
		uint8_t intf_config1;       /* REG_INTF_CONFIG1       */
		uint8_t pwr_mgmt_0;         /* REG_PWR_MGMT_0         */
		uint8_t accel_config0;      /* REG_ACCEL_CONFIG0      */
		uint8_t accel_config1;      /* REG_ACCEL_CONFIG1      */
		uint8_t gyro_config0;       /* REG_GYRO_CONFIG0       */
		uint8_t gyro_config1;       /* REG_GYRO_CONFIG1       */
		uint8_t accel_gyro_config0; /* REG_ACCEL_GYRO_CONFIG0 */
		uint8_t fifo_config1;       /* REG_FIFO_CONFIG1       */
		uint8_t self_test_config;   /* REG_SELF_TEST_CONFIG   */
	};
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
  
/* Private functions ---------------------------------------------------------*/
 

static BB_StatusTypeDef save_settings(struct inv_icm426xx * s, struct recover_regs * saved_regs);
static BB_StatusTypeDef recover_settings(struct inv_icm426xx * s, struct recover_regs * saved_regs);
static BB_StatusTypeDef average_sensor_output(struct inv_icm426xx * s, int sensor, uint8_t self_test_config, int32_t average[3]);
static BB_StatusTypeDef set_user_offset_regs(struct inv_icm426xx * s, uint8_t sensor);
static BB_StatusTypeDef run_accel_self_test(struct inv_icm426xx * s, uint8_t * result);
static BB_StatusTypeDef run_gyro_self_test(struct inv_icm426xx * s, uint8_t * result);
static int reg_to_gyro_fsr(ICM426XX_GYRO_CONFIG0_FS_SEL_t reg);
static int reg_to_accel_fsr(ICM426XX_ACCEL_CONFIG0_FS_SEL_t reg);

/** @addtogroup 
  * @{
  */

/** @addtogroup 
  * @{
  */


/**
  * @brief  
  * @param              
  * @retval 
 */ 


//Self-Test codes begin
/**
  * @brief: run icm42605 gyro self-test
  * @param:struct inv_icm426xx * s     
  *         result : GYR success (1 << 0) and ACC success (1 << 1)       
  * @retval :BB_StatusTypeDef status
 */

BB_StatusTypeDef inv_icm426xx_run_selftest(struct inv_icm426xx * s, uint8_t * result)
{
  BB_StatusTypeDef status = BB_NO_ERROR;
	uint8_t gyro_result = 0, accel_result = 0;
	struct recover_regs saved_regs;
 
  *result = 0;

	/* Run self-test only once */
	if (s->st_result == 0) {

		/* Save current settings to restore them at the end of the routine */
		status |= save_settings(s, &saved_regs);

		status |= run_gyro_self_test(s, &gyro_result);
		BB_MSG(BB_MSG_LEVEL_DEBUG,"Gyro ST result:%d",gyro_result);
		if ((status == 0) && (gyro_result == 1))
			status |= set_user_offset_regs(s, INV_ICM426XX_SENSOR_ON_MASK_GYRO);
		
		status |= run_accel_self_test(s, &accel_result);
		BB_MSG(BB_MSG_LEVEL_DEBUG,"Accel ST result:%d",accel_result);
		if ((status == 0) && (accel_result == 1))
			status |= set_user_offset_regs(s, INV_ICM426XX_SENSOR_ON_MASK_ACCEL);

		/* Restore settings previously saved */
		status |= recover_settings(s, &saved_regs);
		
		/* Store acc and gyr results */
		s->st_result = (accel_result << 1) | gyro_result;
	}

	*result = s->st_result;
	return status;
}


uint8_t inv_icm426xx_get_st_bias(struct inv_icm426xx * s, int st_bias[6])
{
	uint8_t status = 0;
	uint8_t i;
	
	/* if ST didn't run, return null biases */
	if (s->st_result == 0) {
		for (i = 0; i < 6; i++)
			st_bias[i] = 0;
		return (uint8_t)1;
	}

	/* Gyro bias LN: first 3 elements */
	for (i = 0; i < 3; i++) /* convert bias to 1 dps Q16 */
		st_bias[i] = s->gyro_st_bias[i] * 2 * reg_to_gyro_fsr(ST_GYRO_FSR);

	/* Accel bias LN: last 3 elements */
	for (i = 0; i < 3; i++) /* convert bias to 1 gee Q16 */
		st_bias[i+3] = s->accel_st_bias[i] * 2 * reg_to_accel_fsr(ST_ACCEL_FSR);

	return status;
}

/*
 * Params:
 *   - result: 1 if success, 0 if failure
 * Returns 0 if success, error code if failure
 */
static BB_StatusTypeDef run_gyro_self_test(struct inv_icm426xx * s, uint8_t * result)
{

  struct inv_icm426xx_transport *t = (struct inv_icm426xx_transport *)s;
	BB_StatusTypeDef status = BB_NO_ERROR;
	uint8_t data;
	int32_t STG_OFF[3], STG_ON[3];
	uint32_t STG_response[3];

	uint8_t ST_code_regs[3];
	uint32_t STG_OTP[3];

	int i = 0;
	
	uint32_t gyro_sensitivity_1dps;
	
	*result = 1;

	/* Set gyro configuration */
	status |= t->serif.read_reg(&(t->serif), MPUREG_GYRO_CONFIG0, &data, 1);
	data &= ~BIT_GYRO_CONFIG0_FS_SEL_MASK;
	data &= ~BIT_GYRO_CONFIG0_ODR_MASK;
	data |= ST_GYRO_FSR;
	data |= ST_GYRO_ODR;
	status |= t->serif.write_reg(&(t->serif), MPUREG_GYRO_CONFIG0, &data, 1);

	status |= t->serif.read_reg(&(t->serif), MPUREG_GYRO_CONFIG1, &data, 1);
	data &= ~BIT_GYRO_CONFIG1_GYRO_UI_FILT_ORD_MASK;
	data |= ST_GYRO_UI_FILT_ORD_IND;
	status |= t->serif.write_reg(&(t->serif), MPUREG_GYRO_CONFIG1, &data, 1);
	
	status |= t->serif.read_reg(&(t->serif), MPUREG_ACCEL_GYRO_CONFIG0, &data, 1);
	data &= ~BIT_GYRO_ACCEL_CONFIG0_GYRO_FILT_MASK; 
	data |= ST_GYRO_UI_FILT_BW_IND;
	status |= t->serif.write_reg(&(t->serif), MPUREG_ACCEL_GYRO_CONFIG0, &data, 1);
	/* Read average gyro digital output for each axis and store them as STG_OFF_{x,y,z} in lsb */
	status |= average_sensor_output(s, INV_ICM426XX_SENSOR_ON_MASK_GYRO, 0, STG_OFF);
	BB_MSG(BB_MSG_LEVEL_DEBUG,"Gyro STG_OFF[0] = %d",STG_OFF[0]);
	BB_MSG(BB_MSG_LEVEL_DEBUG,"Gyro STG_OFF[1] = %d",STG_OFF[1]);
	BB_MSG(BB_MSG_LEVEL_DEBUG,"Gyro STG_OFF[2] = %d",STG_OFF[2]);
		/* Enable self-test for each axis and read average gyro digital output 
	 * for each axis and store them as STG_ON_{x,y,z} in lsb */
	status |= average_sensor_output(s, INV_ICM426XX_SENSOR_ON_MASK_GYRO, 
		(BIT_GYRO_X_ST_EN | BIT_GYRO_Y_ST_EN | BIT_GYRO_Z_ST_EN), STG_ON);
	BB_MSG(BB_MSG_LEVEL_DEBUG,"Gyro STG_ON[0] = %d",STG_ON[0]);
	BB_MSG(BB_MSG_LEVEL_DEBUG,"Gyro STG_ON[1] = %d",STG_ON[1]);
	BB_MSG(BB_MSG_LEVEL_DEBUG,"Gyro STG_ON[2] = %d",STG_ON[2]);

		/* calculate the self-test response as ABS(ST_ON_{x,y,z} - ST_OFF_{x,y,z}) for each axis */
	for(i = 0; i < 3; i++)
		STG_response[i] = INV_ABS(STG_ON[i] - STG_OFF[i]);
	BB_MSG(BB_MSG_LEVEL_DEBUG,"Gyro STG_response[0] = %d",STG_response[0]);
	BB_MSG(BB_MSG_LEVEL_DEBUG,"Gyro STG_response[1] = %d",STG_response[1]);
	BB_MSG(BB_MSG_LEVEL_DEBUG,"Gyro STG_response[2] = %d",STG_response[2]);

	/* calculate ST results OTP based on the equation */
	status |= inv_icm426xx_set_reg_bank(s,1);
	status |= t->serif.read_reg(&(t->serif), MPUREG_XG_ST_DATA_B1, ST_code_regs, 3);
	status |= inv_icm426xx_set_reg_bank(s,0);

	for (i = 0; i < 3; i++) {
		int fs_sel = ST_GYRO_FSR >> BIT_GYRO_CONFIG0_FS_SEL_POS;
		STG_OTP[i] = INV_ST_OTP_EQUATION(fs_sel, ST_code_regs[i]);
		BB_MSG(BB_MSG_LEVEL_DEBUG,"Gyro OTP[%d]= %d",i,STG_OTP[i]);
	}
	
	/** Check Gyro self-test results */
	gyro_sensitivity_1dps = 32768 / reg_to_gyro_fsr(ST_GYRO_FSR);
	BB_MSG(BB_MSG_LEVEL_DEBUG,"gyro_sensitivity_1dps = %d",gyro_sensitivity_1dps);

	for (i = 0; i < 3; i++) 
	{
		if (STG_OTP[i]) 
		{
			float ratio = ((float)STG_response[i]) / ((float)STG_OTP[i]);
			BB_MSG(BB_MSG_LEVEL_DEBUG,"Gyro Ratio[%1d] = %f",i, ratio);
			if (ratio <= MIN_RATIO_GYRO)
			{
			BB_MSG(BB_MSG_LEVEL_ERROR,"Ratio Compare Fail");					
			*result = 0; /* fail */
			}
		} 
		else if(STG_response[i] < (MIN_ST_GYRO_DPS * gyro_sensitivity_1dps)) 
		{
			*result = 0; /* fail */
		}
	}

	/* stored the computed bias (checking GST and GOFFSET values) */
	for (i = 0; i < 3; i++) {
		if((INV_ABS(STG_OFF[i]) > (int32_t)(MAX_ST_GYRO_OFFSET_DPS * gyro_sensitivity_1dps)))
			*result = 0; /* fail */
		s->gyro_st_bias[i] = STG_OFF[i];
	}

	return status;
}

/* @brief:
 * @param:
 *   - result: 1 if success, 0 if failure
 * Returns 0 if success, error code if failure
 */
static BB_StatusTypeDef run_accel_self_test(struct inv_icm426xx * s, uint8_t * result)
{
	struct inv_icm426xx_transport *t = (struct inv_icm426xx_transport *)s;
	BB_StatusTypeDef status = BB_NO_ERROR;
	uint8_t data;

	int32_t STA_OFF[3], STA_ON[3];
	uint32_t STA_response[3];

	uint8_t ST_code_regs[3];
	uint32_t STA_OTP[3];

	int i = 0;
	int axis, axis_sign;
	
	uint32_t accel_sensitivity_1g, gravity; 

	*result = 1;
	
	/* Set accel configuration */
	status |= t->serif.read_reg(&(t->serif), MPUREG_ACCEL_CONFIG0, &data,1);
	data &= ~BIT_ACCEL_CONFIG0_FS_SEL_MASK;
	data &= ~BIT_ACCEL_CONFIG0_ODR_MASK;
	data |= ST_ACCEL_FSR;
	data |= ST_ACCEL_ODR;
	status |= t->serif.write_reg(&(t->serif), MPUREG_ACCEL_CONFIG0,&data,1);
	
	status |= t->serif.read_reg(&(t->serif), MPUREG_ACCEL_CONFIG1, &data,1);
	data &= ~BIT_ACCEL_CONFIG1_ACCEL_UI_FILT_ORD_MASK;
	data |= ST_ACCEL_UI_FILT_ORD_IND;
	status |= t->serif.write_reg(&(t->serif), MPUREG_ACCEL_CONFIG1,&data,1);
	
	status |= t->serif.read_reg(&(t->serif), MPUREG_ACCEL_GYRO_CONFIG0, &data,1);
	data &= ~BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_MASK;
	data |= ST_ACCEL_UI_FILT_BW_IND;
	status |= t->serif.write_reg(&(t->serif), MPUREG_ACCEL_GYRO_CONFIG0, &data,1);

	/* read average accel digital output for each axis and store them as ST_OFF_{x,y,z} in lsb x 1000 */
	status |= average_sensor_output(s, INV_ICM426XX_SENSOR_ON_MASK_ACCEL, 0, STA_OFF);
	BB_MSG(BB_MSG_LEVEL_DEBUG,"Accel STG_OFF[0] = %d",STA_OFF[0]);
	BB_MSG(BB_MSG_LEVEL_DEBUG,"Accel STG_OFF[1] = %d",STA_OFF[1]);
	BB_MSG(BB_MSG_LEVEL_DEBUG,"Accel STG_OFF[2] = %d",STA_OFF[2]);
	/* Enable self-test for each axis and read average gyro digital output 
	 * for each axis and store them as ST_ON_{x,y,z} in lsb x 1000 */
	status |= average_sensor_output(s, INV_ICM426XX_SENSOR_ON_MASK_ACCEL, 
		(BIT_ACCEL_X_ST_EN | BIT_ACCEL_Y_ST_EN |BIT_ACCEL_Z_ST_EN | BIT_ST_REGULATOR_EN), STA_ON);
	BB_MSG(BB_MSG_LEVEL_DEBUG,"Accel STG_ON[0] = %d",STA_ON[0]);
	BB_MSG(BB_MSG_LEVEL_DEBUG,"Accel STG_ON[1] = %d",STA_ON[1]);
	BB_MSG(BB_MSG_LEVEL_DEBUG,"Accel STG_ON[2] = %d",STA_ON[2]);
	/* calculate the self-test response as ABS(ST_ON_{x,y,z} - ST_OFF_{x,y,z}) for each axis */
	/* outputs from this routine are in units of lsb and hence are dependent on the full-scale used on the DUT */
	for(i = 0; i < 3; i++)
		STA_response[i] = INV_ABS(STA_ON[i] - STA_OFF[i]);
	BB_MSG(BB_MSG_LEVEL_DEBUG,"Accel STG_response[0] = %d",STA_response[0]);
	BB_MSG(BB_MSG_LEVEL_DEBUG,"Accel STG_response[1] = %d",STA_response[1]);
	BB_MSG(BB_MSG_LEVEL_DEBUG,"Accel STG_response[2] = %d",STA_response[2]);
	/* calculate ST results OTP based on the equation */

	status |= inv_icm426xx_set_reg_bank(s,2);
	status |= t->serif.read_reg(&(t->serif), MPUREG_XA_ST_DATA_B2, ST_code_regs, 3);
	status |= inv_icm426xx_set_reg_bank(s,0);

	for (i = 0; i < 3; i++) {
		int fs_sel = ST_ACCEL_FSR >> BIT_ACCEL_CONFIG0_FS_SEL_POS;
		STA_OTP[i] = INV_ST_OTP_EQUATION(fs_sel, ST_code_regs[i]);
	}
	
	/** Check Accel self-test result */
	accel_sensitivity_1g = 32768 / reg_to_accel_fsr(ST_ACCEL_FSR);
	BB_MSG(BB_MSG_LEVEL_DEBUG,"Accel_sensitivity_1g = %d",accel_sensitivity_1g);

	for (i = 0; i < 3; i++) {
		if (STA_OTP[i]) {
			float ratio = ((float)STA_response[i]) / ((float)STA_OTP[i]);
			BB_MSG(BB_MSG_LEVEL_DEBUG,"Accel Ratio[%1d] = %f",i, ratio);
			if ((ratio >= MAX_RATIO_GYRO) || (ratio <= MIN_RATIO_GYRO))
				*result = 0; /* fail */
		} else if ((STA_response[i] < ((MIN_ST_ACCEL_MG * accel_sensitivity_1g) / 1000))
				|| (STA_response[i] > ((MAX_ST_ACCEL_MG * accel_sensitivity_1g) / 1000))) {
			*result = 0; /* fail */
		}
	}

	/* stored the computed offset */
	for(i = 0; i < 3; i++) {
		s->accel_st_bias[i] = STA_OFF[i];
	}

	/* assume the largest data axis shows +1 or -1 gee for gravity */
	axis = 0;
	axis_sign = 1;
	if (INV_ABS(s->accel_st_bias[1]) > INV_ABS(s->accel_st_bias[0]))
		axis = 1;
	if (INV_ABS(s->accel_st_bias[2]) > INV_ABS(s->accel_st_bias[axis]))
		axis = 2;
	if (s->accel_st_bias[axis] < 0)
		axis_sign = -1;

	gravity = accel_sensitivity_1g * axis_sign;
	s->accel_st_bias[axis] -= gravity;

	return status;
}

static BB_StatusTypeDef set_user_offset_regs(struct inv_icm426xx * s, uint8_t sensor)
{
	struct inv_icm426xx_transport *t = (struct inv_icm426xx_transport *)s;
	BB_StatusTypeDef status = BB_NO_ERROR;

	uint8_t data[5];
	int16_t cur_bias;

	// Set memory bank 4
	status |= inv_icm426xx_set_reg_bank(s, 4);

	/* Set offset registers sensor */ 
	if (sensor == INV_ICM426XX_SENSOR_ON_MASK_ACCEL) {
		/* 
		 * Invert sign for OFFSET and
		 * accel_st_bias is 2g coded 16 
		 * OFFUSER is 1g coded 12 (or 2g coded 12 for High FSR parts)
		 */
		status |= t->serif.read_reg(&(t->serif),MPUREG_OFFSET_USER_4_B4, &data[0], 1); // Fetch gyro_z_offuser[8-11]
		data[0] &= BIT_GYRO_Z_OFFUSER_MASK_HI;
		
		cur_bias = (int16_t)(-s->accel_st_bias[0] >> 3);
		cur_bias /= ACCEL_OFFUSER_MAX_MG/1000;
		data[0] |= (((cur_bias & 0x0F00) >> 8) << BIT_ACCEL_X_OFFUSER_POS_HI);
		data[1] = ((cur_bias & 0x00FF) << BIT_ACCEL_X_OFFUSER_POS_LO);
		cur_bias = (int16_t)(-s->accel_st_bias[1] >> 3);
		cur_bias /= ACCEL_OFFUSER_MAX_MG/1000;
		data[2] = ((cur_bias & 0x00FF) << BIT_ACCEL_Y_OFFUSER_POS_LO);
		data[3] = (((cur_bias & 0x0F00) >> 8) << BIT_ACCEL_Y_OFFUSER_POS_HI);
		cur_bias = (int16_t)(-s->accel_st_bias[2] >> 3);
		cur_bias /= ACCEL_OFFUSER_MAX_MG/1000;
		data[3] |= (((cur_bias & 0x0F00) >> 8) << BIT_ACCEL_Z_OFFUSER_POS_HI);
		data[4] = ((cur_bias & 0x00FF) << BIT_ACCEL_Z_OFFUSER_POS_LO);
		
		status |= t->serif.write_reg(&(t->serif), MPUREG_OFFSET_USER_4_B4, &data[0], 5);
		
	} else if (sensor == INV_ICM426XX_SENSOR_ON_MASK_GYRO) {
		/* 
		 * Invert sign for OFFSET and
		 * gyro_st_bias is 250dps coded 16 
		 * OFFUSER is 64dps coded 12 (or 128dps coded 12 for High FSR parts)
		 */
		status |= t->serif.read_reg(&(t->serif), MPUREG_OFFSET_USER_4_B4, &data[4], 1); // Fetch accel_x_offuser[8-11]
		data[4] &= BIT_ACCEL_X_OFFUSER_MASK_HI;
		
		cur_bias = (int16_t)(-(s->gyro_st_bias[0]*250/GYRO_OFFUSER_MAX_DPS) >> 4);
		data[0] = ((cur_bias & 0x00FF) << BIT_GYRO_X_OFFUSER_POS_LO);
		data[1] = (((cur_bias & 0x0F00) >> 8) << BIT_GYRO_X_OFFUSER_POS_HI);
		cur_bias = (int16_t)(-(s->gyro_st_bias[1]*250/GYRO_OFFUSER_MAX_DPS) >> 4);
		data[1] |= (((cur_bias & 0x0F00) >> 8) << BIT_GYRO_Y_OFFUSER_POS_HI);
		data[2] = ((cur_bias & 0x00FF) << BIT_GYRO_Y_OFFUSER_POS_LO);
		cur_bias = (int16_t)(-(s->gyro_st_bias[2]*250/GYRO_OFFUSER_MAX_DPS) >> 4);
		data[3] = ((cur_bias & 0x00FF) << BIT_GYRO_Z_OFFUSER_POS_LO);
		data[4] |= (((cur_bias & 0x0F00) >> 8) << BIT_GYRO_Z_OFFUSER_POS_HI);
		
		status |= t->serif.write_reg(&(t->serif), MPUREG_OFFSET_USER_0_B4, &data[0], 5);
		
	}

	// Set memory bank 0
	status |= inv_icm426xx_set_reg_bank(s, 0);

	return status;
}
/**
  * @brief: Get the sensor average output for gyro and accel
  * @param:struct inv_icm426xx * s     
  *        int sensor : INV_ICM426XX_SENSOR_ON_MASK_ACCEL for accel
                      : INV_ICM426XX_SENSOR_ON_MASK_GYRO for gyro
                      : else NOTSUPPORT
           uint8_t self_test_config : 1 for self test on
  * @retval :BB_StatusTypeDef status
 */

static BB_StatusTypeDef average_sensor_output(struct inv_icm426xx * s, 
												int sensor, 
												uint8_t self_test_config, 
												int32_t average[3])
{
	struct inv_icm426xx_transport *t = (struct inv_icm426xx_transport *)s;
	BB_StatusTypeDef status = BB_NO_ERROR;
  
	int it = 0; /* Number of sample read */
	int sample_discarded = 0; /* Number of sample discarded */ 
	int timeout = 300; /* us */
	uint8_t data_reg; /* address of the register where to read the data */
	uint8_t self_test_config_reg; /* SELF_TEST_CONFIG register content */
	uint8_t pwr_mgmt_reg; /* PWR_MGMT register content */
	int32_t sum[3] = {0}; /* sum of all data read */

	if(sensor == INV_ICM426XX_SENSOR_ON_MASK_GYRO) 
	{
		data_reg = MPUREG_GYRO_DATA_X0_UI;

		/* Enable Gyro */
		status |= t->serif.read_reg(&(t->serif), MPUREG_PWR_MGMT_0, &pwr_mgmt_reg, 1);
		pwr_mgmt_reg &= (uint8_t)~BIT_PWR_MGMT_0_GYRO_MODE_MASK;
		pwr_mgmt_reg |= (uint8_t)ICM426XX_PWR_MGMT_0_GYRO_MODE_LN;
		status |= t->serif.write_reg(&(t->serif), MPUREG_PWR_MGMT_0, &pwr_mgmt_reg, 1);

		/* wait for 60ms to allow output to settle */
		HAL_Delay(60)	;
	} 
	else if(sensor == INV_ICM426XX_SENSOR_ON_MASK_ACCEL) 
	{
		data_reg = MPUREG_ACCEL_DATA_X0_UI;
		
		/* Enable Accel */
		status |= t->serif.read_reg(&(t->serif), MPUREG_PWR_MGMT_0, &pwr_mgmt_reg, 1);
		pwr_mgmt_reg &= (uint8_t)~BIT_PWR_MGMT_0_ACCEL_MODE_MASK;
		pwr_mgmt_reg |= (uint8_t)ICM426XX_PWR_MGMT_0_ACCEL_MODE_LN;
		status |= t->serif.write_reg(&(t->serif), MPUREG_PWR_MGMT_0, &pwr_mgmt_reg, 1);

		/* wait for 25ms to allow output to settle */
		HAL_Delay(25);
	}
	else
		return BB_ERROR_BAD_ARG; /* Invalid sensor provided */

	/* Apply ST config if required */
	if(self_test_config != 0) {
		status |= t->serif.read_reg(&(t->serif), MPUREG_SELF_TEST_CONFIG, &self_test_config_reg, 1);
		self_test_config_reg |= self_test_config; 
		status |= t->serif.write_reg(&(t->serif), MPUREG_SELF_TEST_CONFIG, &self_test_config_reg, 1);

		if(sensor == INV_ICM426XX_SENSOR_ON_MASK_GYRO)
			/* wait 200ms for the oscillation to stabilize */
			HAL_Delay(200);
		else 
			/* wait for 25ms to allow output to settle */
			HAL_Delay(25);
	}

	do {
		uint8_t int_status;
		status |= t->serif.read_reg(&(t->serif), MPUREG_INT_STATUS, &int_status, 1);
		
		if (int_status & BIT_INT_STATUS_DRDY) 
		{
			int16_t sensor_data[3] = {0}; 
			uint8_t sensor_data_reg[6]; /* sensor data registers content */

			/* Read data */
			status |= t->serif.read_reg(&(t->serif), data_reg, sensor_data_reg, 6);
			
			if (s->endianess_data == ICM426XX_INTF_CONFIG0_DATA_BIG_ENDIAN) 
			{
				sensor_data[0] = (sensor_data_reg[0] << 8) | sensor_data_reg[1];
				sensor_data[1] = (sensor_data_reg[2] << 8) | sensor_data_reg[3];
				sensor_data[2] = (sensor_data_reg[4] << 8) | sensor_data_reg[5];
			} 
			else 
			{ // LITTLE ENDIAN
				sensor_data[0] = (sensor_data_reg[1] << 8) | sensor_data_reg[0];
				sensor_data[1] = (sensor_data_reg[3] << 8) | sensor_data_reg[2];
				sensor_data[2] = (sensor_data_reg[5] << 8) | sensor_data_reg[4];
			}

			if ((sensor_data[0] != -32768) && (sensor_data[1] != -32768) && (sensor_data[2] != -32768)) 
			{
				sum[0] += sensor_data[0];
				sum[1] += sensor_data[1];
				sum[2] += sensor_data[2];
			} 
			else 
			{
				sample_discarded++;
			}
			it++;
		}
		HAL_Delay(1);
		//timeout--;
	} while((it < 200) && (timeout > 0));

	/* Disable Accel and Gyro */
	status |= t->serif.read_reg(&(t->serif), MPUREG_PWR_MGMT_0, &pwr_mgmt_reg, 1);
	pwr_mgmt_reg &= (uint8_t)~BIT_PWR_MGMT_0_GYRO_MODE_MASK;
	pwr_mgmt_reg &= (uint8_t)~BIT_PWR_MGMT_0_ACCEL_MODE_MASK;
	pwr_mgmt_reg |= (uint8_t)ICM426XX_PWR_MGMT_0_GYRO_MODE_OFF;
	pwr_mgmt_reg |= (uint8_t)ICM426XX_PWR_MGMT_0_ACCEL_MODE_OFF;
	status |= t->serif.write_reg(&(t->serif), MPUREG_PWR_MGMT_0, &pwr_mgmt_reg, 1);
	s->gyro_power_off_tmst = HAL_GetTick();
	/* Disable self-test config if necessary */
	if(self_test_config) {
		self_test_config_reg &= ~self_test_config;
		status |= t->serif.write_reg(&(t->serif), MPUREG_SELF_TEST_CONFIG, &self_test_config_reg, 1);
	}

	/* Compute average value */
	it -= sample_discarded;

	average[0] = (sum[0] / it);
	average[1] = (sum[1] / it);
	average[2] = (sum[2] / it);

	BB_MSG(BB_MSG_LEVEL_DEBUG,"Sample Discard:%d",sample_discarded);
	BB_MSG(BB_MSG_LEVEL_DEBUG,"IT: %d",it);
	
	// BB_MSG(BB_MSG_LEVEL_DEBUG,"SensorDataSum X:%d",sum[0]);
	// BB_MSG(BB_MSG_LEVEL_DEBUG,"SensorDataSum Y:%d",sum[1]);
	// BB_MSG(BB_MSG_LEVEL_DEBUG,"SensorDataSum Z:%d",sum[2]);
	// BB_MSG(BB_MSG_LEVEL_DEBUG,"SensorDataAverage X:%d",average[0]);
	// BB_MSG(BB_MSG_LEVEL_DEBUG,"SensorDataAverage Y:%d",average[1]);
	// BB_MSG(BB_MSG_LEVEL_DEBUG,"SensorDataAverage Z:%d",average[2]);

	return status;
}

/**
  * @brief: Save settings for follow register
  *      - INTF_CONFIG1
  *      - PWR_MGMT_0
  *      - ACCEL_CONFIG0
  *      - ACCEL_CONFIG1
  *      - GYRO_CONFIG0
  *      - GYRO_CONFIG1
  *      - ACCEL_GYRO_CONFIG
  *      - FIFO_CONFIG1
  *      - SELF_TEST_CONFIG
  * @param:struct inv_icm426xx * s             
  * @retval 
 */ 
static BB_StatusTypeDef save_settings(struct inv_icm426xx * s, struct recover_regs * saved_regs)
{
  struct inv_icm426xx_transport *t = (struct inv_icm426xx_transport *)s;
  BB_StatusTypeDef status = BB_NO_ERROR;

  status |= t->serif.read_reg(&(t->serif), MPUREG_INTF_CONFIG1, &saved_regs->intf_config1, 1);
  status |= t->serif.read_reg(&(t->serif), MPUREG_PWR_MGMT_0, &saved_regs->pwr_mgmt_0, 1);
  status |= t->serif.read_reg(&(t->serif), MPUREG_ACCEL_CONFIG0, &saved_regs->accel_config0, 1);
  status |= t->serif.read_reg(&(t->serif), MPUREG_ACCEL_CONFIG1, &saved_regs->accel_config1, 1);
  status |= t->serif.read_reg(&(t->serif), MPUREG_GYRO_CONFIG0, &saved_regs->gyro_config0, 1);
  status |= t->serif.read_reg(&(t->serif), MPUREG_GYRO_CONFIG1, &saved_regs->gyro_config1, 1);
  status |= t->serif.read_reg(&(t->serif), MPUREG_ACCEL_GYRO_CONFIG0, &saved_regs->accel_gyro_config0, 1);
  status |= t->serif.read_reg(&(t->serif), MPUREG_FIFO_CONFIG1, &saved_regs->fifo_config1, 1);
  status |= t->serif.read_reg(&(t->serif), MPUREG_SELF_TEST_CONFIG, &saved_regs->self_test_config, 1);

  return status;
}


/**
  * @brief: recover settings for follow register
  *      - INTF_CONFIG1
  *      - PWR_MGMT_0
  *      - ACCEL_CONFIG0
  *      - ACCEL_CONFIG1
  *      - GYRO_CONFIG0
  *      - GYRO_CONFIG1
  *      - ACCEL_GYRO_CONFIG
  *      - FIFO_CONFIG1
  *      - SELF_TEST_CONFIG
  * @param:struct inv_icm426xx * s             
  * @retval 
 */ 
static BB_StatusTypeDef recover_settings(struct inv_icm426xx * s, struct recover_regs * saved_regs)
{
  struct inv_icm426xx_transport *t = (struct inv_icm426xx_transport *)s;
  BB_StatusTypeDef status = BB_NO_ERROR;

  /* Set en_g{x/y/z}_st_d2a to 0 disable self-test for each axis */
  status |= t->serif.write_reg(&(t->serif), MPUREG_SELF_TEST_CONFIG, &saved_regs->self_test_config, 1);
  /*Restore gyro_dec2_m2_ord, gyro_ui_filt_ord_ind and gyro_ui_filt_bw_ind to previous values.*/
  status |= t->serif.write_reg(&(t->serif), MPUREG_INTF_CONFIG1, &saved_regs->intf_config1, 1);
  status |= t->serif.write_reg(&(t->serif), MPUREG_PWR_MGMT_0, &saved_regs->pwr_mgmt_0, 1);
  status |= t->serif.write_reg(&(t->serif), MPUREG_ACCEL_CONFIG0, &saved_regs->accel_config0,1);
  status |= t->serif.write_reg(&(t->serif), MPUREG_ACCEL_CONFIG1, &saved_regs->accel_config1,1);
  status |= t->serif.write_reg(&(t->serif), MPUREG_GYRO_CONFIG0 , &saved_regs->gyro_config0,1);
  status |= t->serif.write_reg(&(t->serif), MPUREG_GYRO_CONFIG1 , &saved_regs->gyro_config1,1);
  status |= t->serif.write_reg(&(t->serif), MPUREG_FIFO_CONFIG1 , &saved_regs->fifo_config1,1);
  status |= t->serif.write_reg(&(t->serif), MPUREG_ACCEL_GYRO_CONFIG0, &saved_regs->accel_gyro_config0, 1);
  /* wait 200ms for gyro output to settle */
  HAL_Delay(200);

  //status |= inv_icm426xx_reset_fifo(s);

  return status;
}

static int reg_to_accel_fsr(ICM426XX_ACCEL_CONFIG0_FS_SEL_t reg)
{
	switch(reg) {
	case ICM426XX_ACCEL_CONFIG0_FS_SEL_2g:   return 2;
	case ICM426XX_ACCEL_CONFIG0_FS_SEL_4g:   return 4;
	case ICM426XX_ACCEL_CONFIG0_FS_SEL_8g:   return 8;
	case ICM426XX_ACCEL_CONFIG0_FS_SEL_16g:  return 16;
#if defined(ICM42686)
	case ICM426XX_ACCEL_CONFIG0_FS_SEL_32g:  return 32;
#endif
	default:                                 return -1;
	}
}

static int reg_to_gyro_fsr(ICM426XX_GYRO_CONFIG0_FS_SEL_t reg)
{
	switch(reg) {
#if !defined(ICM42686)
	case ICM426XX_GYRO_CONFIG0_FS_SEL_16dps:   return 16;
#endif
	case ICM426XX_GYRO_CONFIG0_FS_SEL_31dps:   return 31;
	case ICM426XX_GYRO_CONFIG0_FS_SEL_62dps:   return 62;
	case ICM426XX_GYRO_CONFIG0_FS_SEL_125dps:  return 125;
	case ICM426XX_GYRO_CONFIG0_FS_SEL_250dps:  return 250;
	case ICM426XX_GYRO_CONFIG0_FS_SEL_500dps:  return 500;
	case ICM426XX_GYRO_CONFIG0_FS_SEL_1000dps: return 1000;
	case ICM426XX_GYRO_CONFIG0_FS_SEL_2000dps: return 2000;
#if defined(ICM42686)
	case ICM426XX_GYRO_CONFIG0_FS_SEL_4000dps: return 4000;
#endif
	default:                                   return -1;
	}
}
//Self-test codes end


/**
  * @}
  */

/**
  * @}
  */



/*****END OF FILE****/


/********************************************************************************
  * @file    IMU/drivers/ICM42605.h
  * @author  Zhang Xiaodong
  * @version V0.1.0
  * @date    2020/5
  * @brief   ICM42605 Define file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2020 BeyondBits</center></h2>
  *
  *
  *******************************************************************************/

#ifndef ICM42605_h
#define ICM42605_h

#include "main.h"
#include "icm426xxDefs.h"

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/**Serif type 
  ICM426XX_UI_I2C,
  ICM426XX_UI_SPI4,
  ICM426XX_UI_I3C,
  ICM426XX_AUX1_SPI3,
  ICM426XX_AUX2_SPI3
**/

#define ACCEL_CONFIG0_FS_SEL_MAX ICM426XX_ACCEL_CONFIG0_FS_SEL_16g
#define GYRO_CONFIG0_FS_SEL_MAX  ICM426XX_GYRO_CONFIG0_FS_SEL_2000dps

#define ACCEL_OFFUSER_MAX_MG 1000 
#define GYRO_OFFUSER_MAX_DPS 64
/** @brief Icm426xx maximum buffer size mirrored from FIFO at polling time
 *  @warning fifo_idx type variable must be large enough to parse the FIFO_MIRRORING_SIZE
 */
#define ICM426XX_FIFO_MIRRORING_SIZE 16 * 129 // packet size * max_count = 2064

/** @brief Default value for the WOM threshold
 *  Resolution of the threshold is ~= 4mg
 */
#define ICM426XX_DEFAULT_WOM_THS_MG 52>>2 /* = 52mg/4 */

/** @brief Icm426xx Accelerometer start-up time before having correct data
 */
#define ICM426XX_ACC_STARTUP_TIME_US 20000U

/** @brief Icm426xx Gyroscope start-up time before having correct data
 */
#define ICM426XX_GYR_STARTUP_TIME_US 60000U

/** @brief Sensor identifier for UI control and OIS function
 */
enum inv_icm426xx_sensor {
  INV_ICM426XX_SENSOR_ACCEL,               /**< Accelerometer (UI control path) */
  INV_ICM426XX_SENSOR_GYRO,                /**< Gyroscope (UI control path) */
  INV_ICM426XX_SENSOR_FSYNC_EVENT,         /**< Used by OIS and UI control layers */
  INV_ICM426XX_SENSOR_OIS,                 /**< Only used by OIS layer */
  INV_ICM426XX_SENSOR_TEMPERATURE,         /**< Chip temperature, enabled by default. However, it will be reported only if Accel and/or Gyro are also enabled. 
                                                The Temperature's ODR (Output Data Rate) will match the ODR of Accel or Gyro, or the fastest if both are enabled*/
  INV_ICM426XX_SENSOR_TAP,                 /**< Tap and Double tap */
  INV_ICM426XX_SENSOR_DMP_PEDOMETER_EVENT, /**< Pedometer: step is detected */
  INV_ICM426XX_SENSOR_DMP_PEDOMETER_COUNT, /**< Pedometer: step counter */
  INV_ICM426XX_SENSOR_DMP_TILT,            /**< Tilt */
  INV_ICM426XX_SENSOR_DMP_R2W,             /**< Raise to wake */
  INV_ICM426XX_SENSOR_MAX
};

/** @brief Configure Fifo usage
 */
#define ICM426XX_FIFO_MIRRORING_SIZE 16 * 129 // packet size * max_count = 2064

typedef enum {
  INV_ICM426XX_FIFO_DISABLED = 0,              /**< Fifo is disabled and data source is sensors registers */
  INV_ICM426XX_FIFO_ENABLED  = 1,              /**< Fifo is used as data source */
}INV_ICM426XX_FIFO_CONFIG_t;

/** @brief Sensor event structure definition
 */
typedef struct {
  int sensor_mask;
  uint16_t timestamp_fsync;
  int16_t accel[3]; 
  int16_t gyro[3]; 
  int16_t temperature;
  int8_t accel_high_res[3];
  int8_t gyro_high_res[3];
} inv_icm426xx_sensor_event_t;

/** @brief enumeration  of serial interfaces available on icm42605 */
typedef enum
{
  ICM426XX_UI_I2C,
  ICM426XX_UI_SPI4,
  ICM426XX_UI_I3C,
  ICM426XX_AUX1_SPI3,
  ICM426XX_AUX2_SPI3
  
} ICM426XX_SERIAL_IF_TYPE_t;

/** @brief basesensor serial interface
 */
/**
when use dma transfer, make sure the rxbuff and txbuff is align to 4bytes
**/
#define ICM4_REGRDMAX (32)//max read out :one reg+31bytes
#define ICM4_REGWRMAX (ICM4_REGRDMAX<<1) //max write out :32 reg + 32bytes
#pragma pack(4) 
struct inv_icm426xx_serif {// use packed to make sure DMA buff is align
  void *     context;
  BB_StatusTypeDef      (*read_reg)(struct inv_icm426xx_serif * serif, uint8_t reg, uint8_t * rbuffer, uint8_t rlen);
  BB_StatusTypeDef      (*write_reg)(struct inv_icm426xx_serif * serif, uint8_t reg, uint8_t * wbuffer, uint16_t len);
  BB_StatusTypeDef      (*configure)(struct inv_icm426xx_serif * serif);
  uint32_t   max_read;
  uint32_t   max_write;
  uint8_t Rxbuff[ICM4_REGRDMAX];
  uint8_t Txbuff[ICM4_REGWRMAX];
  ICM426XX_SERIAL_IF_TYPE_t serif_type;
};
#pragma pack() 
/** @brief transport interface
 */
struct inv_icm426xx_transport {
  struct inv_icm426xx_serif serif; /**< Warning : this field MUST be the first one of struct inv_icm426xx_transport */

  /** @brief Contains mirrored values of some IP registers */
  struct register_cache {
    uint8_t intf_cfg_1_reg;   /**< INTF_CONFIG1, Bank: 0, Address: 0x4D */
    uint8_t pwr_mngt_0_reg;   /**< PWR_MGMT_0, Bank: 0, Address: 0x4E */
    uint8_t gyro_cfg_0_reg;   /**< GYRO_CONFIG0, Bank: 0, Address: 0x4F */
    uint8_t accel_cfg_0_reg;  /**< ACCEL_CONFIG0, Bank: 0, Address: 0x50 */
    uint8_t tmst_cfg_reg;     /**< TMST_CONFIG, Bank: 0, Address: 0x54 */
    uint8_t bank_sel_reg;     /**< MPUREG_REG_BANK_SEL, All banks, Address 0x76*/
  } register_cache; /**< Store mostly used register values on SRAM. 
                      *  MPUREG_OTP_SEC_STATUS_B1 and MPUREG_INT_STATUS registers
                      *  are read before the cache has a chance to be initialized. 
                      *  Therefore, these registers shall never be added to the cache 
            *  Registers from bank 1,2,3 or 4 shall never be added to the cache
                      */
};
/** @brief Icm426xx driver states definition
 */
struct inv_icm426xx {
  struct inv_icm426xx_transport transport;                              /**< Warning : this field MUST be the first one of 
                                                                     struct icm426xx */

  void (*sensor_event_cb)(inv_icm426xx_sensor_event_t * event); /**< callback executed by inv_icm426xx_get_data_from_fifo function
                                                                     for each data packet extracted from fifo or inv_icm426xx_get_data_from_registers read data from register
                                                                     This field may be NULL if inv_icm426xx_get_data_from_fifo/inv_icm426xx_get_data_from_registers
                                                                     is not used by application. */

  int gyro_st_bias[3];                                          /**< collected bias values (lsb) during self test */
  int accel_st_bias[3];
  uint8_t st_result;                                                   /**< Flag to keep track if self-test has been already run by storing acc and gyr results */

  uint8_t fifo_data[ICM426XX_FIFO_MIRRORING_SIZE];              /**<  FIFO mirroring memory area */

  uint8_t tmst_to_reg_en_cnt;                                   /**< internal counter to keep track of the timestamp to register access availability */
  
  uint8_t dmp_is_on;                                            /**< DMP started status */

  uint64_t gyro_start_time_us;                                  /**< internal state needed to discard first gyro samples */
  uint64_t accel_start_time_us;                                 /**< internal state needed to discard first accel samples */
  uint8_t endianess_data;                                       /**< internal status of data endianess mode to report correctly data */
  uint8_t fifo_highres_enabled;                                 /**< FIFO packets are 20 bytes long */
  INV_ICM426XX_FIFO_CONFIG_t fifo_is_used;                      /**< Data are get from FIFO or from sensor registers. By default Fifo is used*/
  
  #if (!INV_ICM426XX_LIGHTWEIGHT_DRIVER)
    /* First FSYNC event after enable is irrelevant 
     * When the sensors are switched off and then on again, an FSYNC tag with incorrect FSYNC value can be generated 
     * on the next first ODR. 
     * Solution: FSYNC tag and FSYNC data should be ignored on this first ODR.
     */
    uint8_t fsync_to_be_ignored;
  #endif
  
  /* Accel Low Power could report with wrong ODR if internal counter for ODR changed overflowed
   * WUOSC clock domain is informed through an 3bit counter that ODR
   * has changed in RC/PLL mode. Every 8 event, this counter overflows
   * and goes back to 0, same as 'no ODR changes', therefore previous ALP
   * ODR is re-used.
   * Solution: Keep track of ODR changes when WUOSC is disabled and perform dummy ODR changes when re-enabling WU after 8*n ODR changes.
   */
  uint32_t wu_off_acc_odr_changes;

  uint8_t wom_smd_mask;   /**< This variable keeps track if wom or smd is enabled */

  uint8_t wom_enable;   /**< This variable keeps track if wom is enabled */
  
  /* Software mirror of BW and AVG settings in hardware, to be re-applied on each enabling of sensor */
  struct {
    uint8_t acc_lp_avg; /**< Low-power averaging setting for accelerometer */
    uint8_t reserved;   /**< reserved field */
    uint8_t acc_ln_bw;  /**< Low-noise filter bandwidth setting for accelerometer */
    uint8_t gyr_ln_bw;  /**< Low-noise filter bandwidth setting for gyroscope */
  } avg_bw_setting;

  uint8_t gyro_status;
  uint8_t accel_status;
  uint8_t current_reg_bank;
  
  uint64_t gyro_power_off_tmst;   /**< This variable keeps track of timestamp when gyro is power off */
};

/* Interrupt enum state for INT1, INT2, and IBI */
typedef enum{
  INV_ICM426XX_DISABLE = 0,
  INV_ICM426XX_ENABLE
}inv_icm426xx_interrupt_value;

/** @brief Icm426xx set of interrupt enable flag
 */
typedef struct {
  inv_icm426xx_interrupt_value INV_ICM426XX_UI_FSYNC;
  inv_icm426xx_interrupt_value INV_ICM426XX_UI_DRDY;
  inv_icm426xx_interrupt_value INV_ICM426XX_FIFO_THS;
  inv_icm426xx_interrupt_value INV_ICM426XX_FIFO_FULL;
  inv_icm426xx_interrupt_value INV_ICM426XX_SMD;
  inv_icm426xx_interrupt_value INV_ICM426XX_WOM_X;
  inv_icm426xx_interrupt_value INV_ICM426XX_WOM_Y;
  inv_icm426xx_interrupt_value INV_ICM426XX_WOM_Z;
  inv_icm426xx_interrupt_value INV_ICM426XX_STEP_DET;
  inv_icm426xx_interrupt_value INV_ICM426XX_STEP_CNT_OVFL;
  inv_icm426xx_interrupt_value INV_ICM426XX_TILT_DET;
  inv_icm426xx_interrupt_value INV_ICM426XX_SLEEP_DET;
  inv_icm426xx_interrupt_value INV_ICM426XX_WAKE_DET;
  inv_icm426xx_interrupt_value INV_ICM426XX_TAP_DET;
}inv_icm426xx_interrupt_parameter_t;


/* USER CODE END Private defines */
BB_StatusTypeDef inv_icm426xx_init(struct inv_icm426xx * s, struct inv_icm426xx_serif * serif, void (*sensor_event_cb)(inv_icm426xx_sensor_event_t * event)); 
BB_StatusTypeDef inv_icm426xx_device_reset(struct inv_icm426xx * s);

BB_StatusTypeDef inv_io_hal_read_reg(struct inv_icm426xx_serif * serif, uint8_t reg, uint8_t * rbuffer, uint8_t rlen);
BB_StatusTypeDef inv_io_hal_write_reg(struct inv_icm426xx_serif * serif, uint8_t reg, uint8_t * wbuffer, uint16_t wlen);

BB_StatusTypeDef inv_icm426xx_set_reg_bank(struct inv_icm426xx * s, uint8_t bank);
BB_StatusTypeDef inv_icm426xx_get_who_am_i(struct inv_icm426xx * s, uint8_t * who_am_i);
BB_StatusTypeDef inv_icm426xx_ag_config(struct inv_icm426xx * s, 
                      ICM426XX_PWR_MGMT_0_ACCEL_MODE_t accel_mode,
                      ICM426XX_PWR_MGMT_0_GYRO_MODE_t gyro_mode,
                      ICM426XX_ACCEL_CONFIG0_FS_SEL_t acc_fsr_g,
                      ICM426XX_GYRO_CONFIG0_FS_SEL_t gyro_fsr_dps,
                      ICM426XX_ACCEL_CONFIG0_ODR_t acc_freq,
                      ICM426XX_GYRO_CONFIG0_ODR_t gyr_freq);

BB_StatusTypeDef inv_icm426xx_enable_gyro_low_noise_mode(struct inv_icm426xx * s);
BB_StatusTypeDef inv_icm426xx_enable_accel_low_power_mode(struct inv_icm426xx * s);
BB_StatusTypeDef inv_icm426xx_enable_accel_low_noise_mode(struct inv_icm426xx * s);
BB_StatusTypeDef inv_icm426xx_set_gyro_fsr_freq(struct inv_icm426xx * s,
            ICM426XX_GYRO_CONFIG0_FS_SEL_t gyro_fsr_dps,
            ICM426XX_GYRO_CONFIG0_ODR_t gyr_freq);
BB_StatusTypeDef inv_icm426xx_set_accel_fsr_freq(struct inv_icm426xx * s, 
              ICM426XX_ACCEL_CONFIG0_FS_SEL_t acc_fsr_g,
              ICM426XX_ACCEL_CONFIG0_ODR_t acc_freq);

BB_StatusTypeDef inv_icm426xx_set_config_int1(struct inv_icm426xx * s, inv_icm426xx_interrupt_parameter_t * interrupt_to_configure);
BB_StatusTypeDef inv_icm426xx_configure_fifo(struct inv_icm426xx * s, INV_ICM426XX_FIFO_CONFIG_t fifo_config);
BB_StatusTypeDef inv_icm426xx_get_config_int1(struct inv_icm426xx * s, inv_icm426xx_interrupt_parameter_t * interrupt_to_configure);

#endif//ICM42605_h


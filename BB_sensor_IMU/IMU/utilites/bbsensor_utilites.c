/*****************************************************************************
 *
 * @file:	D:\prj\BBSensor\IMU\utilites\bbsensor_utilites.c
 * @author: Xiaodong Zhang
 * @Email:	sacntumz@foxmail.com;zhangxiaodong@beyondbit.com
 * @date:	2020/06
 * @brief:  system utilites for Project BBSensor

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
#include "bbsensor_utilites.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
  
/* Private functions ---------------------------------------------------------*/
static int msg_level;
static bb_msg_printer_t msg_printer;

/** @addtogroup 
  * @{
  */

/**
  * @}
  */

/** @addtogroup BB_MSG_UART
  * @{
  */

void bb_msg_setup(int level, bb_msg_printer_t printer)
{
  msg_level   = level;
  if (level < BB_MSG_LEVEL_OFF)
    msg_level = BB_MSG_LEVEL_OFF;
  else if (level > BB_MSG_LEVEL_MAX)
    msg_level = BB_MSG_LEVEL_MAX;
  msg_printer = printer;
}


void bb_msg(int level, const char * str, ...)
{
  if(level && level <= msg_level && msg_printer) {
    va_list ap;
    va_start(ap, str);
    msg_printer(level, str, ap);
    va_end(ap);
  }
}


int bb_msg_get_level(void)
{
  return msg_level;
}


void bb_msg_level_setup(int level)
{
  msg_level   = level;
}

/**
  * @}
  */





/*****END OF FILE****/


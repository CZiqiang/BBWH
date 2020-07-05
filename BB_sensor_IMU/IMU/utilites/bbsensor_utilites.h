
/*****************************************************************************
 *
 * @file:	D:\prj\BBSensor\IMU\utilites\bbsensor_utilites.h 
 * @author: Xiaodong Zhang
 * @Email:	sacntumz@foxmail.com;zhangxiaodong@beyondbit.com
 * @date:	2020/06
 * @brief:  

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
#ifndef BBSENSOR_UTILITES_H
#define BBSENSOR_UTILITES_H
/******************************************************************************
 *  Include Files
 *****************************************************************************/
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
/******************************************************************************
 *  Macro Definitions
 *****************************************************************************/
#define BB_DEBUGMSG_ENABLE

#ifdef BB_DEBUGMSG_ENABLE
	#undef BB_DEBUGMSG_DISABLE
#endif

#define MSG_LEVEL BB_MSG_LEVEL_DEBUG
/******************************************************************************
 *  Const Definitions
 *****************************************************************************/

#if defined(BB_DEBUGMSG_DISABLE)
	#define BB_MSG(level, ...)           (void)0
	#define BB_MSG_SETUP(level, printer) (void)0
	#define BB_MSG_SETUP_LEVEL(level)    (void)0
	#define BB_MSG_LEVEL                 BB_MSG_LEVEL_OFF
#else
	#define BB_MSG(level, ...)           bb_msg(level, __VA_ARGS__)
 	#define BB_MSG_SETUP(level, printer) bb_msg_setup(level, printer)
	#define BB_MSG_SETUP_LEVEL(level)    bb_msg_level_setup(level)
	#define BB_MSG_LEVEL                 bb_msg_get_level()
#endif
 
/*****************************************************************************
 *  Type Definitions
 *****************************************************************************/
typedef enum 
{
	BB_NO_ERROR				    = 0,   /**< no error */
	BB_INV_ERROR			    = -1,  /**< unspecified error */
	BB_ERROR_NIMPL      	= -2,  /**< function not implemented for given
	                    	            arguments */
	BB_ERROR_TRANSPORT  	= -3,  /**< error occured at transport level */
	BB_ERROR_TIMEOUT    	= -4,  /**< action did not complete in the expected
	                                time window */
	BB_ERROR_SIZE         = -5,  /**< size/length of given arguments is not
	                                suitable to complete requested action */
	BB_ERROR_OS           = -6,  /**< error related to OS */
	BB_ERROR_IO          	= -7,  /**< error related to IO operation */
	BB_ERROR_MEM          = -9,  /**< not enough memory to complete requested
	                      	          action */
	BB_ERROR_HW           = -10, /**< error at HW level */
	BB_ERROR_BAD_ARG      = -11, /**< provided arguments are not good to
	                      	          perform requestion action */
	BB_ERROR_UNEXPECTED   = -12, /**< something unexpected happened */
	BB_ERROR_FILE         = -13, /**< cannot access file or unexpected format */
	BB_ERROR_PATH         = -14, /**< invalid file path */
	BB_ERROR_IMAGE_TYPE   = -15, /**< error when image type is not managed */
	BB_ERROR_WATCHDOG     = -16 /**< error when device doesn't respond 
                                  to ping */
} BB_StatusTypeDef;

 /** @brief message level definition
 */
 enum bb_msg_level {
	BB_MSG_LEVEL_OFF     = 0,
	BB_MSG_LEVEL_ERROR,
	BB_MSG_LEVEL_WARNING,
	BB_MSG_LEVEL_INFO,
	BB_MSG_LEVEL_VERBOSE,
	BB_MSG_LEVEL_DEBUG,
	BB_MSG_LEVEL_MAX
};

typedef void (*bb_msg_printer_t)(int level, const char * str, va_list ap);


/*****************************************************************************
 *  Extern Variables
 *****************************************************************************/
 
 
/*****************************************************************************
 *  Function Prototypes
 *****************************************************************************/

/*****************************************************************************
 *  Debug Message Function
 *****************************************************************************/
void bb_msg_setup(int level, bb_msg_printer_t printer);
void bb_msg(int level, const char * str, ...);
int bb_msg_get_level(void);
void bb_msg_level_setup(int level);

#endif /*BBSENSOR_UTILITES_H*/
/******************************************************************************
*
* Revision History:
*
* Rev.  YYMMDD  Who       Changes
* 1     2020/06 Xiaodong Zhang New Created.
******************************************************************************/


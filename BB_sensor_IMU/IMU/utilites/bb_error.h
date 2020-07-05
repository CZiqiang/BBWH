/*****************************************************************************
 *
 * @file:	D:\prj\BBSensor\IMU\utilites\bb_error.h 
 * @author: Xiaodong Zhang
 * @Email:	sacntumz@foxmail.com;zhangxiaodong@beyondbit.com
 * @date:	2020/06
 * @brief:  Error code define

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

#ifndef BB_ERROR_H
#define BB_ERROR_H

/******************************************************************************
 *  Include Files
 *****************************************************************************/
 
 
/******************************************************************************
 *  Macro Definitions
 *****************************************************************************/
 
 
/******************************************************************************
 *  Const Definitions
 *****************************************************************************/
 
 
/*****************************************************************************
 *  Type Definitions
 *****************************************************************************/
 
enum bb_error
{
	BB_NO_ERROR				= 0,   /**< no error */
	BB_INV_ERROR			= -1,  /**< unspecified error */
	BB_ERROR_NIMPL      	= -2,  /**< function not implemented for given
	                    	            arguments */
	BB_ERROR_TRANSPORT  	= -3,  /**< error occured at transport level */
	BB_ERROR_TIMEOUT    	= -4,  /**< action did not complete in the expected
	                                time window */
	BB_ERROR_SIZE         	= -5,  /**< size/length of given arguments is not
	                                suitable to complete requested action */
	BB_ERROR_OS           	= -6,  /**< error related to OS */
	BB_ERROR_IO          	= -7,  /**< error related to IO operation */
	BB_ERROR_MEM          	= -9,  /**< not enough memory to complete requested
	                      	          action */
	BB_ERROR_HW           	= -10, /**< error at HW level */
	BB_ERROR_BAD_ARG      	= -11, /**< provided arguments are not good to
	                      	          perform requestion action */
	BB_ERROR_UNEXPECTED   	= -12, /**< something unexpected happened */
	BB_ERROR_FILE         	= -13, /**< cannot access file or unexpected format */
	BB_ERROR_PATH         	= -14, /**< invalid file path */
	BB_ERROR_IMAGE_TYPE   	= -15, /**< error when image type is not managed */
	BB_ERROR_WATCHDOG     	= -16, /**< error when device doesn't respond 
									   to ping */
};
 
/*****************************************************************************
 *  Extern Variables
 *****************************************************************************/
 
 
/*****************************************************************************
 *  Function Prototypes
 *****************************************************************************/
 
#endif /*BB_ERROR_H*/
/******************************************************************************
*
* Revision History:
*
* Rev.  YYMMDD  Who       Changes
* 1     2020/06 Xiaodong Zhang New Created.
******************************************************************************/


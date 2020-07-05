/**
  ******************************************************************************
  * @file    IMU/utilites/ano_bbuf.c
  * @author  Zhang Xiaodong
  * @version V0.1.0
  * @date    2020/05
  * @brief   user frame implementation for ANO UPPer
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2020 BeyondBits</center></h2>
  *
  *
  ******************************************************************************
 */

 /* Includes ------------------------------------------------------------------*/
#include "ano_bbuf.h"
#include "usart.h"

/** @addtogroup 
  * @{
  */

#define BYTE0(dwTemp)		(*(char *)(&dwTemp))
#define BYTE1(dwTemp)		(*((char *)(&dwTemp)+1))
#define BYTE2(dwTemp)		(*((char *)(&dwTemp)+2))
#define BYTE3(dwTemp)		(*((char *)(&dwTemp)+3))

/** @addtogroup 
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

//frame define
#define ANOHEAD   0xAA
#define ANODADDR  0xFF

//user define frame
#define ANOIDF1   0xF1
#define ANOIDF2   0xF2
#define ANOIDF3   0xF3
#define ANOIDF4   0xF4
#define ANOIDF5   0xF5
#define ANOIDF6   0xF6
#define ANOIDF7   0xF7
#define ANOIDF8   0xF8
#define ANOIDF9   0xF9
#define ANOIDFA   0xFA

#define ANOF1LENTH  (6)//get two bytes in one axe

//IMU frame
#define ANOIMUID	0x01
#define ANOIMULENTH  (13)

//compass,pressure and temperature frame
#define ANOCPTID	0x02
#define ANOCPTLENTH  (14)

//Euler Angle frame
#define ANOIEULERID	0x03
#define ANOEULFRAMLENTH  (7)

//Quaternion frame
#define ANOIQUATID	0x04
#define ANOQUATFRAMLENTH  (9)

//Altitude frame
#define ANOIALTID	0x05
#define ANOALTFRAMLENTH  (9)



/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
  
/* Private functions ---------------------------------------------------------*/

uint8_t DataFramBuffer[100];


void trans_Q16_send(fix16_t *acc)
{
	float acc_f[3]={0};
	int32_t acc_t[3];
	for(uint8_t i=0;i<3;i++)
	{
		acc_f[i]=(float)acc[i]/65536;
		acc_t[i]=acc_f[i]*1000;
	}
	send_ANO_acc(acc_t);
}

void send_ANO_acc(int32_t *acc)
{
  uint8_t index = 0;
  uint8_t sumcheck = 0;//sum check byte
  uint8_t addcheck = 0;//additional check byte
  int32_t accx,accy,accz;
  accx = acc[0];
  accy = acc[1];
  accz = acc[2];
	while(huart1.gState!=HAL_UART_STATE_READY);
  DataFramBuffer[index++] = ANOHEAD;
  DataFramBuffer[index++] = ANODADDR;
  DataFramBuffer[index++] = ANOIDF1;
  DataFramBuffer[index++] = 12;

	DataFramBuffer[index++] = BYTE0(accx);
  DataFramBuffer[index++] = BYTE1(accx);
  DataFramBuffer[index++] = BYTE2(accx);
  DataFramBuffer[index++] = BYTE3(accx);

  DataFramBuffer[index++] = BYTE0(accy);
  DataFramBuffer[index++] = BYTE1(accy);
  DataFramBuffer[index++] = BYTE2(accy);
  DataFramBuffer[index++] = BYTE3(accy);

  DataFramBuffer[index++] = BYTE0(accz);
  DataFramBuffer[index++] = BYTE1(accz);
  DataFramBuffer[index++] = BYTE2(accz);
  DataFramBuffer[index++] = BYTE3(accz);
	
	for(uint8_t i=0; i < (DataFramBuffer[3] + 4); i++) 
  {
    sumcheck += DataFramBuffer[i]; 
    addcheck += sumcheck; 
  }

  DataFramBuffer[index++] = sumcheck;
  DataFramBuffer[index++] = addcheck;

  //BB_UART_TxDMA(&halBBuart,DataFramBuffer,index);
	while(HAL_UART_Transmit_DMA(&huart1,DataFramBuffer,index)!=HAL_OK);
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT BeyondBits *****END OF FILE****/

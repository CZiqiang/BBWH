/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "inv_app.h"

#include "ano_bbuf.h"
#include "fix16.h"
#ifdef BBINVST
#include "icm42605SelfTest.h"
#endif //BBINVST

#define OPERATION_SUM 0x01    
#define OPERATION_SUB 0x02  
#define OPERATION_MUL 0x03    
#define OPERATION_DIV 0x04   

void SystemClock_Config(void);

static void bb_icm42605_serif_init(struct inv_icm426xx_serif * icm_serif);
//BB_INV_BOARD_SPI_DEVICE spi_sel_device;
//void BB_UART_debug(const char *format, ...); 
void bb_uart_debug_printer(int level, const char * str, va_list ap);
void Test_Delay_fix16(fix16_t a,fix16_t b,uint8_t def);
void Test_Delay_float(float a,float b,uint8_t def);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  //IF Setup
  //struct inv_icm426xx_serif __attribute__((aligned (4))) icm426xx_serif;
  struct inv_icm426xx_serif icm426xx_serif;
//	int32_t datatemp[3] = {0};
//	float temp[3]={0.00123,0.00234,0.00345};
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();

  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  //int rc;

/* Setup message facility to see internal traces from FW */
  BB_MSG_SETUP(MSG_LEVEL, bb_uart_debug_printer);
  
// for debug 
// can be removed
 
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

	printf("-----------Calculate Test------------\n");
	float temp_float_a = 123.1415926;
	float temp_float_b = 256.1354678;
//	BB_MSG(BB_MSG_LEVEL_INFO,"Address of TEMP:0x%8x",temp_float_a);
//	BB_MSG(BB_MSG_LEVEL_INFO,"Lenght of TEMP:%d",sizeof(temp_float_a));
//	BB_MSG(BB_MSG_LEVEL_INFO,"TEMP[0]:0x%8x",temp_float_a);
//	BB_MSG(BB_MSG_LEVEL_INFO,"TEMP[1]:0x%8x",temp_float_b);
	fix16_t temp_fix16_a=temp_float_a*65536.0f;  
	fix16_t temp_fix16_b=temp_float_b*65536.0f; //trans to q16.16

	BB_MSG(BB_MSG_LEVEL_INFO,"-----------Calculate Float Data-----------");
//	fix16_t testsum=0,testsub=0; //caculate result
	Test_Delay_float(temp_float_a,temp_float_b,OPERATION_SUM);
	Test_Delay_float(temp_float_a,temp_float_b,OPERATION_SUB);
	Test_Delay_float(temp_float_a,temp_float_b,OPERATION_MUL);
	Test_Delay_float(temp_float_a,temp_float_b,OPERATION_DIV);
	
	BB_MSG(BB_MSG_LEVEL_INFO,"-----------Calculate Q16 Data-------------");
	Test_Delay_fix16(temp_fix16_a,temp_fix16_b,OPERATION_SUM);
	Test_Delay_fix16(temp_fix16_a,temp_fix16_b,OPERATION_SUB);
	Test_Delay_fix16(temp_fix16_a,temp_fix16_b,OPERATION_MUL);
	Test_Delay_fix16(temp_fix16_a,temp_fix16_b,OPERATION_DIV);
	while(1){};
		
//  while(1)
//  {
//    HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_15);
//    BB_Delay_bytick(1000);
//  }
  //init for icm42605 serif st
  bb_icm42605_serif_init(&icm426xx_serif);
  //setup icm42605
  SetupInvDevice(&icm426xx_serif);
#ifdef BBINVST
  RunSelfTestInv();//run self test for inv
#endif //BBINVST
////  GetInvBias();
  //rc = SetupInvDevice(&icm426xx_serif);
  //check_rc(rc, "error while setting up INV device");
  
  /* Perform Self-Test */
//  RunSelfTest();

  /* USER CODE END 2 */

	
//	int tickOver1 = HAL_GetTick()-tickStart;
//	testsub = fix16_sub(datatemp[0],datatemp[1]);
//	int tickOver2 = HAL_GetTick()-tickOver1;
	
//	HAL_Delay(500);
//	printf("Asub:%f   %dms\n",(float)testsub/65536,tickOver2);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  while (1)
//  {

//  }
}

//systick_per_us = systick_load * SYSTICK_Frequency / 1000 / 1000;
//systick_per_us = 8000*1000/1000/1000=8；//Change value of 1us sys counter
void Test_Delay_fix16(fix16_t a,fix16_t b,uint8_t def)
{
  uint32_t systick_pre = 0;
  uint32_t systick_now = 0;
	
  uint32_t delay_total = 0;
  uint8_t systick_per_us = 8;
  
  //systick_load = HAL_RCC_GetHCLKFreq()/SYSTICK_Frequency;  //8000000/1000=8000
  uint32_t systick_load = 8000;
	fix16_t testvalue=0;
	float testvalue_float = 0;
	float overtime=0;
//	BB_MSG(BB_MSG_LEVEL_INFO,"Address of systick_pre:0x%8x",&systick_pre);
//	BB_MSG(BB_MSG_LEVEL_INFO,"Address of systick_pre:0x%8x",&systick_now);
//	uint16_t start = HAL_GetTick();
	SysTick->VAL = 0xFFFFFF;
	switch(def)
	{
		case OPERATION_SUM:
			systick_pre = SysTick->VAL;
			for(int i=0;i<100;i++)
				testvalue = fix16_add(a,b);  //
			systick_now = SysTick->VAL;		
			break;
		case OPERATION_SUB:
			systick_pre = SysTick->VAL;
			for(int i=0;i<100;i++)
				testvalue = fix16_sub(a,b);  //
			systick_now = SysTick->VAL;
			break;
		case OPERATION_MUL:
			systick_pre = SysTick->VAL;
			for(int i=0;i<100;i++)
				testvalue = fix16_mul(a,b);  //
			systick_now = SysTick->VAL;
			break;
		case OPERATION_DIV:
			systick_pre = SysTick->VAL;
			for(int i=0;i<100;i++)
				testvalue = fix16_div(a,b);  //
			systick_now = SysTick->VAL;
			break;
		default :
			break;
	}
//	uint16_t stop = HAL_GetTick();
//	if((stop-start)!=0)
//		BB_MSG(BB_MSG_LEVEL_DEBUG,"flow Over!");
	
	testvalue_float= (float)(testvalue/(float)(1<<16));
	
  if (systick_now <= systick_pre)  //no reload for a period
    delay_total = systick_pre-systick_now;
  else
    delay_total = systick_load-(systick_now-systick_pre);
	overtime = (float)delay_total/systick_per_us/1000; //trans us to ms
	switch(def)
	{
		case OPERATION_SUM:
			BB_MSG(BB_MSG_LEVEL_INFO,"OPERATION_SUM=%f  overtime:%f ms\n",testvalue_float,overtime);
			break;
		case OPERATION_SUB:
			BB_MSG(BB_MSG_LEVEL_INFO,"OPERATION_SUB=%f  overtime:%f ms\n",testvalue_float,overtime);
			break;
		case OPERATION_MUL:
			BB_MSG(BB_MSG_LEVEL_INFO,"OPERATION_MUL=%f  overtime:%f ms\n",testvalue_float,overtime);
			break;
		case OPERATION_DIV:
			BB_MSG(BB_MSG_LEVEL_INFO,"OPERATION_DIV=%f  overtime:%f ms\n",testvalue_float,overtime);
			break;
		default:
			break;
	}
}

void Test_Delay_float(float a,float b,uint8_t def)
{
	uint32_t systick_pre = 0;
  uint32_t systick_now = 0;
	
  uint32_t delay_total = 0;
  uint8_t systick_per_us = 8;
  
  //systick_load = HAL_RCC_GetHCLKFreq()/SYSTICK_Frequency;  //8000000/1000=8000
  uint32_t systick_load = 8000;
	float testvalue=0;
//	float testvalue_float = 0;
	float overtime=0;
//	BB_MSG(BB_MSG_LEVEL_INFO,"Address of systick_pre:0x%8x",&systick_pre);
//	BB_MSG(BB_MSG_LEVEL_INFO,"Address of systick_pre:0x%8x",&systick_now);
	uint32_t startTick,stopTick;
	SysTick->VAL = 0xFFFFFF;
	switch(def)
	{
		case OPERATION_SUM:
			
			systick_pre = SysTick->VAL;
//			for(int i=0;i<100;i++)
				testvalue = a+b;  //
			systick_now = SysTick->VAL;		
			break;
		case OPERATION_SUB:
			systick_pre = SysTick->VAL;
//			for(int i=0;i<100;i++)
				testvalue = a-b;  //
			systick_now = SysTick->VAL;
			break;
		case OPERATION_MUL:		
			startTick = HAL_GetTick();
			systick_pre = SysTick->VAL;
			for(int i=0;i<1000;i++)
				testvalue = a*b;  //
//			HAL_Delay(100);
			systick_now = SysTick->VAL;
			stopTick = HAL_GetTick();
			BB_MSG(BB_MSG_LEVEL_INFO,"Interval time: %d",stopTick-startTick);
			break;
		case OPERATION_DIV:
			systick_pre = SysTick->VAL;
//			for(int i=0;i<100;i++)
				testvalue = a/b;  //
			systick_now = SysTick->VAL;
			break;
		default :
			break;
	}

//	BB_MSG(BB_MSG_LEVEL_DEBUG,"flow Over!");
  if (systick_now <= systick_pre)  //no reload for a period
    delay_total = systick_pre-systick_now;
  else
    delay_total = systick_load-(systick_now-systick_pre);
	overtime = (float)delay_total/systick_per_us/1000; //trans us to ms
	switch(def)
	{
		case OPERATION_SUM:
			BB_MSG(BB_MSG_LEVEL_INFO,"OPERATION_SUM=%f  overtime:%f ms\n",testvalue,overtime);
			break;
		case OPERATION_SUB:
			BB_MSG(BB_MSG_LEVEL_INFO,"OPERATION_SUB=%f  overtime:%f ms\n",testvalue,overtime);
			break;
		case OPERATION_MUL:
			BB_MSG(BB_MSG_LEVEL_INFO,"OPERATION_MUL=%f  overtime:%f ms\n",testvalue,overtime);
			break;
		case OPERATION_DIV:
			BB_MSG(BB_MSG_LEVEL_INFO,"OPERATION_DIV=%f  overtime:%f ms\n",testvalue,overtime);
			break;
		default:
			break;
	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief This function configures the source of the time base. 
  *        The time source is configured  to have 1ms time base with a dedicated 
  *        Tick interrupt priority.
  * @note This function is called  automatically at the beginning of program after
  *       reset by HAL_Init() or at any time when clock is reconfigured  by HAL_RCC_ClockConfig(). 
  * @note In the default implementation, SysTick timer is the source of time base. 
  *       It is used to generate interrupts at regular time intervals. 
  *       Care must be taken if HAL_Delay() is called from a peripheral ISR process, 
  *       The SysTick interrupt must have higher priority (numerically lower) 
  *       than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
  *       The function is declared as __Weak  to be overwritten  in case of other
  *       implementation  in user file.
  * @param TickPriority Tick interrupt priority.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  /*Configure the SysTick to have interrupt in 1ms time basis*/
  if (HAL_SYSTICK_Config(SystemCoreClock / (BBSYSFRAC / uwTickFreq)) > 0U)
  {
    return HAL_ERROR;
  }

  /* Configure the SysTick IRQ priority */
  if (TickPriority < (1UL << __NVIC_PRIO_BITS))
  {
    HAL_NVIC_SetPriority(SysTick_IRQn, TickPriority, 0U);
    uwTickPrio = TickPriority;
  }
  else
  {
    return HAL_ERROR;
  }

   /* Return function status */
  return HAL_OK;
}

/**
  * @brief This function provides accurate delay (in milliseconds) based 
  *        on variable incremented.
  * @note In the default implementation , SysTick timer is the source of time base.
  *       It is used to generate interrupts at regular time intervals where uwTick
  *       is incremented.
  * @note ThiS function is declared as __weak to be overwritten in case of other
  *       implementations in user file.
  * @param Delay specifies the delay time length, in milliseconds.
  * @retval None
  */
void BB_Delay_bytick(uint32_t Delay)
{
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait = Delay;
  
  /* Add a freq to guarantee minimum wait */
  if (wait < HAL_MAX_DELAY)
  {
    wait += (uint32_t)(uwTickFreq);
  }
  
  while((HAL_GetTick() - tickstart) < wait)
  {
  }
}

/**
  * @brief  This function configures:
 *  - ICM42605 Struct SETUP
 *  - a serial link to communicate from MCU to Icm426xx
 *  - setup Txbuff and Rxbuff for reg read
 *  - setup max_read and max_write for icm42605 internal FIFO read
 *  @
  * @retval None
  */
void bb_icm42605_serif_init(struct inv_icm426xx_serif * icm_serif)
{
  BB_MSG(BB_MSG_LEVEL_DEBUG, "#ICM42605 Struct SETUP#");

  uint8_t temp_txarray[ICM4_REGWRMAX] = {0};
  uint8_t temp_rxarray[ICM4_REGRDMAX] = {0};
 
  /* Initialize serial inteface between MCU and Icm426xx */
  icm_serif->context   = 0;        /* no need */
  icm_serif->read_reg  = inv_io_hal_read_reg;
  icm_serif->write_reg = inv_io_hal_write_reg;
  icm_serif->max_read  = 1024*32;  /* maximum number of bytes allowed per serial read */
  icm_serif->max_write = 1024*32;  /* maximum number of bytes allowed per serial write */
  icm_serif->serif_type = ICM_SERIF_TYPE;
  if(ICM_SERIF_TYPE == ICM426XX_UI_SPI4)
    {
      BB_MSG(BB_MSG_LEVEL_INFO,"ICM Serif is SPI4.SET PA4(SPICS) to HIGH and Init SPI");
      BB_SPI_CS_DISABLE(BB_ICM42605); 
      MX_SPI1_Init();
    } 
  else if(ICM_SERIF_TYPE == ICM426XX_UI_I2C)
    {
      BB_MSG(BB_MSG_LEVEL_INFO,"ICM Serif is I2C");
      MX_I2C1_Init();
    }
  //else if(ICM_SERIF_TYPE == ICM426XX_UI_SPI4)
  else
    BB_MSG(BB_MSG_LEVEL_ERROR,"ICM Serif define is not valid");

  memcpy(icm_serif->Txbuff, temp_txarray, ICM4_REGWRMAX*sizeof(uint8_t));
  memcpy(icm_serif->Rxbuff, temp_rxarray, ICM4_REGRDMAX*sizeof(uint8_t));
  BB_MSG(BB_MSG_LEVEL_DEBUG, "#ICM42605 Struct SETUP DONE#");
  // BB_MSG(BB_MSG_LEVEL_DEBUG, "Sizeof icmserif struct: %d",sizeof(*icm_serif));
  // BB_MSG(BB_MSG_LEVEL_DEBUG, "icm_serif address:  0x%8x",icm_serif);
  // BB_MSG(BB_MSG_LEVEL_DEBUG, "context address:    0x%8x",&icm_serif->context);  
  // BB_MSG(BB_MSG_LEVEL_DEBUG, "read_reg address:   0x%8x",&icm_serif->read_reg); 
  // BB_MSG(BB_MSG_LEVEL_DEBUG, "write_reg address:  0x%8x",&icm_serif->write_reg);  
  // BB_MSG(BB_MSG_LEVEL_DEBUG, "configure address:  0x%8x",&icm_serif->configure);  
  // BB_MSG(BB_MSG_LEVEL_DEBUG, "max_read address:   0x%8x",&icm_serif->max_read);
  // BB_MSG(BB_MSG_LEVEL_DEBUG, "max_write address:  0x%8x",&icm_serif->max_write);
  
  // BB_MSG(BB_MSG_LEVEL_DEBUG, "Rxbuff address:     0x%8x",&icm_serif->Rxbuff);
  // BB_MSG(BB_MSG_LEVEL_DEBUG, "Txbuff address:     0x%8x",&icm_serif->Txbuff);  

  // BB_MSG(BB_MSG_LEVEL_DEBUG, "serif_type address: 0x%8x",&icm_serif->serif_type);
}

/**
  * @brief  This function realized the debug printer 
  * 
  * @retval None
**/
void bb_uart_debug_printer(int level, const char * str, va_list ap)
{

  static char out_str[128]; /* static to limit stack usage */
  uint16_t idx = 0;
  
  const char * s[BB_MSG_LEVEL_MAX] = {
      "",    // INV_MSG_LEVEL_OFF
      "[E]", // INV_MSG_LEVEL_ERROR
      "[W]", // INV_MSG_LEVEL_WARNING
      "[I]", // INV_MSG_LEVEL_INFO
      "[V]", // INV_MSG_LEVEL_VERBOSE
      "[D]", // INV_MSG_LEVEL_DEBUG
  };
  
  //put the while code before to change the out_str
  while(huart1.gState!=HAL_UART_STATE_READY);
  idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "%s: ", s[level]);
  if(idx >= (sizeof(out_str)))
    return;
  idx += vsnprintf(&out_str[idx], sizeof(out_str) - idx, str, ap);
  if(idx >= (sizeof(out_str)))
    return;
  idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "\r\n");
  if(idx >= (sizeof(out_str)))
    return;

  while(HAL_UART_Transmit_DMA(&huart1,(uint8_t*)out_str,idx)!=HAL_OK);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  BB_MSG(BB_MSG_LEVEL_ERROR,"Wrong parameters: file %s on line %d",file,line);
  while(1)
  {
    //also can add sth to do.
  }
  /* USER CODE END 6 */
}
// redirect printf()
int fputc(int ch,FILE *f)
{
    uint8_t temp[1]={ch};
    HAL_UART_Transmit(&huart1,temp,1,2);
	return 0;
}
#endif /* USE_FULL_ASSERT */



/************************ (C) COPYRIGHT BeyondBit *****END OF FILE****/

/******************************************************************
**	  红龙开发板（V1.0）
**	  系统时钟配置文件
**
**	  论    坛：bbs.openmcu.com
**	  旺    宝：www.openmcu.com
**	  邮    箱：support@openmcu.com
**
**    版    本：V1.0
**	  作    者：FXL
**	  完成日期:	2012.7.20
********************************************************************/
#include "stm32f10x.h"
#include <stdio.h>
#include "sys_config.h"

/********************************************************************************************
*函数名称：void RCC_Configuration(void)
*
*入口参数：无
*
*出口参数：无
*
*功能说明：系统时钟初始化配置
*	       RCC_HSICmd(ENABLE);//使能内部高速晶振 ;
* 	       RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);//选择内部高速时钟作为系统时钟SYSCLOCK=8MHZ	
*	       RCC_HCLKConfig(RCC_SYSCLK_Div1);       //选择HCLK时钟源为系统时钟SYYSCLOCK
*  	       RCC_PCLK1Config(RCC_HCLK_Div4);        //APB1时钟为2M 
*  	       RCC_PCLK2Config(RCC_HCLK_Div4);        //APB2时钟为2M
*  	       RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);//使能APB2外设GPIOB时钟
*******************************************************************************************/
void RCC_Configuration(void) 
{
	/* RCC system reset(for debug purpose) */
  	RCC_DeInit();

  	/* Enable HSE */
  	RCC_HSEConfig(RCC_HSE_ON);

  	/* Wait till HSE is ready */
  	while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET);

    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);
 
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1);
  
    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1);

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* PLLCLK = 8MHz * 9 = 72 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

    /* Enable PLL */ 
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08);
}

void NVIC_Configuration(void)
{
	#ifdef  VECT_TAB_RAM  
	  /* Set the Vector Table base location at 0x20000000 */ 
	  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
	#else  /* VECT_TAB_FLASH  */
	  /* Set the Vector Table base location at 0x08000000 */ 
	  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
	#endif	
}

void GPIO_Configuration(void)
{
	
}


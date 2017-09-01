/**
  ******************************************************************************
  * @file    stm32_usart1.h
 
  
  ******************************************************************************/



#ifndef __STM32_USART1_H
#define __STM32_USART1_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
   
 void GPIO_Configuration(void);
 void USART1_Configuration(void);
 void USART1_Init(void);

void USART1_SendByte(uint8_t data);
void USART1_SendString(char *s);
void USART1_SendData(uint8_t data);
 
 #ifdef __cplusplus
}
#endif

#endif
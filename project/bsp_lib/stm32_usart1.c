
/*******************************************************************************

* Copyright (C) 2015 Maxim Integrated Products, Inc., All Rights Reserved.

*

* Permission is hereby granted, free of charge, to any person obtaining a

* copy of this software and associated documentation files (the "Software"),

* to deal in the Software without restriction, including without limitation

* the rights to use, copy, modify, merge, publish, distribute, sublicense,

* and/or sell copies of the Software, and to permit persons to whom the

* Software is furnished to do so, subject to the following conditions:

*

* The above copyright notice and this permission notice shall be included

* in all copies or substantial portions of the Software.

*

* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS

* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF

* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.

* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES

* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,

* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR

* OTHER DEALINGS IN THE SOFTWARE.

*

* Except as contained in this notice, the name of Maxim Integrated 

* Products, Inc. shall not be used except as stated in the Maxim Integrated 

* Products, Inc. Branding Policy.

*

* The mere transfer of this software does not imply any licenses

* of trade secrets, proprietary technology, copyrights, patents,

* trademarks, maskwork rights, or any other form of intellectual

* property whatsoever. Maxim Integrated Products, Inc. retains all 

* ownership rights.

*******************************************************************************

*/

/************************************************************************

* 

* File Name:    stm32_usart1.c

* Dependencies: stm32_usart1.h

* Processor:    Stm32F103

* Author:       Frank Gao

* Version:      1.0

* Release Date: Oct 22, 2015

* Comments:     Initial Release

*

************************************************************************/


/** Note
  * It uses usart1 in this software 
**/

#include "stm32_usart1.h"
#include "stdlib.h"
#include <string.h>

void GPIO_Configuration(void)

{

  GPIO_InitTypeDef GPIO_InitStructure;  //

  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;      

  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  

  GPIO_Init(GPIOA, &GPIO_InitStructure);  

 

 

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;  

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  

  GPIO_Init(GPIOA, &GPIO_InitStructure);  

}

void USART1_Configuration(void)
{
  USART_InitTypeDef USART_InitStructure;
  
  USART_InitStructure.USART_BaudRate = 115200;                   //set baud rate

   USART_InitStructure.USART_WordLength = USART_WordLength_8b;    //set word length

   USART_InitStructure.USART_StopBits = USART_StopBits_1;        //set stop bit

   USART_InitStructure.USART_Parity = USART_Parity_No;        //set parity bit

   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardwareflow control

   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//enable receive and send

   USART_Init(USART1, &USART_InitStructure);                      

   USART_Cmd(USART1, ENABLE);    // enable USART1

}

void USART1_Init(void)
{
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1| RCC_APB2Periph_AFIO, ENABLE);  
 GPIO_Configuration();
 USART1_Configuration();
}

void USART1_SendByte(uint8_t data)
{
   while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
   USART_SendData(USART1, data);
   while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);// 
}

void USART1_SendString(char *s)
{
  uint16_t length,i;
  char *pointer;
  char x;
  
  length = strlen(s);
  pointer=s;
  for(i=0;i<length;i++)
  {  x=*pointer;
     USART1_SendByte(x);
     pointer++;
    
  }
  
}

void USART1_SendData(uint8_t data)
{
 USART1_SendByte(data/16+'0');
 USART1_SendByte(data%16+'0');
}
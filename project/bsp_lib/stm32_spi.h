
/** \file stm32_spi.h ******************************************************
 *
 
 *  --------------------------------------------------------------------
 *
 *  This code follows the following naming conventions:
 *
 *\n    char                    	ch_pmod_value
 *\n    char (array)            s_pmod_string[16]
 *\n    float                  	 f_pmod_value
 *\n    int                    	 n_pmod_value
 *\n    int (array)             	an_pmod_value[16]
 *\n    u16                     	u_pmod_value
 *\n    u16 (array)             au_pmod_value[16]
 *\n    u8                     	 uch_pmod_value
 *\n    u8 (array)              	auch_pmod_buffer[16]
 *\n    unsigned int     	un_pmod_value
 *\n    int *                   	pun_pmod_value
 *
 *  ------------------------------------------------------------------------- */

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __STM32_SPI_H
#define __STM32_SPI_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
   
#define FLAG_TIMEOUT         ((uint32_t)0x1000)
#define LONG_TIMEOUT         ((uint32_t)(10 *FLAG_TIMEOUT))

// define SPI pins
#define SPI_SCK       GPIO_Pin_5    /* GPIOA.5*/
#define SPI_MISO      GPIO_Pin_6    /*GPIOA.6*/
#define SPI_MOSI      GPIO_Pin_7    /*GPIOA.7*/
#define SPI_CS        GPIO_Pin_6    /*GPIOB.6*/
#define DRDY_PIN      GPIO_Pin_5    /*GPIOB.5*/

#define SPI_GPIO_CLK             RCC_APB2Periph_GPIOA
#define SPI_CS_GPIO_CLK          RCC_APB2Periph_GPIOB
#define DRDY_GPIO_CLK            RCC_APB2Periph_GPIOB


#define SPI_CS_HIGH              GPIO_SetBits(GPIOB, SPI_CS)
#define SPI_CS_LOW               GPIO_ResetBits(GPIOB, SPI_CS)
   
   
#define USE_DEFAULT_CRITICAL_CALLBACK 
   
void  SPI_Inital(void);
uint8_t SPI_Write(uint8_t *buffer, uint8_t nBytes);
uint8_t SPI_Read(uint8_t *buffer, uint8_t nBytes);  
uint8_t SPI_WriteByte(uint8_t data);
uint8_t SPI_ReadByte(void);
void Set_DRDY_Pin_INPUT(void);
uint8_t DRDY_Pin_Value(void);
void Enable_Exti(void);    //enable external interrupt
    
void EnterCriticalSection_UserCallback(void);
void ExitCriticalSection_UserCallback(void);   
#ifdef __cplusplus
}
#endif

#endif
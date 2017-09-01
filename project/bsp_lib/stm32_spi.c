/** \file stm32_spi.c ******************************************************
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
/*
 
 *
 ***************************************************************************/



#include "stm32_spi.h"

__IO uint32_t    TIMEOUT=LONG_TIMEOUT;


    
/************************************************************************

* 

* Name:         spi_inital

* Dependencies: stm32f10x_spi.c stm32f10x_rcc.c stm32f10x_gpio.c

* Return:       no

* Description:  This code is executed  to initial spi 

************************************************************************/
   
void  SPI_Inital(void)
{
 SPI_InitTypeDef  SPI_InitStructure;
 GPIO_InitTypeDef  GPIO_InitStructure;
    
  //Inital GPIO
   
   RCC_APB2PeriphClockCmd(SPI_GPIO_CLK, ENABLE);
   RCC_APB2PeriphClockCmd(SPI_CS_GPIO_CLK, ENABLE);
 
   /*!< SPI Periph clock enable */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
   
   /*!< GPIO configuration */  
   /*!< Configure SPI pins */
   GPIO_InitStructure.GPIO_Pin = SPI_SCK|SPI_MOSI|SPI_MISO;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
   
   
   /*!< Configure SPI pin: CS */
   GPIO_InitStructure.GPIO_Pin = SPI_CS;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_Init(GPIOB, &GPIO_InitStructure); 
   
   //set CS high
    GPIO_SetBits(GPIOB, SPI_CS); 
    
   /*!< SPI configuration */
   SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
   SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
   SPI_InitStructure.SPI_DataSize =SPI_DataSize_8b;
   SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
   SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
   SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
   SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
   SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
   SPI_InitStructure.SPI_CRCPolynomial = 7;
   
      /* SPI Peripheral Enable */
   SPI_Cmd(SPI1, ENABLE);
   /* Apply SPI configuration after enabling it */
   SPI_Init(SPI1, &SPI_InitStructure);


}


/************************************************************************

* 

* Name:         SPI_Write

* Dependencies: stm32f10x_SPI.c 

* Return:       status

* Description:  This code is executed  to return SPI status

************************************************************************/
uint8_t SPI_Write(uint8_t *buffer, uint8_t nBytes)
{
  uint8_t i;
  for(i=0;i<nBytes;i++)
     {
       TIMEOUT = LONG_TIMEOUT;
       while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)  //not empty
             {
                if((TIMEOUT--) == 0) return (1);
             }
       SPI_I2S_SendData(SPI1,buffer[i]);

       TIMEOUT = LONG_TIMEOUT;
       while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
            {
               if((TIMEOUT--) == 0) return (1);
            }
       SPI_I2S_ReceiveData(SPI1);
      // SPI_I2S_ReceiveData(SPI1);
       
     }
  return(0);
  
}



/************************************************************************

* 

* Name:         SPI_Read

* Dependencies: stm32f10x_SPI.c 

* Return:       status

* Description:  This code is executed  to read data through SPI

************************************************************************/
uint8_t SPI_Read(uint8_t *buffer, uint8_t nBytes)
{
  uint8_t i;
  
  SPI_I2S_ReceiveData(SPI1);
  for(i=0;i<nBytes;i++)
     {
       TIMEOUT = LONG_TIMEOUT;
       while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)  //not empty
             {
                if((TIMEOUT--) == 0) return (1);
             }
       SPI_I2S_SendData(SPI1,0x00);
       TIMEOUT = LONG_TIMEOUT;
       while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
            {
               if((TIMEOUT--) == 0) return (1);
            }
       buffer[i]= SPI_I2S_ReceiveData(SPI1);
     }
  return(0);
}

/************************************************************************

SPI_Write_Byte used to write a byte through SPI
************************************************************************/

uint8_t SPI_WriteByte(uint8_t data)
{
       TIMEOUT = LONG_TIMEOUT;
       while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)  //not empty
             {
                if((TIMEOUT--) == 0) return (1);
             }
       SPI_I2S_SendData(SPI1,data);
       TIMEOUT = LONG_TIMEOUT;
       while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
            {
               if((TIMEOUT--) == 0) return (1);
            }
       SPI_I2S_ReceiveData(SPI1);
       SPI_I2S_ReceiveData(SPI1);
    
       return(0);
}

/************************************************************************

SPI_ReadByte used to read a byte through SPI
************************************************************************/

uint8_t SPI_ReadByte(void)
{
      SPI_I2S_ReceiveData(SPI1);
       while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);  //not empty
            
       SPI_I2S_SendData(SPI1,0xff);
      
       while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
           
       return (SPI_I2S_ReceiveData(SPI1));
}

/************************************************************************

GPIO configuration, this can be modify and adds other function according to 

application

************************************************************************/

void Set_DRDY_Pin_INPUT(void)
{
   GPIO_InitTypeDef  GPIO_InitStructure;
   
   RCC_APB2PeriphClockCmd(DRDY_GPIO_CLK, ENABLE); 
   GPIO_InitStructure.GPIO_Pin = DRDY_PIN;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_Init(GPIOB, &GPIO_InitStructure);      
}

uint8_t DRDY_Pin_Value(void)
{
   return(GPIO_ReadOutputDataBit(GPIOB, DRDY_PIN));      
}
/************************************************************************

* 

* Name:         Enable_Exti

* Dependencies:  

* Return:       no

* Description:  This code is executed  to  enable external interrupt
* In this configuration, PC0 is used at interrupt input pin, falling edge triggers the interrupt, 

************************************************************************/
void Enable_Exti(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  
   //enable GPIOC and AFIO clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO,ENABLE);       //change this for other port
  
  
 //inital GPIO
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;               //change this for other pin

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;

  GPIO_Init(GPIOC, &GPIO_InitStructure);                  //change this for other port
  
  // configure GPIO as interrput
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource0);  //change this for other port and other pin
  
  

  EXTI_InitStructure.EXTI_Line = EXTI_Line0;

  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;

  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // falling edge trigger interrupt

  EXTI_InitStructure.EXTI_LineCmd = ENABLE;

  EXTI_Init(&EXTI_InitStructure);
  
  /* Configure one bit for preemption priority */
  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;                        //change this for other pin

  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;

  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;

  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);
}


#ifdef USE_DEFAULT_CRITICAL_CALLBACK
void EnterCriticalSection_UserCallback(void)
 {
   __disable_irq();  
 }
void ExitCriticalSection_UserCallback(void)
 {
   __enable_irq();
 }
#endif
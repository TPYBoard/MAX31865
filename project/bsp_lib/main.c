/** \file main.c ******************************************************
*
* Project: MAX31865
* Filename: main.c
* Description: This module contains the Main application for the MAX31865 example program.
*

*
* --------------------------------------------------------------------
*
* This code follows the following naming conventions:
*
*\n	char				ch_pmod_value
*\n	char (array)			s_pmod_string[16]
*\n	float				f_pmod_value
*\n	int				n_pmod_value
*\n	int (array)                     an_pmod_value[16]
*\n     int16_t			        w_pmod_value
*\n     int16_t (array)		        aw_pmod_value[16]
*\n	uint16_t		        uw_pmod_value
*\n	uint16_t (array)	        auw_pmod_value[16]
*\n	uint8_t			        uch_pmod_value
*\n	uint8_t (array)		        auch_pmod_buffer[16]
*\n	unsigned int			un_pmod_value
*\n	int *			        pn_pmod_value
*
* ------------------------------------------------------------------------- */

/*!\mainpage Main Page

*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32_spi.h"
#include "MAX31865drv.h"
#include "stm32_usart1.h"
#include <stdio.h>
#include "math.h"



char s[64];
uint8_t uch_fault_status;
uint8_t auch_rtd[2];
uint16_t w_rtd_value;
float   f_temperature;
float   f_resistance;
void Delay(uint32_t ms);

max31865_configuration  configuration;
int main(void)
{
  RCC_DeInit();
  RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
  SPI_Inital();           //初始化SPI
  USART1_Init();           //初始化串口1
   
  Set_DRDY_Pin_INPUT();    //将MAX31865 DRDY对应引脚设置为输入
  
/*配置MAX31865工作模式*/  
  
  configuration.Vbias=ON;
  
  configuration.Rtd_wire=RTD_2wire;
  configuration.Filter=Filter_50Hz;
  maxim_31865_init(&configuration);
  
  /*设置故障门限，输入为电阻值*/
  
  maxim_set_fault_threshold(400,0);
  
  while(1)
  {
  
  /*运行故障检测*/
  
  uch_fault_status=maxim_manual_fault_detection();
  /*
   *如果检测到RTD故障，则输入故障代码，停止程序运行，否则，将进入循环程序
  *在循环程序中，采用单次测量方式，每秒测量一次，将测量结果转换成温度输出。
  *如果在循环程序中检测到RTD故障，同样输入故障代码，但是，会循环检测，一旦
  *故障消除，将会继续进行温度采集
  */
  if((uch_fault_status&0xFC)==0)
     {
         Delay(1000);                                                   //延迟程序 1s
         configuration.Conversion_mode=One_Shot_Conversion;
         maxim_31865_init(&configuration);                              //启动一次测量
         while(DRDY_Pin_Value()==SET);                                  //等待测量结束
         maxim_get_rtd_value(auch_rtd);                                 //读取测量结果
         if((auch_rtd[1]&0x01)==0x01)                                   //如果故障标识置位，运行故障检测，输入故障代码
            {
             uch_fault_status=maxim_manual_fault_detection();           //运行手动故障检测，也可以运行自动故障检测
             USART1_SendString("\033[2J");
             USART1_SendString("MAX31865 Error Code:      ");           //输出故障代码
             sprintf(s,"%x",uch_fault_status&0xFC);
             USART1_SendString(s);
             USART1_SendString("\r\n");
             maxim_clear_fault_status();                                //清除故障寄存器
            }
         else
            {                                                           //如果没有故障，则进行温度转换，输出温度转换结果
             w_rtd_value=(auch_rtd[0]<<8|auch_rtd[1])>>1;               //构造RTD电阻值数据
             f_resistance=w_rtd_value*REF_RES/32768.000;
             f_temperature=(0-R0*A+sqrt(pow(R0,2)*pow(A,2)-4*R0*B*(R0-f_resistance)))/(2*R0*B); //将电阻值转换成温度（该公式仅适用于T>0时）
             sprintf(s,"%.2f",f_temperature);						//设置计算出来的温度值，并将得到的温度值赋值给变量S。
             USART1_SendString("\033[2J");
             USART1_SendString("Current Temperature:   ");              //输出温度转换结果
             USART1_SendString(s);													//串口打印出计算出来的温度值
             USART1_SendString(" ℃\r\n");
            }
         
      }
  else
    {
     USART1_SendString("MAX31865 Error Code:         ");
     sprintf(s,"%x",uch_fault_status&0xFC);
     USART1_SendString(s);
     USART1_SendString("\r\n"); 
     maxim_clear_fault_status();
    }
  }
}

//延迟1ms，该程序和选择的CPU时钟有关，可根据时钟进行调整
void Delay(uint32_t ms)
{
  uint32_t temp;
  //temp=0x000640;
  temp=0x0007D0;
  while(ms)
  {
    while (temp!=0)
        temp--;
    temp=0x0007D0;
    ms--;
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */




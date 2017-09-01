/*******************************************************************************
** 文件名: 		mian.c
** 版本：  		2.0
** 工作环境: 	RealView MDK-ARM 4.1.0
** 作者: 		jeansonm
** 生成日期: 	2013-07-25
** 功能:		SPI 读写 AT45DB161 FLASH
** 相关文件:	无
** 修改日志：	2013-07-25   创建文档
*******************************************************************************/

/******************************
** 红牛开发板(V2.0)
** SPI 读写 AT45DB161 FLASH
** 论坛：bbs.openmcu.com
** 旺宝：www.openmcu.com
** 邮箱：support@openmcu.com
*******************************/

#include "stm32f10x.h"
#include "sys_config.h"
#include <stdio.h>
#include "sys_config.h"
#include "Gpio_Led.h"
#include "spi_flash.h"

typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/* Private define ------------------------------------------------------------*/
#define  FLASH_WriteAddress     0x000000
#define  FLASH_ReadAddress      FLASH_WriteAddress
#define  FLASH_SectorToErase    FLASH_WriteAddress
#define  AT45DB161D_FLASH_ID    0x1F260000
#define  BufferSize (countof(Tx_Buffer)-1)
#define countof(a) (sizeof(a) / sizeof(*(a)))

u8 Tx_Buffer[] = "STM32F10x SPI Firmware Library Example: communication with an AT45DB161D SPI FLASH";//缓冲
u8 Index, Rx_Buffer[BufferSize];
vu32 flash_ID = 0;
volatile TestStatus TransferStatus1 = FAILED, TransferStatus2 = PASSED;

TestStatus Buffercmp(u8* pBuffer1, u8* pBuffer2, u16 BufferLength);


/*******************************************************************************
  * @函数名称	main
  * @函数说明   主函数 
  * @输入参数   无
  * @输出参数   无
  * @返回参数   无
  *****************************************************************************/
int main(void)
{
  RCC_Configuration();
	NVIC_Configuration();
	GPIO_Configuration();
	GpioLed_Init();
  SPI_FLASH_Init();

	flash_ID = SPI_FLASH_ReadID();      //读ID
	
	if(flash_ID == AT45DB161D_FLASH_ID)	//检查ID是否匹配
	{
	    GPIO_ResetBits(GPIO_LED, LED1); //ID正确，打开LED1
	}
	else
	{
	    GPIO_SetBits(GPIO_LED, LED1);   //ID错误，关闭LED1
	}

	/* 读写测试 */
	SPI_FLASH_PageErase(FLASH_SectorToErase);  //页擦出
	SPI_FLASH_BufferWrite(Tx_Buffer, FLASH_WriteAddress, BufferSize); //写入数据
	SPI_FLASH_BufferRead(Rx_Buffer, FLASH_ReadAddress, BufferSize);   //读出数据

	/* 检查写入与读出的数据是否相同 */
	TransferStatus1 = Buffercmp(Tx_Buffer, Rx_Buffer, BufferSize);    //比较读出与写入的数据

	if(TransferStatus1 == PASSED)
	{
	    GPIO_ResetBits(GPIO_LED, LED2); //写入与读出的数据相同 LED2亮
	}
	else
	{
	    GPIO_SetBits(GPIO_LED, LED2);   //写入与读出的数据相同 LED2灭
	}	    

	/* 擦出并读数据测试 */
	SPI_FLASH_PageErase(FLASH_SectorToErase);   //页擦出
	SPI_FLASH_BufferRead(Rx_Buffer, FLASH_ReadAddress, BufferSize);	//读擦出后的数据
	
	for (Index = 0; Index < BufferSize; Index++)  //检查数据是否不是0XFF
	{
		if (Rx_Buffer[Index] != 0xFF)
		{
			TransferStatus2 = FAILED;
		}
	}

	if(TransferStatus2 == PASSED)
	{
	    GPIO_ResetBits(GPIO_LED, LED3); //擦出后，读出数据全为0XFF 擦除正确 LED3亮
	}
	else
	{
	    GPIO_SetBits(GPIO_LED, LED3);   //擦出后，读出数据不全为0XFF 擦除错误 LED3灭
	}
    while (1)
    {
    }
}

/*******************************************************************************
* Function Name  : Buffercmp
* Description    : Compares two buffers.
* Input          : - pBuffer1, pBuffer2: buffers to be compared.
*                : - BufferLength: buffer's length
* Output         : None
* Return         : PASSED: pBuffer1 identical to pBuffer2
*                  FAILED: pBuffer1 differs from pBuffer2
*******************************************************************************/
TestStatus Buffercmp(u8* pBuffer1, u8* pBuffer2, u16 BufferLength)
{
  while (BufferLength--)
  {
    if (*pBuffer1 != *pBuffer2)
    {
      return FAILED;
    }

    pBuffer1++;
    pBuffer2++;
  }

  return PASSED;
}

#ifdef  USE_FULL_ASSERT
/*******************************************************************************
  * @函数名称	assert_failed
  * @函数说明   报告在检查参数发生错误时的源文件名和错误行数
  * @输入参数   file: 源文件名
  				line: 错误所在行数 
  * @输出参数   无
  * @返回参数   无
  *****************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
    while (1)
    {
    }
}
#endif

/***********************************文件结束***********************************/

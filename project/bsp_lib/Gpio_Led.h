#ifndef _GPIO_LED_H_
#define _GPIO_LED_H_ 

/*****led1½Ó¿ÚÉùÃ÷*****/
#define RCC_APB2Periph_GPIO_LED  RCC_APB2Periph_GPIOF
#define GPIO_LED            GPIOF
#define LED1				GPIO_Pin_6
#define LED2				GPIO_Pin_7
#define LED3				GPIO_Pin_8
#define LED4				GPIO_Pin_9
#define LED5				GPIO_Pin_10	  

void GpioLed_Init(void);
void LED_Display(void);

#endif

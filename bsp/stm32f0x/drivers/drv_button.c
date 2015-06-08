/* Includes ------------------------------------------------------------------*/	
#include "stm32f0xx.h"
#include "drv_button.h"

void Delay(__IO uint32_t nCount)
{
	for(;nCount != 0; nCount--);
}

void BUTTON_Init(void)
{
	/*定义一个GPIO_InitTypeDef类型的结构体*/
  GPIO_InitTypeDef GPIO_InitStructure;
   
  /* Enable the GPIO  Clock 开启GPIOA的外设时钟*/
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	
  /* Configure USART Tx Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;										//GPIOA0
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;								//普通输入模式
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;							//下拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);											//初始化PA0
	
}


uint8_t BUTTON_Scan(void)
{	 
//	static uint8_t key = 0;
	
	if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == BUTTON_ON)
	{
		/* 延时消抖*/
		Delay(10000);
		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == BUTTON_ON)
		{
			return BUTTON_ON;
		}
	}
	
	return BUTTON_OFF;// 无按键按下
}

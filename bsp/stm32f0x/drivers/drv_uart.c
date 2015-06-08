/* Includes ------------------------------------------------------------------*/	
#include "stm32f0xx.h"
#include "drv_uart.h"



//加入以下代码,支持printf函数,而不需要选择use MicroLIB
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 
//	while((USART1->ISR&0X40)==0);  位7 TXE:发送数据寄存器
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);//循环发送,直到发送完毕     
	USART1->TDR = (ch & (uint16_t)0x01FF);  
	
	return ch;
}
#endif 

/*使用microLib的方法*/
 /* 
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}	
   
    return ch;
}
int GetKey (void)  { 

    while (!(USART1->SR & USART_FLAG_RXNE));

    return ((int)(USART1->DR & 0x1FF));
}
*/


void UART_Init(void)
{
	/*定义一个GPIO_InitTypeDef类型的结构体*/
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
   
  /* Enable the GPIO  Clock 开启GPIOD的外设时钟*/
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	
	/* Enable USART clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	/* Connect PA9 to USART1_Tx */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);

  /* Connect PA10 to USART1_Rx */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);
	
  /* Configure USART Tx Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;			//GPIOA9与GPIOA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;								//复用功能
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;						//速度50M
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;							//推挽复用输出
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;								//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);											//初始化PA9，PA10
	
	
	/* USARTx configured as follow:
  - BaudRate = 115200 baud  
  - Word Length = 8 Bits
  - Stop Bit = 1 Stop Bit
  - Parity = No Parity
  - Hardware flow control disabled (RTS and CTS signals)
  - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;								
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	/* USART configuration */
  USART_Init(USART1, &USART_InitStructure);
    
  /* Enable USART */
  USART_Cmd(USART1, ENABLE);

	
}

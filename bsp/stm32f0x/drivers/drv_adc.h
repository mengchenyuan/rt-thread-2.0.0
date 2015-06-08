/**@file drv_adc.h
  ******************************************************************************
  */
	
#ifndef DRV_ADC_H_
#define DRV_ADC_H_

#include <rtthread.h>
#include "stm32f0xx.h"


//void ADC1_Init(void);
//void ADC2_Init(void);
//void ADC3_Init(void);
//void ADC4_Init(void);
//void ADC5_Init(void);
//void ADC6_Init(void);
void ADC_DMA_Config(void);
void Read_ADC(void);
void USART_Put_Char(unsigned char ch);
//void USART_Config(void);
//void rt_hw_adc_init(void);

#endif // DRV_ADC_H_

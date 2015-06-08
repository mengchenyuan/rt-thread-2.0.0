/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2013-11-15     bright       add init thread and components initial
 */

/**
 * @addtogroup STM32
 */
/*@{*/

#include <stdio.h>
#include <stdlib.h>

#include <board.h>
#include <rtthread.h>
#ifdef  RT_USING_COMPONENTS_INIT
#include <components.h>
#endif  /* RT_USING_COMPONENTS_INIT */

#include "stm32f0xx.h"

#include "drv_led.h"
#include "drv_encoder.h"
#include "usart.h"
#include "drv_adc.h"
#include "drv_timer.h"
#include "drv_button.h"
//#include "drv_adc.h"


int8_t  encoder_thread_entry(void);
volatile int8_t encoder_num;
extern __IO uint16_t  uhADC1ConvertedValue;
extern __IO uint32_t  uwADC1ConvertedVoltage; 


/* led thread entry */
static void led_thread_entry(void* parameter)
{	
	while(1) 
	{
        rt_hw_led1_on();
        rt_thread_delay(RT_TICK_PER_SECOND/5);
				rt_kprintf("\r\n this is a LD_U printf demo \r\n");

        rt_hw_led1_off();
        rt_thread_delay(RT_TICK_PER_SECOND/5);
		
		    rt_hw_led3_on();
        rt_thread_delay(RT_TICK_PER_SECOND/5);
				rt_kprintf("\r\n this is a LD_L printf demo \r\n");

        rt_hw_led3_off();
        rt_thread_delay(RT_TICK_PER_SECOND/5);
		
		    rt_hw_led2_on();
        rt_thread_delay(RT_TICK_PER_SECOND/5);
				rt_kprintf("\r\n this is a LD_D printf demo \r\n");

        rt_hw_led2_off();
        rt_thread_delay(RT_TICK_PER_SECOND/5);
		
		    rt_hw_led4_on();
        rt_thread_delay(RT_TICK_PER_SECOND/5);
				rt_kprintf("\r\n this is a LD_R printf demo \r\n");

        rt_hw_led4_off();
        rt_thread_delay(RT_TICK_PER_SECOND/5);
	}
}

int8_t encoder_thread_entry(void)
{
//  static  uint16_t  last_count = 0;
//  uint16_t  cur_count = TIM3->CNT;
//  int32_t dAngle = cur_count - last_count;
//	rt_kprintf("encoder_num:%d \r\n",encoder_num);	
//  if(dAngle >= 50){
//    dAngle -= 100;
//  }else if(dAngle < -50){
//    dAngle += 100;
//  }
//  last_count = cur_count;
//	return (int8_t)dAngle;
	
	 int16_t*degree;
	 Get_Angle(degree);
	rt_kprintf("\r\n this is encoder demo : %d%d%d \r\n",degree[0],degree[1],degree[2]);
}

/* adc thread entry */
static void adc_thread_entry(void* parameter)
{
//    rt_int32_t adc;  
		rt_kprintf("hello here is a ADC test\n\r");  
 //   ADC_SoftwareStartConvCmd(ADC1,ENABLE);
 //   while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)==RESET);  //检查制定ADC标志位置1与否 ADC_FLAG_EOC转换结束标志位
//    adc = ADC_GetConversionValue(ADC1);  
//    rt_kprintf("ADC: 0X%xV\n\r", adc);  
//    rt_kprintf("ADC: 0D%dV\n\r", adc);  
//    rt_kprintf("ADC: 0X%xV\n\r", adc*3230);  
//    rt_kprintf("ADC: 0D%dV\n\r", adc*3230);  
//    rt_kprintf("ADC: 0X%xV\n\r", adc*3230/4095);  
//    rt_kprintf("ADC: 0D%dV\n\r", adc*3230/4095);  
//    rt_thread_delay( RT_TICK_PER_SECOND/2 ); 

		ADC_DMA_Config();
		rt_hw_usart_init();

    while(1)
  {     
        rt_thread_delay(RT_TICK_PER_SECOND);
        Read_ADC(); 
        USART_Put_Char(0xD);
        USART_Put_Char(0xA);
  }  
}

static void rt_init_thread_entry(void* parameter)
{
	rt_thread_t led_thread;
	rt_thread_t encoder_thread;
	rt_thread_t adc_thread;

/* Initialization RT-Thread Components */
#ifdef RT_USING_COMPONENTS_INIT
    rt_components_init();
#endif

/* Set finsh device */
#ifdef  RT_USING_FINSH
    finsh_set_device(RT_CONSOLE_DEVICE_NAME);
#endif  /* RT_USING_FINSH */

    /* Create led thread */
    led_thread = rt_thread_create("led",
    		led_thread_entry, RT_NULL,
    		256, 18, 20);
	    if(led_thread != RT_NULL)
    	rt_thread_startup(led_thread);
					
	  encoder_thread = rt_thread_create("encoder",
    		encoder_thread_entry, RT_NULL,
    		256, 20, 20);				
		if(encoder_thread != RT_NULL)
    	rt_thread_startup(encoder_thread);	
		
		 /* Create adc thread */
    adc_thread = rt_thread_create("adc",
    		adc_thread_entry, RT_NULL,
    		256, 19, 20);			
		if(adc_thread != RT_NULL)
    	rt_thread_startup(adc_thread);
}

int rt_application_init()
{

	rt_thread_t init_thread;
	
	rt_hw_usart_init();

#if (RT_THREAD_PRIORITY_MAX == 32)
		init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   512, 8, 20);
		init_thread = rt_thread_create("led",
                                   led_thread_entry, RT_NULL,
                                   256, 18, 20);
		init_thread = rt_thread_create("adc",
                                   adc_thread_entry, RT_NULL,
                                   256, 19, 20);
    
#else
		  init_thread = rt_thread_create("encoder",
                                   encoder_thread_entry, RT_NULL,
                                   512, 8, 20);
#endif
    if(init_thread != RT_NULL)
    	rt_thread_startup(init_thread);

    return 0;

}
/*@}*/

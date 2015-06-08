/*
 * File      : drv_led.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 */

#ifndef __DRV_LED_H__
#define __DRV_LED_H__

#include <rthw.h>
#include <rtthread.h>
#include <stm32f0xx.h>

#define rt_hw_led1_on()   GPIO_SetBits(GPIOC, GPIO_Pin_9)
#define rt_hw_led1_off()  GPIO_ResetBits(GPIOC, GPIO_Pin_9)

#define rt_hw_led2_on()   GPIO_SetBits(GPIOC, GPIO_Pin_7)
#define rt_hw_led2_off()  GPIO_ResetBits(GPIOC, GPIO_Pin_7)

#define rt_hw_led3_on()   GPIO_SetBits(GPIOC, GPIO_Pin_8)
#define rt_hw_led3_off()  GPIO_ResetBits(GPIOC, GPIO_Pin_8)

#define rt_hw_led4_on()   GPIO_SetBits(GPIOC, GPIO_Pin_6)
#define rt_hw_led4_off()  GPIO_ResetBits(GPIOC, GPIO_Pin_6)

void rt_hw_led_init(void);

#endif

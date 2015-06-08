#include "drv_led.h"
/* RT_USING_COMPONENTS_INIT */
#ifdef  RT_USING_COMPONENTS_INIT
#include <components.h>
#endif

/*
LED_GREEN: PC6
LED_RED  : PC7
LED_GREEN: PC8
LED_RED  : PC9
*/

/* Initial led gpio pin  */
void rt_hw_led_init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    /* Enable the GPIO_LED Clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

    /* Configure the GPIO_LED pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/* Initial components for device */
INIT_DEVICE_EXPORT(rt_hw_led_init);

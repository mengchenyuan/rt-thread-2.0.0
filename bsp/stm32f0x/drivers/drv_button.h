	
#ifndef DRV_BUTTON_H_
#define DRV_BUTTON_H_


#include "stm32f0xx.h"

/*默认低0，断开  高1 按下*/
#define BUTTON_ON  1
#define BUTTON_OFF 0

void     BUTTON_Init(void);//IO初始化
uint8_t  BUTTON_Scan(void);  //按键扫描函数	

#endif // DRV_BUTTON_H_

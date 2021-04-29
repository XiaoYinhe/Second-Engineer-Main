#ifndef __SWITCH_H__
#define __SWITCH_H__

#include "board.h"
/***************SWITCH ¿‡∂®“Â******************/



class Switch{
private:
		
	uint32_t RCC_AHB1Periph_GPIOx;	/*RCC*/
	GPIO_TypeDef * GPIOx;	/*GPIO*/
	uint16_t GPIO_Pin_x;		/*Pin*/
public:

	Switch(uint32_t RCC_AHB1Periph_GPIOx,GPIO_TypeDef * GPIOx,uint16_t GPIO_Pin_x);
	void init(void);


};


	
#endif

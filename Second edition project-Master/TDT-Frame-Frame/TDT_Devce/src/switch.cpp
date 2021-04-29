#include "switch.h"

Switch::Switch(uint32_t RCC_AHB1Periph_GPIOx,GPIO_TypeDef * GPIOx,uint16_t GPIO_Pin_x)
{
	this->RCC_AHB1Periph_GPIOx=RCC_AHB1Periph_GPIOx;
	this->GPIOx=GPIOx;
	this->GPIO_Pin_x=GPIO_Pin_x;
}

void Switch::init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOx , ENABLE );
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_x;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOx,&GPIO_InitStructure);
	GPIOx->BSRRL = GPIO_Pin_x;
}
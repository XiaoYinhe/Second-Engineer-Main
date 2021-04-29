/*****************************************************************************
File name: TDT_Bsp\src\can.cpp
Description: can底层
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.16 #合成can1.c和can2.c #修改接收函数，从队列传递改为直接解算
	——————————————————————————————————————————————————————————————————————————
	19.11.12 #首次完成
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef __CAN1_H__
#define __CAN1_H__

#include "board.h"

void canInit(CAN_TypeDef * can_x);
//void TDT_Can_Tx(vec4f* value,CAN_TypeDef * can_x,uint32_t id);
void canTx(float *data,CAN_TypeDef * can_x,uint32_t id);
	
#ifdef __cplusplus
extern "C"{
#endif	/*__cplusplus*/	

void CAN1_TX_IRQHandler(void);
void CAN2_TX_IRQHandler(void);
void CAN1_RX0_IRQHandler(void);
void CAN2_RX0_IRQHandler(void);

#ifdef __cplusplus
}
#endif	/*__cplusplus*/
#endif

#ifndef __ELEVATOR_TASK_H__
#define __ELEVATOR_TASK_H__

#include "board.h"

void Travel_switch_init(void);
void rescue(void);
void Help_without_holding(void);

typedef struct{
	u8 reliveAir;
	u8 knifeAir;
	


}_AirCmd;

extern _AirCmd AirCmd;




#endif
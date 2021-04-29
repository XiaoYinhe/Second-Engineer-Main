#ifndef __ELEVATOR_TASK_H__
#define __ELEVATOR_TASK_H__

#include "board.h"

#define Exchange_position  240000
#define Silver_position  300000
#define caught_falling_position 350000
typedef struct{
	u8 Elevator_position = 0;
	u8 QuadraticCmd = 0;
	



}_FlagCmd;

/*计数和计时*/
typedef struct{
	int Uplift_times;
	int Uplift_times_last;

}_count;



void TDT_Uplift_Three(void);

extern _FlagCmd FlagCmd;
extern _count Count;


#endif
/**************
@brief 升降机控制函数
@describe	1、抬升两个电机，3508
			2、通过串口与副控通信，实现完整动作
			3、将升降机写为标志为控制,其中bottom,exchange,silver,caught分别为0，600，700,1000高度
@function
TDT_Uplift_Three();三段抬升标志位控制，Count.Uplift_times = 0，1，2，3即可






****************/


#include "elevator.h"

/**FreeRTOS*START***************/
#include "FreeRTOS.h" //FreeRTOS使用
#include "timers.h"
#include "list.h"
#include "queue.h"
#include "task.h"
/**FreeRTOS*END***************/
#include "motor.h"
#include "can.h"
#include "dbus.h"

_FlagCmd FlagCmd;
_count Count;
enum Elevator_sta{bottom,exchange,silver,caught};

Motor Elevator[2] =
{
   Motor (M3508,CAN2,0x201),
   Motor (M3508,CAN2,0x202)

};
PidParam in_elevator[2],out_elevator[2];
void TDT_Uplift_Three()
{
	/*抬升的两个电机0x201上升时顺时针，下降时逆时针，顺时针为正，0x202相反*/
		if(Count.Uplift_times == 1 && Count.Uplift_times_last == 0)   //上升到1
		{
			FlagCmd.QuadraticCmd = 1;
		}
		else if((Count.Uplift_times == 1 && Count.Uplift_times_last == 2) || (Count.Uplift_times == 1 && Count.Uplift_times_last == 3))
		{
			FlagCmd.QuadraticCmd = 0;																		//下降到1
		}
		else if((Count.Uplift_times == 2 && Count.Uplift_times_last == 1) || (Count.Uplift_times == 2 && Count.Uplift_times_last == 0))//上升到2
		{
			FlagCmd.QuadraticCmd = 2;
		}
		else if(Count.Uplift_times == 2 && Count.Uplift_times_last == 3) //下降到2
		{
			FlagCmd.QuadraticCmd = 3;
		}
		
		
	switch(Count.Uplift_times)																						
	{
		case bottom:			
			out_elevator[0].resultMax = Elevator[0].getMotorSpeedLimit()/7;		
			out_elevator[1].resultMax = Elevator[0].getMotorSpeedLimit()/7;
			Elevator[0].ctrlPosition(0);
			Elevator[1].ctrlPosition(0);			
			break;
		case exchange:																			
			if( FlagCmd.QuadraticCmd == 1)					//上升到500
			{
				out_elevator[0].resultMax = Elevator[0].getMotorSpeedLimit()/5;
				out_elevator[1].resultMax = Elevator[0].getMotorSpeedLimit()/5;
				Elevator[0].ctrlPosition(Exchange_position);
				Elevator[1].ctrlPosition(-Exchange_position);			
			}
			if(FlagCmd.QuadraticCmd == 0)					//下降到500
			{
				out_elevator[0].resultMax = Elevator[0].getMotorSpeedLimit()/7;
				out_elevator[1].resultMax = Elevator[0].getMotorSpeedLimit()/7;
				Elevator[0].ctrlPosition(Exchange_position);			
				Elevator[1].ctrlPosition(-Exchange_position);
			}
			break;
		case silver:
			if(FlagCmd.QuadraticCmd == 2)					//上升到600
			{
				out_elevator[0].resultMax = Elevator[0] .getMotorSpeedLimit()/5;
				out_elevator[1].resultMax = Elevator[0] .getMotorSpeedLimit()/5;
				Elevator[0].ctrlPosition(Silver_position);
				Elevator[1].ctrlPosition(-Silver_position);
			}
			if(FlagCmd.QuadraticCmd == 3)					//下降
			{
				out_elevator[0].resultMax = Elevator[0].getMotorSpeedLimit()/7;
				out_elevator[1].resultMax = Elevator[0].getMotorSpeedLimit()/7;
				Elevator[0].ctrlPosition(Silver_position);			
				Elevator[1].ctrlPosition(-Silver_position);		
			}
			break;
		case caught://上升到700
			
			out_elevator[0].resultMax = Elevator[0].getMotorSpeedLimit()/5;
			out_elevator[1].resultMax = Elevator[0].getMotorSpeedLimit()/5;
			Elevator[0].ctrlPosition(caught_falling_position);
			Elevator[1].ctrlPosition(-caught_falling_position);
			
			break;
		default:
			break;				
	}

			Count.Uplift_times_last = Count.Uplift_times;
		
			vTaskDelay(pdMS_TO_TICKS(5));
}


void Elevator_Task(void *pvParameters)
{
	

	for(int i = 0;i<2;i++)
	{
	
		Elevator[i].pidInner.setPlanNum(2);
		Elevator[i].pidOuter.setPlanNum(2);
	
		in_elevator[i].kp = 0;
		in_elevator[i].ki = 0;
		in_elevator[i].kd = 0;
		in_elevator[i].resultMax = Elevator[i].getMotorCurrentLimit();
		
		out_elevator[i].kp = 0;
		out_elevator[i].ki = 0;
		out_elevator[i].kd = 0;	
		
		
		Elevator[i].pidInner.paramPtr = &in_elevator[i];
		Elevator[i].pidOuter.paramPtr = &out_elevator[i];
		
		Elevator[i].pidInner.fbValuePtr[0] = &Elevator[i].canInfo.speed;
		Elevator[i].pidOuter.fbValuePtr[0] = &Elevator[i].canInfo.totalEncoder;
		
		
	}

	while(1)
	{
		TDT_Uplift_Three();
		
		
		vTaskDelay(pdMS_TO_TICKS(5));
	}
	
	


}
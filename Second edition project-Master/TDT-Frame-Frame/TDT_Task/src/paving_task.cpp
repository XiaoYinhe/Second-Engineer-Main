/************
@brief 铺路任务函数
@describe	1、此函数与视觉联调插障碍块实现铺路
			2、按键控制
@function	可以直接使用AirCmd	
		
		
**************/

#include "paving_task.h"
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
#include "vision.h"
#include "rescue_task.h"

#include "task_virtual.h"


Motor Pave[2]
{
	Motor (M3508,CAN2,0x207),
    Motor (M3508,CAN2,0x208)
};


extern void loadPara(PidParam *parm,float kp,float ki,float kd,float intMax, float outMax);
enum Paving_sta{anastole,insert};
u8 flag_pave;


class PaveTask : public VirtualTask
{
public:
	u8 offsetFlag=0;
	void deforceCancelCallBack() override
	{
		
		while(1)
		{
			offsetFlag = Pave[0].ctrlMotorOffset(-1,2000,7000);
			Pave[1].ctrlCurrent(-Pave[0].pidInner.result);
			
			if(offsetFlag >0)
			{
				VirtualTask::deforceCancelCallBack();
				break;
			}
			vTaskDelay(pdMS_TO_TICKS(5));
		}
	}
};


PaveTask paveTask;



void Paving_Task(void *pvParameters)
{
	paveTask.setTaskHandler(NULL);
	PidParam in_pave,out_pave;
		
	Pave[0].pidInner.setPlanNum(2);
	
	loadPara(&in_pave,0,0,0,1000,Pave[0].getMotorCurrentLimit());
		 
	Pave[0].pidInner.paramPtr = &in_pave;
	Pave[0].pidOuter.paramPtr = &out_pave;
		 
	Pave[0].pidInner.fbValuePtr[0] = &Pave[0].canInfo.speed;
	Pave[0].pidOuter.fbValuePtr[0] = &Pave[0].canInfo.totalEncoder;
	while(1)
	{
		switch(flag_pave)
		{
			case anastole:	//收回
				Pave[0].ctrlPosition(0);
				Pave[1].ctrlCurrent(-Pave[0].pidInner.result);
				break;
			case insert:	//放下
				Pave[0].ctrlPosition(-60000);
				Pave[1].ctrlCurrent(-Pave[0].pidInner.result);
				break;
		
		}
		
		vTaskDelay(pdMS_TO_TICKS(5));
	}
	
	
	
}
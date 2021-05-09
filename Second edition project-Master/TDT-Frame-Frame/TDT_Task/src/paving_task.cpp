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
class PaveTask : public VirtualTask
{
public:
	u8 offsetFlag=0;
	void deforceCancelCallBack() override
	{
		VirtualTask::deforceCancelCallBack();
		while(1)
		{
			offsetFlag = Pave[0].ctrlMotorOffset(-1,2000,7000);
			Pave[1].ctrlCurrent(-Pave[0].pidInner.result);
			
			if(offsetFlag >0)
				break;
			vTaskDelay(pdMS_TO_TICKS(5));
		}
	}
};


PaveTask paveTask;


enum Paving_sta{anastole,insert};
u8 flag_pave;
void pave_ctrl()
{
	switch(flag_pave)
	{
		case anastole:
			Pave[0].ctrlPosition(0);
			Pave[1].ctrlCurrent(-Pave[0].pidInner.result);
			break;
		case insert:
			Pave[0].ctrlPosition(-60000);
			Pave[1].ctrlCurrent(-Pave[0].pidInner.result);
			break;
	
	}

}

void Paving_Task(void *pvParameters)
{
	paveTask.setTaskHandler(NULL);
	PidParam in_pave[2],out_pave[2];
	
	for(int i = 0;i<2;i++)
    {
    	Pave[i].pidInner.setPlanNum(2);
    	Pave[i].pidOuter.setPlanNum(2);
   
    	in_pave[i].kp = 0;
    	in_pave[i].ki = 0;
    	in_pave[i].kd = 0;
    	in_pave[i].resultMax = Pave[i].getMotorCurrentLimit();
    	
    	out_pave[i].kp = 0;
    	out_pave[i].ki = 0;
    	out_pave[i].kd = 0;	
    	out_pave[i].resultMax = Pave[i].getMotorSpeedLimit();
    	
    	Pave[i].pidInner.paramPtr = &in_pave[i];
    	Pave[i].pidOuter.paramPtr = &out_pave[i];
    	
    	Pave[i].pidInner.fbValuePtr[0] = &Pave[i].canInfo.speed;
    	Pave[i].pidOuter.fbValuePtr[0] = &Pave[i].canInfo.totalEncoder;
    	
    	
    }
	while(1)
	{
	
		pave_ctrl();
		
		vTaskDelay(pdMS_TO_TICKS(5));
	}
	
	
	
}
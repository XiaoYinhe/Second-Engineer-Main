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

#include "curve_model.h"

Motor Pave[2]
{
	Motor (M3508,CAN2,0x207),
    Motor (M3508,CAN2,0x208)
};


extern void loadPara(PidParam *parm,float kp,float ki,float kd,float intMax, float outMax);
enum Paving_sta{anastole,insert};
u8 flag_pave;


//铺路任务类：为了重写虚函数
class PaveTask : public VirtualTask
{
public:
	u8 offsetFlag=0;
	//重写虚函数：添加脱力校准Flag
	void deforceCancelCallBack() override
	{
		offsetFlag  =0;
		//恢复铺路任务
		VirtualTask::deforceCancelCallBack();
	}
};





float paveSetPos=0;
float paveRest=0;
	PidParam in_pave,out_pave;
void Paving_Task(void *pvParameters)
{
	PaveTask paveTask;
	paveTask.setTaskHandler(NULL);
		
	Pave[0].pidInner.setPlanNum(2);
	//加载PID
	loadPara(&in_pave,5,0,0,1000,Pave[0].getMotorCurrentLimit());
	loadPara(&out_pave,0.3,0,0,1000,Pave[0].getMotorSpeedLimit());
	Pave[0].pidInner.paramPtr = &in_pave;
	Pave[0].pidOuter.paramPtr = &out_pave;
		 
	Pave[0].pidInner.fbValuePtr[0] = &Pave[0].canInfo.speed;
	Pave[0].pidOuter.fbValuePtr[0] = &Pave[0].canInfo.totalEncoder;
	
	while(1)
	{
		//电机零点校准校准
		while(paveTask.offsetFlag == 0)
		{
			paveTask.offsetFlag = Pave[0].ctrlMotorOffset(170,8000,6000);
			Pave[1].ctrlCurrent(-Pave[0].pidInner.result);
			
			if(paveTask.offsetFlag >0)
			{
				break;
			}
			vTaskDelay(pdMS_TO_TICKS(5));
		}
		
		switch(flag_pave)
		{
			case anastole:	//收回
				paveSetPos = -800;	
			out_pave.resultMax = 1200;	//限速的同时保证输出不要太小
				break;
			case insert:	//放下
				paveSetPos = -50000;
				out_pave.resultMax = 500;
				break;
		}
		
		
		Pave[0].ctrlPosition(paveSetPos);
		Pave[1].ctrlCurrent(-Pave[0].pidInner.result);//辅助电机不适用位置环
		
		vTaskDelay(pdMS_TO_TICKS(5));
	}
	
	
	
}
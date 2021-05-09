/************
@brief ��·������
@describe	1���˺������Ӿ��������ϰ���ʵ����·
			2����������
@function	����ֱ��ʹ��AirCmd	
		
		
**************/

#include "paving_task.h"
/**FreeRTOS*START***************/
#include "FreeRTOS.h" //FreeRTOSʹ��
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
Motor Pave[2]
{
	Motor (M3508,CAN1,0x205),
    Motor (M3508,CAN1,0x206)
};
enum Paving_sta{anastole,insert};
u8 flag_pave;
void pave_ctrl()
{
	switch(flag_pave)
	{
		case anastole:
			Pave[0].ctrlPosition(0);
			Pave[1].ctrlPosition(0);
			break;
		case insert:
			Pave[0].ctrlPosition(30000);
		    Pave[1].ctrlPosition(-30000);
			break;
	
	}

}

void Paving_Task(void *pvParameters)
{
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
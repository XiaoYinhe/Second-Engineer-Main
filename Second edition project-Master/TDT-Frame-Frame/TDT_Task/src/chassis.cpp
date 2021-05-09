#include "chassis_task.h"

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
#include "dbus_task.h"
#include "filter.h"

Motor MChassis[WHEEL_NUM] =
{
   Motor (M3508,CAN1,0x201),
   Motor (M3508,CAN1,0x202),
	 Motor (M3508,CAN1,0x203),
	 Motor (M3508,CAN1,0x204)
};
Kf KfChassis;

PidParam inner[4],outer[4];

#define MID_YAW_ENCODER   3000

const float FB_MAXSPEED = 16384.0f / 660.0f; /**< 底盘前后的最大速度*/   //16384
const float LR_MAXSPEED = 16384.0f / 660.0f; /**< 底盘左右的最大速度*/   //16384
const float ROTA_MAXSPEED = 16384.0f / 660.0f; /**< 底盘旋转的最大速度*/ //16384

typedef struct{
	float FB;//前后速度
	float LR;//左右速度
	float ROT;//旋转速度
	float ROX;//X轴旋转
}ChassisSpd_t;


int16_t chassisSetSpd[4] = {0};//底盘速度设定值
float chassisCtrlIndex;

void Chassis_Task(void *pvParameters)
{
	ChassisSpd_t ChassisSpd;
	
	for(u8 i = 0;i < WHEEL_NUM ; i++)
  {
		MChassis[i].pidInner.setPlanNum(2);
		MChassis[i].pidOuter.setPlanNum(2);
		
		inner[i].kp = 4;
		inner[i].ki = 0;
	  inner[i].kd = 0;
	  inner[i].resultMax = MChassis[i].getMotorCurrentLimit();
		
		outer[i].kp = 0;
	  outer[i].ki = 0;
	  outer[i].kd = 0;	
  	outer[i].resultMax = MChassis[i].getMotorSpeedLimit();		
		
	  MChassis[i].pidInner.paramPtr = &inner[i];
	  MChassis[i].pidOuter.paramPtr = &outer[i];
	
	  MChassis[i].pidInner.fbValuePtr[0] = &MChassis[i].canInfo.speed;
	  MChassis[i].pidOuter.fbValuePtr[0] = &MChassis[i].canInfo.totalEncoder;
	}
	


	while(1)
	{		
		ChassisSpd.ROX = (KfChassis.KalmanFilter((double) RC.Key.CH[6]*15,0.5f,660.2f,1))*(2 + RC.Key.SHIFT * 0.45f - RC.Key.CTRL * 1.60f);
		ChassisSpd.ROX = LIMIT(ChassisSpd.ROX,-9000,9000);
		chassisCtrlIndex = (0.35f + RC.Key.SHIFT * 0.45f - RC.Key.CTRL * 0.20f) * 1320.0f;
		ChassisSpd.FB = (RC.Key.CH[3] * FB_MAXSPEED + (RC.Key.W - RC.Key.S) * FB_MAXSPEED * chassisCtrlIndex );
		ChassisSpd.LR = (RC.Key.CH[2] * LR_MAXSPEED + (RC.Key.D - RC.Key.A) * LR_MAXSPEED * chassisCtrlIndex );
		ChassisSpd.ROT = (RC.Key.CH[0] * ROTA_MAXSPEED *0.33f + ChassisSpd.ROX);
			
		chassisSetSpd[0] = (int16_t)((+ ChassisSpd.FB + ChassisSpd.LR) * ((!ChassisSpd.ROT) + (!!ChassisSpd.ROT * 0.4)) + ChassisSpd.ROT);
		chassisSetSpd[1] = - (int16_t)((- ChassisSpd.FB + ChassisSpd.LR) * ((!ChassisSpd.ROT) + (!!ChassisSpd.ROT * 0.4)) - ChassisSpd.ROT );
		chassisSetSpd[2] = - (int16_t)((+ ChassisSpd.FB - ChassisSpd.LR) * ((!ChassisSpd.ROT) + (!!ChassisSpd.ROT * 0.4)) - ChassisSpd.ROT);
		chassisSetSpd[3] = (int16_t)((- ChassisSpd.FB - ChassisSpd.LR) * ((!ChassisSpd.ROT) + (!!ChassisSpd.ROT * 0.4)) + ChassisSpd.ROT );
		/*保证四电机合成速度方向*/
		float setMax;//最大速度值 
		setMax = ABS(chassisSetSpd[0]);
		for(u8 i = 0;i < WHEEL_NUM ; i++)
		{
			if(chassisSetSpd[i]>setMax)
				setMax=chassisSetSpd[i];
		    if(setMax>FB_MAXSPEED*660)
				for(u8 i = 0;i < WHEEL_NUM ; i++)
					chassisSetSpd[i]=chassisSetSpd[i]*FB_MAXSPEED*660/setMax;
		}
		
			for(u8 i = 0;i < WHEEL_NUM ; i++)
    {
		MChassis[i].ctrlSpeed(chassisSetSpd[i],0);
  	}	  
	  	vTaskDelay(pdMS_TO_TICKS(5));
	}
}




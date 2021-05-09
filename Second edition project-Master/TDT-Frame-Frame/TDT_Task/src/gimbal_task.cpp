#include "gimbal_task.h"

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
#include "dbus_task.h"
#include "filter.h"
#include "imu.h"
#include "icm20602.h"
#include "filter.h"
#include "curve_model.h"

//VirtualTask gimbalTask;

#include "task_virtual.h"



extern Icm20602 icm20602;
Motor MYaw = Motor(M2006,CAN2,0x202);
//Motor MPitch = Motor(M3510,CAN2,0x208);


#define YAW_PID_NUM 1
#define PITCH_PID_NUM 2

//Yaw 方案1为机械角
PidParam yawInner[YAW_PID_NUM],yawOuter[YAW_PID_NUM];

//Pitch 方案1为机械角 方案2为陀螺仪
//PidParam pitchInner[PITCH_PID_NUM],pitchOuter[PITCH_PID_NUM];



u8 gimbalState=0;	//状态机变量
float yawSetPos=0,yawTarPos;
float pitchSetPos=0;
u8 useImuFlag = 0;	//是否使用陀螺仪标志位，默认使用，同时作为PID参数号
float yawUserOffset =0; //yaw轴手动校准值，用于自动校准失败时手动校准，比如机械限位掉了，就必须使用手动校准
static void loadPara(PidParam *parm,float kp,float ki,float kd,float intMax, float outMax)
{
	parm->kp = kp;
	parm->ki = ki;
	parm->kd = kd;
	parm->integralErrorMax = intMax;
	parm->resultMax = outMax;
}



//yaw校准
u8 yawOffset()
{
	u8 err=0; //校准错误码
	//Yaw轴需要初始校准
	u16 offsetDelayCnt =0;
	float nowOffsetPos=0, startOffsetPos = MYaw.canInfo.totalAngle;
	while(1)
	{
		//校准成功就立马退出
		if(MYaw.ctrlMotorOffset(-100.0f,7000.0f,3000) > 0 )
		{
			err=0;
			break;
		}
		
		//一整圈都没校准成功就判断为失败,直接退出，让操作手手调
		nowOffsetPos = MYaw.canInfo.totalAngle;
		if(ABS(nowOffsetPos - startOffsetPos) > 360)
		{
			err=1;
			break;
		}

		//如果超时5000ms：退出
		if(offsetDelayCnt++ > 5000)
		{
			err=2;
			offsetDelayCnt=0;
			break;
		}
		//延时1ms
		vTaskDelay(pdMS_TO_TICKS(1));
	}
	return err;
}



//PITCH 上极限 3447 下极限 4478 共-45
// 云台Pitch轴随控制
//yaw轴看肚子才转
u8 offsetErrCode=0; //校准错误码
void Gimbal_Task(void *pvParameters)
{
	VirtualTask gimbalTask;
	gimbalTask.setTaskHandler(NULL);
	MYaw.pidInner.setPlanNum(YAW_PID_NUM);
	MYaw.pidOuter.setPlanNum(YAW_PID_NUM);
//	MPitch.pidInner.setPlanNum(PITCH_PID_NUM);
//	MPitch.pidOuter.setPlanNum(PITCH_PID_NUM);

	//Yaw轴机械角内外环
	loadPara(&yawInner[0],1.5,0,0,100,MYaw.getMotorCurrentLimit());
	loadPara(&yawOuter[0],1,0,0,100,MYaw.getMotorSpeedLimit());
	//Pitch轴机械角内外环
//	loadPara(&pitchInner[0],15,0,0,100,MYaw.getMotorCurrentLimit());
//	loadPara(&pitchOuter[0],2,0.5,0,100,MYaw.getMotorSpeedLimit());
//	//Pitch轴陀螺仪内外环
//	loadPara(&pitchInner[1],1,0,0,100,MYaw.getMotorCurrentLimit());
//	loadPara(&pitchOuter[1],1,0,0,100,MYaw.getMotorSpeedLimit());
	

	MYaw.pidInner.paramPtr = yawInner;
	MYaw.pidOuter.paramPtr = yawOuter;
	
//	MPitch.pidInner.paramPtr = pitchInner;
//	MPitch.pidOuter.paramPtr = pitchOuter;

	MYaw.pidInner.fbValuePtr[0] = &MYaw.canInfo.speed;
	MYaw.pidOuter.fbValuePtr[0] = &MYaw.canInfo.totalEncoder;
	
//	MPitch.pidInner.fbValuePtr[0] = &MPitch.canInfo.speed;
//	MPitch.pidOuter.fbValuePtr[0] = &MPitch.canInfo.encoder;
//	
//	MPitch.pidInner.fbValuePtr[1] = &icm20602.gyro.dps.data[2];
//	MPitch.pidOuter.fbValuePtr[1] = &icm20602.Angle.pitch;

		
	//Yaw轴初始校准
	offsetErrCode = yawOffset();
	
	//Kf mouseYkalman;	//PITCH 鼠标Y轴卡尔曼滤波
	FivePower pitchCurver;	//YAW 五次三项曲线
	pitchSetPos=4050;//中间值
	while(1)
	{
		//脱力双重保护
		if(RC.Key.SW2 == RCS::Down)
		{
			//延时1ms
			vTaskDelay(pdMS_TO_TICKS(1));
			continue;
		}
			
		//用户切换云台YAW
		if(RC.Key.CTRL && RC.KeyPress.V)
		{
			gimbalState =! gimbalState;
		}
		
		if(RC.Key.SW1 == RCS::Up)
		{
			gimbalState =1;
		}
		else
			gimbalState =0;
		
		//状态机开始
		switch(gimbalState)
		{
			case 0: //默认状态，摇杆和鼠标均能控制云台Pitch
//				pitchSetPos -= RC.Key.CH[1]/150.0f;//遥控右拨杆Y轴
//				pitchSetPos += mouseYkalman.KalmanFilter((double) RC.Key.CH[7]*15, 2.5f, 330.2f, 1);;//鼠标Y轴
				yawTarPos = 0;	//yaw复位
				break;
			case 1: //看肚子状态，
//				pitchSetPos = 0;
				yawTarPos = -0;
				break;
			default:
				break;
		}
		pitchSetPos = LIMIT(pitchSetPos,3700,4600);
		
		//用户切换Yaw轴PID方案
		if(RC.Key.CTRL  && RC.Key.SHIFT && RC.KeyPress.Z)
		{
			useImuFlag = disable;	//失能PITCH轴IMU
		}
		
		//用户设置Yaw轴校准值
		if(RC.Key.CTRL && RC.Key.X)
			yawUserOffset += 0.01;
		else if(RC.Key.CTRL && RC.Key.V)
			yawUserOffset -= 0.01;
		
		//校准值不可能超过360度，避免误差过大导致的输出过大
		yawUserOffset = LIMIT(yawUserOffset,-360,360);
		
		//PITCH轴曲线
//		pitchCurver.CurveModel(yawTarPos,&yawSetPos,1,1);
		
		//电机执行
		MYaw.ctrlPosition(yawSetPos + yawUserOffset);
//		MPitch.ctrlPosition(pitchSetPos, useImuFlag);
		
		//延时1ms
		vTaskDelay(pdMS_TO_TICKS(5));
	}
}




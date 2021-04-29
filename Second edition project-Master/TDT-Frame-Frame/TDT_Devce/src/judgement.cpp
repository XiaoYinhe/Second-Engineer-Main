#include "judgement.h"
#include "crc.h"
#include "FreeRTOS.h"
#include "timers.h"

#if USE_JUDGEMENT
Judgement judgement;

static const u16 RED_BLUE_ID_DIFF = 100;
static const u16 ROBOT_CLIENT_ID_DIFF = 256;

/**
 * @brief 队列处理
 * 
 * @param xTimer 
 */
void ringQueue(TimerHandle_t xTimer)
{
	judgement.ringQueue();
}

/**
 * @brief 初始化
 * 
 */
void Judgement::init()
{
	uart6Config();

	xTimerStart(xTimerCreate("", pdMS_TO_TICKS(1), pdTRUE, (void *)5, ::ringQueue), 0);
}

void Judgement::uart6Config()
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	USART_DeInit(USART6);
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART6, &USART_InitStructure);
	USART_Cmd(USART6, ENABLE);

	USART_DMACmd(USART6, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DMA_DeInit(DMA2_Stream1);
	DMA_InitStructure.DMA_Channel = DMA_Channel_5;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART6->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)judgeDataBuffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = JUDGE_BUFFER_LENGTH;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream1, &DMA_InitStructure);

	DMA_ITConfig(DMA2_Stream1, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA2_Stream1, ENABLE);
}



void DMA2_Stream1_IRQHandler(void)
{
	if (DMA_GetITStatus(DMA2_Stream1, DMA_IT_TCIF1))
	{
		judgement.addFullCount();

		DMA_ClearFlag(DMA2_Stream1, DMA_IT_TCIF1);
		DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);
	}
}

static u8 CRC8_Check = 0xff;
static uint16_t CRC16_Check = 0xffff;
static u8 start_state = 0;

static uint8_t byte = 0;
static int64_t read_len, rx_len;
static int read_arr;
static uint16_t len;
static int32_t index = 0;
static uint8_t CRC16_char[2];


void Judgement::ringQueue()
{
	rx_len = JUDGE_BUFFER_LENGTH - DMA_GetCurrDataCounter(DMA2_Stream1) + judgementFullCount * JUDGE_BUFFER_LENGTH;
	if (rx_len > read_len + JUDGE_BUFFER_LENGTH + 1)
		wrongStatusCnt.CYCLE_Wrong_cnt++;
	while (rx_len > read_len + 1)
	{
		read_arr = read_len % JUDGE_BUFFER_LENGTH;
		byte = judgeDataBuffer[read_arr];
		/* 独立计算CRC，平摊算力 */
		if (start_state < 2)
			CRC8_Check = CRC8_TAB[CRC8_Check ^ (byte)];
		if (start_state < 3)
			CRC16_Check = ((uint16_t)(CRC16_Check) >> 8) ^ wCRC_Table[((uint16_t)(CRC16_Check) ^ (uint16_t)(byte)) & 0x00ff];
		read_len++;
		switch (judgementStep)
		{
		case STEP_HEADER:
			if (start_state == 0)
			{

				if (byte == 0xA5)
				{
					fullDataBuffer[index++] = byte;
					start_state = 1;
				}
				else //错位
				{
					index = 0;
					start_state = 0;
					CRC8_Check = 0xff;
					CRC16_Check = 0xffff;
				}
			}
			else
			{
				if (index < 2) //数据帧长度位
				{
					fullDataBuffer[index++] = byte;
				}
				else if (index < 3) //包序号位
				{

					fullDataBuffer[index++] = byte;
				}
				else
				{
					if (fullDataBuffer[index] != (u8)((uint16_t)byte + 1))
					{
						wrongStatusCnt.SEQ_Wrong_cnt++;
					}
					start_state = 2; //不再计算CRC8，避免冗余位用于计算CRC8
					fullDataBuffer[index++] = byte;
					judgementStep = STEP_HEADER_CRC8;
				}
			}
			break;
		case STEP_HEADER_CRC8:
		{
			fullDataBuffer[index++] = byte;
			if (((FrameHeader*)fullDataBuffer)->crc8 == CRC8_Check)
			{
				judgementStep = STEP_CMDID_GET;
				start_state = 2;
				CRC8_Check = 0xff;
			}
			else
			{
				wrongStatusCnt.CRC8_Wrong_cnt++;
				judgementStep = STEP_HEADER;
				index = 0;
				start_state = 0;
				CRC8_Check = 0xff;
				CRC16_Check = 0xffff;
			}
		}
		break;
		case STEP_CMDID_GET:
		{
			if (index < 7 - 1)
				fullDataBuffer[index++] = byte;
			else
			{
				fullDataBuffer[index++] = byte;
				judgementStep = STEP_DATA_TRANSFER;
				len = 7 + getLength((FrameHeader*)fullDataBuffer) + 2;
				if (len == 0xFF || len != ((FrameHeader*)fullDataBuffer)->dataLength + 9)
				{
					judgementStep = STEP_HEADER;
					index = 0;
					start_state = 0;
					CRC8_Check = 0xff;
					CRC16_Check = 0xffff;
				}
			}
		}
		break;
		case STEP_DATA_TRANSFER:
		{
			if (index < (len - 3))
			{
				fullDataBuffer[index++] = byte;
			}
			else
			{
				fullDataBuffer[index++] = byte;
				start_state = 3;
				judgementStep = STEP_DATA_CRC16;
			}
		}
		break;
		case STEP_DATA_CRC16:
		{
			if (index < (len))
			{
				fullDataBuffer[index++] = byte;
			}
			else
			{
				CRC16_char[0] = (u8)(CRC16_Check & 0x00ff);
				CRC16_char[1] = (u8)((CRC16_Check >> 8) & 0x00ff);
				if (CRC16_char[0] == fullDataBuffer[index - 2] && CRC16_char[1] == fullDataBuffer[index - 1])
				{
					getJudgeData();
				}
				else
					wrongStatusCnt.CRC16_Wrong_cnt++;
				{
					judgementStep = STEP_HEADER;
					index = 0;
					start_state = 0;
					CRC8_Check = 0xff;
					CRC16_Check = 0xffff;
				}
			}
		}
		break;
		default:
		{
			judgementStep = STEP_HEADER;
			index = 0;

			start_state = 0;
			CRC8_Check = 0xff;
			CRC16_Check = 0xffff;
		}
		break;
		}
		rx_len = JUDGE_BUFFER_LENGTH - DMA_GetCurrDataCounter(DMA2_Stream1) + judgementFullCount * JUDGE_BUFFER_LENGTH;
	}
	if (rx_len % JUDGE_BUFFER_LENGTH > (JUDGE_BUFFER_LENGTH / 3) && rx_len % JUDGE_BUFFER_LENGTH < (2 * JUDGE_BUFFER_LENGTH / 3)) //防止judgementFullCount溢出
	{
		read_len -= JUDGE_BUFFER_LENGTH * judgementFullCount;
		judgementFullCount = 0;
	}
}

/**
 * @brief 获取当前帧对应的长度
 * 
 * @param frameHeader 帧头结构体
 * @return uint8_t 长度
 */
uint8_t Judgement::getLength(FrameHeader *frameHeader)
{
	switch (frameHeader->cmdid)
	{
		case STATUS_DATA:
		return sizeof(GameStatus);
		break; //10Hz
		case RESULT_DATA:
		return sizeof(GameResult);
		break;
		case ROBOT_HP_DATA:
		return sizeof(GameRobotHP);
		break;
		case DART_STATUS:
		return sizeof(DartStatus);
		break;
		case ICRA_BUFF_DEBUFF_ZONE_STATUS:
		return sizeof(ICRA_BuffDebuffZoneStatus_t);
		break;

		case EVENT_DATA:
		return sizeof(EventData);
		break;				   //50hZ
		case SUPPLY_PROJECTILE_ACTION:
		return sizeof(SupplyProjectileAction);
		break; //10hz
		case ROBOT_WARNING_DATA:
		return sizeof(RefereeWarning);
		break;
		case DART_REMAINING_TIME:
		return sizeof(DartRemainingTime);
		break;

		case GAME_ROBOT_STATUS:
		return sizeof(GameRobotStatus);
		break;
		case POWER_HEAT_DATA:
		return sizeof(PowerHeatData);
		break; //50hz
		case GAME_ROBOT_POS:
		return sizeof(GameRobotPos);
		break;
		case BUFF:
		return sizeof(Buff);
		break;
		case AERIAL_ROBOT_ENERGY:
		return sizeof(AerialRobotEnergy);
		break;
		case ROBOT_HURT:
		return sizeof(RobotHurt);
		break;
		case SHOOT_DATA:
		return sizeof(ShootData);
		break;
		case BULLET_REMAINING:
		return sizeof(BulletRemaining);
		break;
		case RFID_STATUS:
		return sizeof(RfidStatus);
		break;
		case DART_CLIENT_CMD:
		return sizeof(DartClientCmd);
		break;

		case STUDENT_INTERACTIVE_HEADER_DATA:
			return frameHeader->dataLength;
			break;	
	}
	return 0xff;
}

/**
 * @brief 结构体复制
 * 
 */
void Judgement::getJudgeData()
{
	static NVIC_InitTypeDef NVIC_InitStructure;
	static DMA_InitTypeDef DMA_InitStructure;
	static u8 firstLoad = 1; //首次加载时加载配置缺省值
	unsigned char *judgeData_ADD;
	if (firstLoad == 1)
	{
		firstLoad = 0;
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)fullDataBuffer+sizeof(FrameHeader);	//外设基地址
		DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToMemory;						//数据传输方向：外设到内存
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;			//外设地址递增
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					//内置地址递增
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //数据宽度为八位
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//数据宽度为八位
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;							//不执行循环模式
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;						//dma通道拥有高优先级

		NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

		DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, DISABLE); //失能传输完成中断中断
	}
	u16 cmdid = ((FrameHeader*)fullDataBuffer)->cmdid;
	switch (cmdid)
	{
		case STATUS_DATA:
		judgeData_ADD = (u8*)&gameStatus;
		break; //10Hz
		
		case RESULT_DATA:
		judgeData_ADD = (u8*)&gameResult;
		break;
		
		case ROBOT_HP_DATA:
		judgeData_ADD = (u8*)&gameRobotHP;
		break;
		
		case DART_STATUS:
		judgeData_ADD = (u8*)&dartStatus;
		break;
		
		case ICRA_BUFF_DEBUFF_ZONE_STATUS:
		judgeData_ADD = (u8*)&ICRA_BuffDebuffZoneStatus;
		break;


		case EVENT_DATA:
		judgeData_ADD = (u8*)&eventData;
		break;				   //50hZ
		
		case SUPPLY_PROJECTILE_ACTION:
		judgeData_ADD = (u8*)&supplyProjectileAction;
		break; //10hz
		
		case ROBOT_WARNING_DATA:
		judgeData_ADD = (u8*)&refereeWarning;
		break;
		
		case DART_REMAINING_TIME:
		judgeData_ADD = (u8*)&dartRemainingTime;
		break;


		case GAME_ROBOT_STATUS:
		judgeData_ADD = (u8*)&gameRobotStatus;
		break;
		
		case POWER_HEAT_DATA:
		judgeData_ADD = (u8*)&powerHeatData;
		break; //50hz
		
		case GAME_ROBOT_POS:
		judgeData_ADD = (u8*)&gameRobotPos;
		break;
		
		case BUFF:
		judgeData_ADD = (u8*)&buff;
		break;
		
		case AERIAL_ROBOT_ENERGY:
		judgeData_ADD = (u8*)&aerialRobotEnergy;
		break;
		
		case ROBOT_HURT:
		judgeData_ADD = (u8*)&robotHurt;
		break;
		
		case SHOOT_DATA:
		judgeData_ADD = (u8*)&shootData;
		break;
		
		case BULLET_REMAINING:
		judgeData_ADD = (u8*)&bulletRemaining;
		break;
		
		case RFID_STATUS:
		judgeData_ADD = (u8*)&rfidStatus;
		break;
		
		case DART_CLIENT_CMD:
		judgeData_ADD = (u8*)&dartClientCmd;
		break;


		case STUDENT_INTERACTIVE_HEADER_DATA:
		judgeData_ADD = (u8*)&studentRecviveData;
		break;
		
		default : return;
	}
	DMA_DeInit(DMA2_Stream2);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)judgeData_ADD; //内存基地址
	DMA_InitStructure.DMA_BufferSize = getLength((FrameHeader*)fullDataBuffer);					 //dma缓存大小
	DMA_Init(DMA2_Stream2, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream2, ENABLE);
	NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief 针对0x301的联合体数据进行发送
 * 
 * @param count 
 */
void Judgement::customSend(u8 count)
{
	sendUnionData.frameHeader.sof = 0xa5;
	sendUnionData.frameHeader.dataLength = count - sizeof(FrameHeader) - 2;
	Append_CRC8_Check_Sum((unsigned char *)&sendUnionData, 5);
	sendUnionData.senderId = gameRobotStatus.robotId;

	if(sendUnionData.receiverId != ClientId())
		sendUnionData.receiverId = IdToMate(sendUnionData.receiverId);

	Append_CRC16_Check_Sum((unsigned char *)&sendUnionData, count);

	uart6SendBytes(&sendUnionData, count);
}

/**
 * @brief 发送机器人交互数据
 * 
 * @param dataLength 交互内容的长度
 */
void Judgement::robotsCommunication(uint16_t dataCmdId, RobotIdDef robotIdDef, u8 dataLength)
{
	sendUnionData.dataCmdId = dataCmdId;
	sendUnionData.receiverId = robotIdDef;
	int count = sizeof(FrameHeader) + 6 + dataLength + 2;
	sendUnionData.frameHeader.cmdid = 0x0301;
	//    unsigned char send_Buffer[17];
	customSend(count);
}

/**
 * @brief 发送客户端UI命令
 * 
 * @param sendGraphics 一次发送的图形数目
 */
void Judgement::graphicDraw(u8 sendGraphics)
{
	if (sendGraphics <= 1)
	{
		sendUnionData.dataCmdId = 0x101;
		sendGraphics = 1;
	}
	else if (sendGraphics <= 2)
	{
		sendUnionData.dataCmdId = 0x102;
		sendGraphics = 2;
	}
	else if (sendGraphics <= 5)
	{
		sendUnionData.dataCmdId = 0x103;
		sendGraphics = 5;
	}
	else if (sendGraphics <= 7)
	{
		sendUnionData.dataCmdId = 0x104;
		sendGraphics = 7;
	}
	sendUnionData.frameHeader.cmdid = 0x0301;

	int count = sizeof(FrameHeader) + 6 + sendGraphics * sizeof(GraphicDataStruct) + 2;
	sendUnionData.receiverId = ClientId();
	sendUnionData.senderId = gameRobotStatus.robotId;

	customSend(count);
}

/**
 * @brief 字符绘制
 * 
 */
void Judgement::characterDraw()
{
	sendUnionData.frameHeader.cmdid = 0x0301;
	sendUnionData.dataCmdId = 0x110;
	int count = sizeof(FrameHeader) + 6 + sizeof(GraphicDataStruct) + 30 + 2;
	sendUnionData.receiverId = ClientId();
	sendUnionData.senderId = gameRobotStatus.robotId;

	customSend(count);
}


/**
 * @brief 删除图形
 * 
 */
void Judgement::graphicDel()
{
	sendUnionData.frameHeader.cmdid = 0x0301;
	sendUnionData.dataCmdId = 0x100;
	int count = sizeof(FrameHeader) + 6 + sizeof(ClientCustomGraphicDelete) + 2;
	sendUnionData.receiverId = ClientId();
	sendUnionData.senderId = gameRobotStatus.robotId;

	customSend(count);
}

/**
 * @brief 发送地图命令
 * 
 */
void Judgement::mapCommandSend()
{
	mapCommandData.frameHeader.cmdid = 0x0303;
	int count = sizeof(FrameHeader) + sizeof(MapCommand) + 2;

	mapCommandData.frameHeader.sof = 0xa5;
	mapCommandData.frameHeader.dataLength = count - sizeof(FrameHeader) - 2;
	Append_CRC8_Check_Sum((unsigned char *)&mapCommandData, 5);
	Append_CRC16_Check_Sum((unsigned char *)&mapCommandData, count);

	uart6SendBytes(&mapCommandData, count);
}

/**
 * @brief 发送多字节数据（采用DMA）
 * 
 * @param ptr 地址
 * @param len 长度
 */
void Judgement::uart6SendBytes(void *ptr, u8 len)
{
	static DMA_InitTypeDef DMA_InitStructure;
	static NVIC_InitTypeDef NVIC_InitStructure;
	static u8 firstLoad = 1; //首次加载时加载配置缺省值
	if (firstLoad == 1)
	{
		firstLoad = 0;
		DMA_InitStructure.DMA_Channel = DMA_Channel_5;
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART6->DR); //外设基地址
		DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;				  //数据传输方向：内存到外设
		//dma缓存大小
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//外设地址不变
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					//内置地址寄存器递增
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //数据宽度为八位
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//数据宽度为八位
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;							//工作模式为环形
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;						//dma通道拥有高优先级

		NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

		DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, DISABLE);
	}
	DMA_Cmd(DMA2_Stream7, DISABLE);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)ptr;							//内存基地址
	DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);									//|DMA_FLAG_FEIF7|DMA_FLAG_HTIF7);
	DMA_InitStructure.DMA_BufferSize = len;											//dma缓存大小
	DMA_Init(DMA2_Stream7, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream7, ENABLE);
	NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief 将ID枚举转换为友方机器人id
 * 
 * @param robotIdDef ID枚举
 * @return uint16_t 友方机器人id
 */
uint16_t Judgement::IdToMate(RobotIdDef robotIdDef)
{
	if(gameRobotStatus.robotId > RED_BLUE_ID_DIFF)
	{
		return (uint16_t)robotIdDef + RED_BLUE_ID_DIFF;
	}
	return (uint16_t)robotIdDef;
}

/**
 * @brief 将机器人id转换为友方机器人id
 * 
 * @param robotId 有效的机器人id
 * @return uint16_t 友方机器人id
 */
uint16_t Judgement::IdToMate(uint16_t robotId)
{
	return IdToMate(RobotIdDef(robotId % RED_BLUE_ID_DIFF));
}

/**
 * @brief 根据自身id返还客户端id
 * 
 * @return uint16_t 客户端id
 */
uint16_t Judgement::ClientId()
{
	return gameRobotStatus.robotId + ROBOT_CLIENT_ID_DIFF;
}

/**
 * @brief 将ID枚举转换为敌方机器人id
 * 
 * @param robotIdDef ID枚举
 * @return uint16_t 敌方机器人id
 */
uint16_t Judgement::IdToEnemy(RobotIdDef robotIdDef)
{
	if(gameRobotStatus.robotId < RED_BLUE_ID_DIFF)
	{
		return (uint16_t)robotIdDef + RED_BLUE_ID_DIFF;
	}
	return robotIdDef;
}

/**
 * @brief 将机器人id转换为敌方机器人id
 * 
 * @param robotId 有效的机器人id
 * @return uint16_t 敌方机器人id
 */
uint16_t Judgement::IdToEnemy(uint16_t robotId)
{
	return IdToEnemy(RobotIdDef(robotId % RED_BLUE_ID_DIFF));
}


#endif
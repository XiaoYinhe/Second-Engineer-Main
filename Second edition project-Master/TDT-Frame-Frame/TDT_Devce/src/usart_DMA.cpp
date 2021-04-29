/***************
@brief 主副控通信，串口3



**************/

#include "usart_DMA.h"
#include "crc.h"

static uint8_t CONTROL_rx_buf[2][CONTROL_RX_BUF_NUM];    //副控发送的数据，二维数组,双缓冲模式
static uint8_t CONTROL_tx_buf[CONTROL_TX_BUF_NUM];     //向副控传送数据的地址
static uint8_t CONTROL_tx_tran_buf[CONTROL_TX_BUF_NUM];   //向外设传输数据的中转站   tx_bufdata到tx_buf到USART1->DR
 
void usart_DMA_init()
{
	//USART3-TX  PB10 
	//USART3-RX  PB11
	/* -------------- Enable Module Clock Source ----------------------------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3, DISABLE);
    
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3); 
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
	
	/* -------------- Configure GPIO ---------------------------------------*/
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 |GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	USART_DeInit(USART3);										
	
	USART_InitStructure.USART_BaudRate = 1000000;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART3, &USART_InitStructure);
	
	USART_DMACmd(USART3, USART_DMAReq_Rx|USART_DMAReq_Tx, ENABLE);
	
	USART_ClearFlag(USART3, USART_FLAG_IDLE);
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
	
	USART_Cmd(USART3, ENABLE);
	
	/* -------------- Configure NVIC ---------------------------------------*/
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}	
	//DMA1 stream1 ch4 RX  or (DMA1 stream3 ch4 TX)    !!!!!!! P205 of the datasheet
	/* -------------- Configure RX DMA -----------------------------------------*/
{
	DMA_InitTypeDef DMA_InitStructure;
	
	DMA_DeInit(DMA1_Stream1);
	
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART3->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)CONTROL_rx_buf[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = CONTROL_RX_BUF_NUM;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream1, &DMA_InitStructure);
	
	DMA_DoubleBufferModeConfig(DMA1_Stream1, (uint32_t)CONTROL_rx_buf[1], DMA_Memory_0);
	DMA_DoubleBufferModeCmd(DMA1_Stream1, ENABLE);
	DMA_Cmd(DMA1_Stream1, DISABLE); //Add a disable
	DMA_Cmd(DMA1_Stream1, ENABLE);
}
	/* -------------- Configure TX DMA -----------------------------------------*/
{	
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	    
	DMA_DeInit(DMA1_Stream3);
	
	DMA_DeInit(DMA1_Stream3);
	DMA_InitStructure.DMA_Channel= DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)CONTROL_tx_buf;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = CONTROL_TX_BUF_NUM;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream3,&DMA_InitStructure);
	
	DMA_ITConfig(DMA1_Stream3,DMA_IT_TC,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	DMA_Cmd(DMA1_Stream3,DISABLE);
}	
	
}





/******接收中断******/
void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3,  USART_IT_TC) != RESET)
    {
        /* 关闭发送完成中断  */ 
        USART_ITConfig(USART3,USART_IT_TC,DISABLE);  
//        Flag_Tx_Gsm_Busy = 0; 
        USART_ClearITPendingBit(USART3, USART_IT_TC);           

    }
    else if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        USART_ReceiveData(USART3);
    }                      
    
    else if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
    {
        static uint16_t this_time_rx_len = 0;
        USART_ReceiveData(USART3);

        if(DMA_GetCurrentMemoryTarget(DMA1_Stream1) == 0)
        {
            //重新设置DMA
            DMA_Cmd(DMA1_Stream1, DISABLE);
            this_time_rx_len = CONTROL_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA1_Stream1);
            DMA_SetCurrDataCounter(DMA1_Stream1, CONTROL_RX_BUF_NUM);
            DMA1_Stream1->CR |= DMA_SxCR_CT;
            //清DMA中断标志
            DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);
            DMA_Cmd(DMA1_Stream1, ENABLE);
            if(this_time_rx_len == CONTROL_FRAME_LENGTH)
            {
                //处理接收的数据
                Rx_data_processing(CONTROL_rx_buf[0]);
            }
        }
        else
        {
            //重新设置DMA
            DMA_Cmd(DMA1_Stream1, DISABLE);
            this_time_rx_len = CONTROL_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA1_Stream1);
            DMA_SetCurrDataCounter(DMA1_Stream1, CONTROL_RX_BUF_NUM);
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            //清DMA中断标志，其中TCIFx传输完成中断标志，HTIFx半传输中断标志
            DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);
            DMA_Cmd(DMA1_Stream1, ENABLE);
            if(this_time_rx_len == CONTROL_FRAME_LENGTH)
            {
                //处理接收的数据
                Rx_data_processing(CONTROL_rx_buf[1]);
            }
        }
    }
}

u8 Translation_STA;
u8 uplift_STA;//肚子里的抬升矿石
u8 climb_STA;//气泵鼻子向前爬
u8 pumpTurn_STA;//气泵鼻子转完成标志
u8 pump_STA;//吸气完成标志
/****接收数据处理****/
static void Rx_data_processing(uint8_t *comm_buf)
{
	if(comm_buf[0]==0xFF &&  (Verify_CRC16_Check_Sum(comm_buf,CONTROL_FRAME_LENGTH)))
    {    
		//commLostCnt=200;//串口丢失标志位
		
            climb_STA = (u8)(comm_buf[1]>>4);
			pumpTurn_STA = (u8)(comm_buf[1]&0x0f);
			pump_STA=(u8)(comm_buf[2]>>7);
			uplift_STA=(u8)(comm_buf[2]&0x7f);
		
    }

}


int16_t tranFeed=0;
u8 climb_CMD;//气泵鼻子向前爬
u8 pumpTurn_CMD;//气泵鼻子转
u8 pump_CMD;//吸气
u8 turntable_CMD;//肚子转盘转
u8 clip_CMD;//肚子夹子
void Communicate_SendChar(void)  
{  
	
//	//平移架位置手动补偿
//	if((RC.CH[10]&KEY_Z) && (RC.CH[10]&KEY_CTRL))
//		tranFeed--;
//	else if((RC.CH[10]&KEY_C) && (RC.CH[10]&KEY_CTRL))
//		tranFeed++;
//	if(upFlag==0)
//		tranFeed=0;
//	
//	
//	if(Stretch_Command==1)
//		Door_Commmand=0;
	
    CONTROL_tx_tran_buf[0]=0xFF;
	CONTROL_tx_tran_buf[1]=(u8)(climb_CMD<<4|clip_CMD<<2|turntable_CMD);
  	CONTROL_tx_tran_buf[2]=(u8)(pumpTurn_CMD);
    CONTROL_tx_tran_buf[3]=(u8)(pump_CMD<<4|turntable_CMD<<3);//气缸状态命令
    CONTROL_tx_tran_buf[4]=1;
    CONTROL_tx_tran_buf[5]=1;
    CONTROL_tx_tran_buf[6]=1;
	
	Append_CRC16_Check_Sum(CONTROL_tx_tran_buf,CONTROL_TX_BUF_NUM);
	
	
	
	

	
	
	
	
    //等待空闲
//    while (Flag_Tx_Gsm_Busy);
//    Flag_Tx_Gsm_Busy = 1;

    //复制数据  
    memcpy(CONTROL_tx_buf,CONTROL_tx_tran_buf,CONTROL_TX_BUF_NUM);  
    //设置传输数据长度  
    DMA_SetCurrDataCounter(DMA1_Stream3,CONTROL_TX_BUF_NUM);  
    //打开DMA,开始发送  
    DMA_Cmd(DMA1_Stream3,ENABLE); 
    delayUs(100);   
}  


void DMA1_Stream3_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_Stream3,DMA_IT_TCIF3) != RESET)   
    {  
        /* 清除标志位 */
        DMA_ClearFlag(DMA1_Stream3,DMA_IT_TCIF3);  
        /* 关闭DMA */
        DMA_Cmd(DMA1_Stream3,DISABLE);
        /* 打开发送完成中断,确保最后一个字节发送成功 */
        USART_ITConfig(USART3,USART_IT_TC,ENABLE);  
    }  
}


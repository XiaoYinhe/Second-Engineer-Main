#ifndef _USART_DMA__H
#define _USART_DMA__H

#include "board.h"

#define CONTROL_RX_BUF_NUM 10u      //副控发送字节数的2倍，无符号整型存储
#define CONTROL_FRAME_LENGTH 5u      //副控发送字节数    
#define CONTROL_TX_BUF_NUM 9u      //发送给副控的数据字节数

static void Rx_data_processing(uint8_t *comm_buf);

extern u8 Translation_STA;
extern u8 uplift_STA;
extern u8 climb_STA;
extern u8 pumpTurn_STA;//气泵鼻子转完成标志
extern u8 pump_STA;//吸气完成标志


extern int16_t tranFeed;
extern u8 uplift_CMD;//肚子里的抬升矿石
extern u8 climb_CMD;//气泵鼻子向前爬
extern u8 pumpTurn_CMD;//气泵鼻子转
extern u8 pump_CMD;//吸气
extern u8 turntable_CMD;//肚子转盘转
extern u8 clip_CMD;//肚子夹子
#endif


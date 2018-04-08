#ifndef __USART_H
#define __USART_H

#include "Lib_usart.h"

typedef struct
{
  char *data;                  /*数据缓冲区*/
	u16 Recv_Len;               /*一帧长度*/	
	u8 Recv_Flag;		           /*收到一帧标志*/
	u8 Send_Finish;            /*一帧发送完毕*/
}_Recv_Str;

void InitAllUsart(void);


#endif




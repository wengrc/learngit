#ifndef __USART_H
#define __USART_H

#include "Lib_usart.h"

typedef struct
{
  char *data;                  /*���ݻ�����*/
	u16 Recv_Len;               /*һ֡����*/	
	u8 Recv_Flag;		           /*�յ�һ֡��־*/
	u8 Send_Finish;            /*һ֡�������*/
}_Recv_Str;

void InitAllUsart(void);


#endif




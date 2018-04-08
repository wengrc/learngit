

#include "UART3.h"
#include "string.h"
#include "NAV350.h"
#include "Lib_retarget_printf.h"

char             UART3_Recv_Buf[UART3_MAX_ARR][UART3_MAX_RECV_LEN]; /*定义N个缓冲区用于轮流接收串口底层协议*/

u8               UART3_User_Buf_No = 0;
_Recv_Str        UART3_Str;
Recv_Str         uart3_Recv_Buf[BufSize];
Recv_Str         Recv_Buf;
int              isWriteForUart3 = 0;

DMA_InitTypeDef  UART3_DMA_TX,UART3_DMA_RX;

/*
 *********************************************************************************************************
 *	函 数 名: USART3_Init
 *	功能说明: 初始化CPU的USART3串口硬件设备。用于和磁传感器通信
 *	形    参：_YD_UART_BAUD:波特率;
 *	返 回 值: 无
 *********************************************************************************************************
 */
void USART3_Init ( _UART_BAUD BaudRate )
{
  USART_InitTypeDef  USART_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;

  RCC_APB2PeriphClockCmd ( USART3_TX_PORT_CLK | USART3_RX_PORT_CLK, ENABLE );

  RCC_APB1PeriphClockCmd ( RCC_APB1Periph_USART3, ENABLE );

  /* Configure USART Tx as alternate function  */
  GPIO_InitStructure.GPIO_Mode                         = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed                        = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin                          = USART3_TX_PIN;
  GPIO_Init ( USART3_TX_PORT, &GPIO_InitStructure );

  /* Configure USART Rx as alternate function  */
  GPIO_InitStructure.GPIO_Mode                         = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Pin                          = USART3_RX_PIN;
  GPIO_Init ( USART3_RX_PORT, &GPIO_InitStructure );


  /* USARTx configured as follow:
   *     - BaudRate = 115200 baud
   *     - Word Length = 8 Bits
   *     - One Stop Bit
   *     - No parity
   *     - Hardware flow control disabled (RTS and CTS signals)
   *     - Receive and transmit enabled
   */
  USART_InitStructure.USART_BaudRate                   = BaudRate;
  USART_InitStructure.USART_WordLength                 = USART_WordLength_9b;
  USART_InitStructure.USART_StopBits                   = USART_StopBits_1;
  USART_InitStructure.USART_Parity                     = USART_Parity_Even; //USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl        = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode                       = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init ( USART3, &USART_InitStructure );

  NVIC_InitStructure.NVIC_IRQChannel                   = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init ( &NVIC_InitStructure );

#if UART3_DMA_RX_ENABLE

  /*空闲中断*/
  USART_ITConfig ( USART3, USART_IT_IDLE, ENABLE );
#else
  USART_ITConfig ( USART3, USART_IT_RXNE | USART_IT_IDLE, ENABLE );
#endif

  /* Enable USART */
  USART_Cmd ( USART3, ENABLE );

  /*
   *  CPU的小缺陷：串口配置好，如果直接Send，则第1个字节发送不出去
   *  如下语句解决第1个字节无法正确发送出去的问题：
   *  清发送完成标志，Transmission Complete flag
   */
  USART_ClearFlag ( USART3, USART_FLAG_TC );

  memset ( (u8 *)&UART3_Str,0x00,sizeof(UART3_Str) );
#if UART3_DMA_RX_ENABLE
  USART3_RX_DMA ();
#endif
#if UART3_DMA_TX_ENABLE
  USART3_TX_DMA ();
  UART3_Str.Send_Finish = 1;
#endif

}

#if UART3_DMA_RX_ENABLE

/*
 *********************************************************************************************************
 *	函 数 名: USART3_RX_DMA
 *	功能说明: 设置USART3 DMA接收方式
 *	形    参：无
 *	返 回 值: 无
 *********************************************************************************************************
 */
void USART3_RX_DMA ( void )
{
  //DMA_InitTypeDef UART1_DMA_RX;
  NVIC_InitTypeDef  NVIC_InitStructure;

  RCC_AHBPeriphClockCmd ( RCC_AHBPeriph_DMA1, ENABLE ); //开DMA时钟

  DMA_StructInit ( &UART3_DMA_RX );

  UART3_DMA_RX.DMA_PeripheralBaseAddr                  = (u32) & USART3->DR;
  UART3_DMA_RX.DMA_MemoryBaseAddr                     = (u32) & UART3_Recv_Buf[UART3_User_Buf_No];
  UART3_DMA_RX.DMA_DIR 								   = DMA_DIR_PeripheralSRC;
  UART3_DMA_RX.DMA_BufferSize                          = UART3_MAX_RECV_LEN;
  UART3_DMA_RX.DMA_PeripheralInc                       = DMA_PeripheralInc_Disable;
  UART3_DMA_RX.DMA_MemoryInc                           = DMA_MemoryInc_Enable;
  UART3_DMA_RX.DMA_PeripheralDataSize                  = DMA_PeripheralDataSize_Byte;
  UART3_DMA_RX.DMA_MemoryDataSize                      = DMA_MemoryDataSize_Byte;
  UART3_DMA_RX.DMA_Mode 							   = DMA_Mode_Normal; //DMA_Mode_Circular;
  UART3_DMA_RX.DMA_Priority                            = DMA_Priority_Medium;
  UART3_DMA_RX.DMA_M2M								   = DMA_M2M_Disable;
  DMA_Init ( DMA1_Channel3, &UART3_DMA_RX );

  USART_DMACmd ( USART3, USART_DMAReq_Rx, ENABLE );
  DMA_Cmd ( DMA1_Channel3, ENABLE );


  DMA_ITConfig ( DMA1_Channel3, DMA_IT_TC, ENABLE ); //DMA1_Channel3传输完成中断
  DMA_ITConfig ( DMA1_Channel3, DMA_IT_TE, ENABLE );


  NVIC_InitStructure.NVIC_IRQChannel                   = DMA1_Channel3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init ( &NVIC_InitStructure );


}


#endif
#if UART3_DMA_TX_ENABLE

/*
 *********************************************************************************************************
 *	函 数 名: USART3_TX_DMA
 *	功能说明: 设置USART3 DMA发送方式
 *	形    参：无
 *	返 回 值: 无
 *********************************************************************************************************
 */
void USART3_TX_DMA ( void )
{
  //DMA_InitTypeDef DMA_InitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;

  RCC_AHBPeriphClockCmd ( RCC_AHBPeriph_DMA1, ENABLE ); //开DMA时钟

  DMA_StructInit ( &UART3_DMA_TX );

  UART3_DMA_TX.DMA_PeripheralBaseAddr                  = (u32) & USART3->DR;
  UART3_DMA_TX.DMA_DIR                                 = DMA_DIR_PeripheralDST;
  UART3_DMA_TX.DMA_PeripheralInc                       = DMA_PeripheralInc_Disable;
  UART3_DMA_TX.DMA_MemoryInc                           = DMA_MemoryInc_Enable;
  UART3_DMA_TX.DMA_PeripheralDataSize                  = DMA_PeripheralDataSize_Byte;
  UART3_DMA_TX.DMA_MemoryDataSize                      = DMA_MemoryDataSize_Byte;
  UART3_DMA_TX.DMA_Mode                                = DMA_Mode_Normal; //DMA_Mode_Circular;
  UART3_DMA_TX.DMA_Priority                            = DMA_Priority_Medium;
  UART3_DMA_TX.DMA_M2M								   = DMA_M2M_Disable;

  DMA_Init ( DMA1_Channel2, &UART3_DMA_TX );

  USART_DMACmd ( USART3, USART_DMAReq_Tx, ENABLE );
  DMA_ITConfig ( DMA1_Channel2, DMA_IT_TC, ENABLE ); //DMA1_Channel2传输完成中断
  DMA_ITConfig ( DMA1_Channel2, DMA_IT_TE, ENABLE );

  NVIC_InitStructure.NVIC_IRQChannel                   = DMA1_Channel2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init ( &NVIC_InitStructure );

}


#endif

/*
 *********************************************************************************************************
 *	函 数 名: USART3_Send_Byte
 *	功能说明: 设置USART3 发送一个字节
 *	形    参：无
 *	返 回 值: 无
 *********************************************************************************************************
 */
unsigned char USART3_Send_Byte ( unsigned char ch )
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData ( USART3, (uint8_t)ch ); /*发送一个字符函数*/

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus ( USART3, USART_FLAG_TXE ) == RESET) { /*等待发送完成*/

  }
  return ch;
}


/*
 *********************************************************************************************************
 *	函 数 名: USART3_Send_Str
 *	功能说明: 设置USART3 发送一串字符
 *	形    参：无
 *	返 回 值: 无
 *********************************************************************************************************
 */
unsigned char USART3_Send_Str ( char *buf,uint16_t len )
{

#if UART3_DMA_TX_ENABLE

  //while(!UART3_Str.Send_Finish);
  UART3_Str.Send_Finish            = 0;

  DMA_Cmd ( DMA1_Channel2, DISABLE );
  UART3_DMA_TX.DMA_MemoryBaseAddr = (u32)buf;
  UART3_DMA_TX.DMA_BufferSize      = len;
  DMA_Init ( DMA1_Channel2, &UART3_DMA_TX );
  DMA_Cmd ( DMA1_Channel2, ENABLE );
#else
  uint16_t  i;

  for (i = 0; i < len; i++) {
    USART_SendData ( USART3, *buf++ );
    while (USART_GetFlagStatus ( USART3, USART_FLAG_TXE ) == RESET) {

    }
  }

#endif
  return 0;
}


void USART3_IRQHandler ( void )
{
	int i;
  if (USART_GetITStatus ( USART3, USART_IT_IDLE ) != RESET)//如果为空闲总线中断 
	{ 
		isWriteForUart3 = 1;
		
#if UART3_DMA_RX_ENABLE
    DMA_Cmd ( DMA1_Channel3, DISABLE );  //关闭DMA,防止处理其间有数据
    
#if Debug_Flag
		ShowTime();
		RTT_printf("Uart3 Data-----> %s \n",UART3_Recv_Buf[UART3_User_Buf_No]);
#endif
		
		UART3_Str.Recv_Len = UART3_MAX_RECV_LEN - DMA_GetCurrDataCounter ( DMA1_Channel3 );
    if (UART3_Str.Recv_Len > 0) 
		{
			for(i=0; i<BufSize; i++)
			{
				if(uart3_Recv_Buf[i].Recv_Flag == 0)
				{
					memcpy(uart3_Recv_Buf[i].data,UART3_Recv_Buf[UART3_User_Buf_No],100*sizeof(char));
					Usart3_ClearBUF(UART3_User_Buf_No);
					uart3_Recv_Buf[i].Recv_Flag = 1;
					uart3_Recv_Buf[i].Recv_Len = UART3_Str.Recv_Len;
					break;
				}
				else if(i == (BufSize-1))
				{
					memcpy(uart3_Recv_Buf[BufSize-1].data,UART3_Recv_Buf[UART3_User_Buf_No],100*sizeof(char));
					Usart3_ClearBUF(UART3_User_Buf_No);
					uart3_Recv_Buf[BufSize-1].Recv_Flag = 1;
					uart3_Recv_Buf[BufSize-1].Recv_Len = UART3_Str.Recv_Len;
				}
			}
    }
		
    DMA_ClearFlag (  DMA1_FLAG_TC3 | DMA1_FLAG_TE3 | DMA1_FLAG_HT3 ); //清标志
    DMA_SetCurrDataCounter ( DMA1_Channel3,UART3_MAX_RECV_LEN );    //重装填
    UART3_DMA_RX.DMA_MemoryBaseAddr = (u32) & UART3_Recv_Buf[UART3_User_Buf_No];
    DMA_Init ( DMA1_Channel3, &UART3_DMA_RX );

    //DMA_SetCurrDataCounter(DMA2_Stream2,MAX_RCV_LEN);//重装填
    DMA_Cmd ( DMA1_Channel3, ENABLE );     //处理完,重开DMA
		isWriteForUart3 = 0;
		
#else
    UART3_Str.data      = UART3_Recv_Buf[UART3_Str.User_Buf_No];
    UART3_Str.Recv_Flag = 1;
    UART3_Str.User_Buf_No++;
    if (UART3_Str.User_Buf_No >= UART3_MAX_ARR)
      UART3_Str.User_Buf_No = 0;
#endif

		
    //读SR后读DR清除Idle
    USART3->SR;
    USART3->DR;
  }
  
	if (USART_GetITStatus ( USART3, USART_IT_RXNE ) != RESET)//如果为接收中断
	{ 
    UART3_Recv_Buf[UART3_User_Buf_No][UART3_Str.Recv_Len] = USART3->DR;
    UART3_Str.Recv_Len++;
    if (UART3_Str.Recv_Len > UART3_MAX_RECV_LEN)
      UART3_Str.Recv_Len = 0;
  }
  if (USART_GetITStatus ( USART3, USART_IT_PE | USART_IT_FE | USART_IT_NE ) != RESET)//出错
	{ 
    USART_ClearITPendingBit ( USART3, USART_IT_PE | USART_IT_FE | USART_IT_NE );
  }
  USART_ClearITPendingBit ( USART3, USART_IT_IDLE );
}


 void  Usart3_ClearBUF(int num)
{
	memset(UART3_Recv_Buf[num],0,sizeof(char)*100);
}

/**
 * ISR for USART3 RX  DMA Stream Interrupt
 */
void DMA1_Channel3_IRQHandler ()
{
  // Channel3 transfer complete interrupt?
  if ( DMA_GetITStatus ( DMA1_IT_TC3 ) ) {
    // clear pending interrupt
    DMA_ClearITPendingBit ( DMA1_IT_TC3 );

    UART3_Str.Recv_Len = UART3_MAX_RECV_LEN - DMA_GetCurrDataCounter ( DMA1_Channel3 );
    if (UART3_Str.Recv_Len > 0)
		{
      UART3_Str.data      = UART3_Recv_Buf[UART3_User_Buf_No];
      UART3_Str.Recv_Flag = 1;
      UART3_User_Buf_No++;
      if (UART3_User_Buf_No >= UART3_MAX_ARR)
        UART3_User_Buf_No = 0;

    }
    DMA_ClearFlag ( DMA1_FLAG_TC3 | DMA1_FLAG_TE3 | DMA1_FLAG_HT3 ); //清标志
    DMA_SetCurrDataCounter ( DMA1_Channel3,UART3_MAX_RECV_LEN );                                                         //重装填

    UART3_DMA_RX.DMA_MemoryBaseAddr = (u32) & UART3_Recv_Buf[UART3_User_Buf_No];
    DMA_Init ( DMA1_Channel3, &UART3_DMA_RX );
  }
  DMA_ClearITPendingBit ( DMA1_IT_TE3 | DMA1_FLAG_HT3 );

}


/**
 * ISR for USART3 TX  DMA Stream Interrupt
 */
void DMA1_Channel2_IRQHandler ()
{
  // Channel2 transfer complete interrupt?
  if ( DMA_GetITStatus ( DMA1_IT_TC2 ) ) {
    // clear pending interrupt
    DMA_ClearITPendingBit( DMA1_IT_TC2 );
    UART3_Str.Send_Finish = 1;
    DMA_Cmd ( DMA1_Channel2, DISABLE );

  }
  DMA_ClearFlag ( DMA1_FLAG_TC2 | DMA1_FLAG_TE2 | DMA1_FLAG_HT2 ); //清标志
  DMA_ClearITPendingBit ( DMA1_IT_TE2 );

}






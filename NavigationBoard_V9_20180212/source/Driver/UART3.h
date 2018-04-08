

#ifndef YD_UART3_H
#define YD_UART3_H
#include "stm32f10x.h"

/*
*定义常用的波特率,用户可自由定义想要的波特率,但最高波特率不要超过115200
*
*/
typedef enum
{
	BAUD_1200 = 1200,
	BAUD_2400 = 2400,
	BAUD_4800 = 4800,
	BAUD_9600 = 9600,
	BAUD_19200 = 19200,
	BAUD_38400 = 38400,
	BAUD_56000 = 56000,
	BAUD_57600 = 57600,
	BAUD_115200 = 115200,
}_UART_BAUD;

#define UART3_DMA_RX_ENABLE   1
#define UART3_DMA_TX_ENABLE   1

#define USART3_TX_PIN   GPIO_Pin_10
#define USART3_RX_PIN   GPIO_Pin_11

#define USART3_TX_PORT   GPIOB
#define USART3_RX_PORT   GPIOB

#define USART3_TX_PORT_CLK   RCC_APB2Periph_GPIOB
#define USART3_RX_PORT_CLK   RCC_APB2Periph_GPIOB

#define USART3_TX_SOURCE GPIO_PinSource10
#define USART3_RX_SOURCE GPIO_PinSource11

/*
*定义通用串口数据结构体
*
*/
typedef struct
{
  char *data;                  /*数据缓冲区*/
	u16 Recv_Len;               /*一帧长度*/	
	u8 Recv_Flag;		           /*收到一帧标志*/
	u8 Send_Finish;            /*一帧发送完毕*/
}_Recv_Str;


typedef struct
{
  char data[100];                  /*数据缓冲区*/
	u16 Recv_Len;               /*一帧长度*/	
	u8 Recv_Flag;		           /*收到一帧标志*/
	u8 Handle_Flag;            /*一帧发送完毕*/
}Recv_Str;
/*
*定义串口单次接收的最大长度
*/
#define UART3_MAX_RECV_LEN    100
/*串口底层轮流接收缓冲区个数*/
#define UART3_MAX_ARR          1

#define BufSize 3
#define Debug_Flag 0

extern _Recv_Str UART3_Str;
extern Recv_Str  uart3_Recv_Buf[BufSize];
extern Recv_Str  Recv_Buf;
extern int       isWriteForUart3;

extern DMA_InitTypeDef UART3_DMA_TX,UART3_DMA_RX;


/*
*********************************************************************************************************
*	函 数 名: USART3_Init
*	功能说明: 初始化CPU的USART3串口硬件设备。用于和上层软件通信
*	形    参：_YD_UART_BAUD:波特率;cb:接收到数据后的回调函数
*	返 回 值: 无
*********************************************************************************************************
*/
 void  Usart3_ClearBUF(int num);
void USART3_Init(_UART_BAUD BaudRate);
#if UART3_DMA_RX_ENABLE
void USART3_RX_DMA(void);
#endif
#if UART3_DMA_TX_ENABLE
void USART3_TX_DMA(void);
#endif
unsigned char USART3_Send_Byte(unsigned char ch);
unsigned char USART3_Send_Str(char *buf,uint16_t len);

#endif












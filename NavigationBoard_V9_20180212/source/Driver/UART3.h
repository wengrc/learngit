

#ifndef YD_UART3_H
#define YD_UART3_H
#include "stm32f10x.h"

/*
*���峣�õĲ�����,�û������ɶ�����Ҫ�Ĳ�����,����߲����ʲ�Ҫ����115200
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
*����ͨ�ô������ݽṹ��
*
*/
typedef struct
{
  char *data;                  /*���ݻ�����*/
	u16 Recv_Len;               /*һ֡����*/	
	u8 Recv_Flag;		           /*�յ�һ֡��־*/
	u8 Send_Finish;            /*һ֡�������*/
}_Recv_Str;


typedef struct
{
  char data[100];                  /*���ݻ�����*/
	u16 Recv_Len;               /*һ֡����*/	
	u8 Recv_Flag;		           /*�յ�һ֡��־*/
	u8 Handle_Flag;            /*һ֡�������*/
}Recv_Str;
/*
*���崮�ڵ��ν��յ���󳤶�
*/
#define UART3_MAX_RECV_LEN    100
/*���ڵײ��������ջ���������*/
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
*	�� �� ��: USART3_Init
*	����˵��: ��ʼ��CPU��USART3����Ӳ���豸�����ں��ϲ����ͨ��
*	��    �Σ�_YD_UART_BAUD:������;cb:���յ����ݺ�Ļص�����
*	�� �� ֵ: ��
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












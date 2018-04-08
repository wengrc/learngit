
#include "USART.h"
#include "Lib_retarget_printf.h"
void InitAllUsart(void)
{
  
  struct com      COM1 =
  {
    USART3,
    RCC_APB1Periph_USART3,
    RCC_APB2Periph_GPIOB,
    RCC_APB2Periph_GPIOB,
  
    GPIOB,
    GPIO_Pin_9,
  
    GPIOB,
    GPIO_Pin_8,
  
    {
      115200,
      USART_WordLength_8b,
      USART_StopBits_1,
      USART_Parity_Even,
      USART_Mode_Rx | USART_Mode_Tx,
      USART_HardwareFlowControl_None
    }
  
  };

  USART_Config( &COM1 );
//  USART_Config( &COM2 );
//  USART_Config( &COM4 );

}


/*
*定义通用串口数据结构体
*
*/

#define UART1_MAX_RECV_LEN    100
/*串口底层轮流接收缓冲区个数*/
#define UART1_MAX_ARR          3

char             UART1_Recv_Buf[UART1_MAX_ARR][UART1_MAX_RECV_LEN]; /*定义N个缓冲区用于轮流接收串口底层协议*/
u8               UART1_User_Buf_No = 0;

_Recv_Str        UART1_Str;


void USART1_IRQHandler ( void )
{
  //u16 i;
  if (USART_GetITStatus ( USART1, USART_IT_IDLE ) != RESET) { //如果为空闲总线中断
    UART1_Str.data      = UART1_Recv_Buf[UART1_Str.Recv_Len];
    UART1_Str.Recv_Flag = 1;
    UART1_Str.Recv_Len++;
    if (UART1_Str.Recv_Len >= UART1_MAX_RECV_LEN)
      UART1_Str.Recv_Len = 0;

    //读SR后读DR清除Idle
    USART1->SR;
    USART1->DR;

  }
  if (USART_GetITStatus ( USART1, USART_IT_RXNE ) != RESET) { //如果为接收中断
    UART1_Recv_Buf[UART1_User_Buf_No][UART1_Str.Recv_Len] = USART1->DR;
    UART1_Str.Recv_Len++;
    if (UART1_Str.Recv_Len > UART1_MAX_RECV_LEN)
      UART1_Str.Recv_Len = 0;
  }
  if (USART_GetITStatus ( USART1, USART_IT_PE | USART_IT_FE | USART_IT_NE ) != RESET) { //出错
    USART_ClearITPendingBit ( USART1, USART_IT_PE | USART_IT_FE | USART_IT_NE );
  }
  USART_ClearITPendingBit ( USART1, USART_IT_IDLE );
}

/*  brief  receive data from usart2
 *  param  none
 *  retval none
 */

void USART2_IRQHandler( void )
{
  if(USART_GetITStatus( USART2, USART_IT_RXNE ) != RESET) {
    USART_ClearITPendingBit( USART2, USART_IT_RXNE );

//    Write_FIFO( &RX_APP, (u8)USART_ReceiveData ( USART2 ));
  }
}


/*  brief  receive data from uart4
 *  param  none
 *  retval none
 */


void UART4_IRQHandler( void ) //
{
  if(USART_GetITStatus( UART4, USART_IT_RXNE ) != RESET) {
    USART_ClearITPendingBit( UART4, USART_IT_RXNE );

  }
  if(USART_GetITStatus( UART4, USART_IT_TC ) != RESET) {
    USART_ClearITPendingBit( UART4, USART_IT_TC );
  }
}


/**
 * @brief  This function RTT_printf from usart1
 * @param  None
 * @retval None
 */
 
#include <stdarg.h>

#define CMD_BUFFER_LEN  ( (uint32_t)500 )
void USART1_printf( char *fmt, ... )
{
  char     buffer[CMD_BUFFER_LEN - 1];
  int      bak_sw_putchar;
  va_list  arg_ptr;                                      //Define convert parameters variable
  va_start( arg_ptr, fmt );                              //Init variable
  vsnprintf( buffer, CMD_BUFFER_LEN + 1, fmt, arg_ptr ); //parameters list format to buffer
  {
    bak_sw_putchar = sw_putchar;
    sw_putchar     = 1;
    RTT_printf( "%s",buffer );
    sw_putchar     = bak_sw_putchar;
  }
  va_end( arg_ptr );
}


/**
 * @brief  This function RTT_printf from usart2
 * @param  None
 * @retval None
 */

void USART2_printf( char *fmt, ... )
{
  char     buffer[CMD_BUFFER_LEN - 1];
  int      bak_sw_putchar;
  va_list  arg_ptr;                                      //Define convert parameters variable
  va_start( arg_ptr, fmt );                              //Init variable
  vsnprintf( buffer, CMD_BUFFER_LEN + 1, fmt, arg_ptr ); //parameters list format to buffer
  {
    bak_sw_putchar = sw_putchar;
    sw_putchar     = 2;
    RTT_printf( "%s",buffer );
    sw_putchar     = bak_sw_putchar;
  }
  va_end( arg_ptr );
}




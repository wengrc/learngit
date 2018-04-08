#ifndef __LIB_USART_H
#define __LIB_USART_H

#include <stdio.h>

#ifdef STM32F1XX
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#define  MAX_USART_NUM  5

#elif defined STM32F2XX
#include "stm32f2xx.h"
#include "stm32f2xx_gpio.h"
#include "stm32f2xx_usart.h"
#include "stm32f2xx_rcc.h"
#include "misc.h"
#define  MAX_USART_NUM  6

#elif defined STM32F4XX
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#define  MAX_USART_NUM  6

#endif

/*串口配置信息*/
struct com
{
//USART
  USART_TypeDef *   USARTx;
  uint32_t          RCC_AxBxPeriph_USARTx;
  uint32_t          RCC_AxBxPeriph_GPIOx_TX;
  uint32_t          RCC_AxBxPeriph_GPIOx_RX;

//TX GPIO
  GPIO_TypeDef *    GPIOx_TX;
  uint32_t          GPIO_Pin_x_TX;

//RX GPIO
  GPIO_TypeDef *    GPIOx_RX;
  uint32_t          GPIO_Pin_x_RX;

//USART INFO
  USART_InitTypeDef USART_InitStruct;

};


/*重定向串口输出为串口1 到 串口6  */
extern int  sw_putchar;

/*是否全映射串口3 */
extern u8   fullRemapForUsart3;


#ifdef __cplusplus
extern "C"{
#endif

/*串口配置 */
void  USART_Config(  struct com *COMx );

#ifdef __cplusplus
}
#endif

#endif /*_USART_H*/



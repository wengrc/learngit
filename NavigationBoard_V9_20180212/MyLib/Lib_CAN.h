
#ifndef __LIB_CAN_H
#define __LIB_CAN_H

#include <stdio.h>
#include "misc.h"

#ifdef STM32F1XX
#include "stm32f10x.h"
#include "stm32f10x_can.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#else
#include "stm32f4xx.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#endif

/*CANBUS 配置参数    */
struct CANBUSx
{
    CAN_TypeDef  *CANx;          //CAN模块选择
    IRQn_Type     CANx_RXx_IRQn;
    GPIO_TypeDef *GPIOx_TX;      //发射端口
    uint32_t      GPIO_Pin_x_TX; //发射脚位

    GPIO_TypeDef *GPIOx_RX;      //接收端口
    uint32_t      GPIO_Pin_x_RX; //接收脚位

    //波特率公式
    //CanFreq=Fpclk1/((1 + bs1tq + bs2tq)*prescaler)

    uint8_t  CAN_BS1_xtq;

    uint8_t  CAN_BS2_xtq;

    uint16_t CAN_Prescaler;

    //

};

//for TX_frame_MV
#define POS_CANID   3 /*can id 位置 */
#define POS_CANDLC  4 /*can 数据长度 */
#define POS_CANDT0  5
#define POS_CANDT1  6
#define POS_CANDT2  7
#define POS_CANDT3  8
#define POS_CANDT4  9
#define POS_CANDT5  10
#define POS_CANDT6  11
#define POS_CANDT7  12


/***********************************************************************
 * 旗标定义
 ************************************************************************/


/***********************************************************************
 * 全局变量声明
 ************************************************************************/

/***********************************************************************
 * 全局函数声明
 ************************************************************************/
/*TX测试*/ 
void  TestCan(CAN_TypeDef *CANx,
              u32 ID,
              u8 if_Extended);

/*CANBUS 配置 */
void  CAN_Config(struct CANBUSx *CANBUS);

/*CANBUS 是否开中断*/
void  CAN_NVIC_Config(IRQn_Type CANx_RXx_IRQn,
                      FunctionalState state);

#endif



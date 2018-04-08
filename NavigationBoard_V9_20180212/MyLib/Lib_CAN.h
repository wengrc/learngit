
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

/*CANBUS ���ò���    */
struct CANBUSx
{
    CAN_TypeDef  *CANx;          //CANģ��ѡ��
    IRQn_Type     CANx_RXx_IRQn;
    GPIO_TypeDef *GPIOx_TX;      //����˿�
    uint32_t      GPIO_Pin_x_TX; //�����λ

    GPIO_TypeDef *GPIOx_RX;      //���ն˿�
    uint32_t      GPIO_Pin_x_RX; //���ս�λ

    //�����ʹ�ʽ
    //CanFreq=Fpclk1/((1 + bs1tq + bs2tq)*prescaler)

    uint8_t  CAN_BS1_xtq;

    uint8_t  CAN_BS2_xtq;

    uint16_t CAN_Prescaler;

    //

};

//for TX_frame_MV
#define POS_CANID   3 /*can id λ�� */
#define POS_CANDLC  4 /*can ���ݳ��� */
#define POS_CANDT0  5
#define POS_CANDT1  6
#define POS_CANDT2  7
#define POS_CANDT3  8
#define POS_CANDT4  9
#define POS_CANDT5  10
#define POS_CANDT6  11
#define POS_CANDT7  12


/***********************************************************************
 * ��궨��
 ************************************************************************/


/***********************************************************************
 * ȫ�ֱ�������
 ************************************************************************/

/***********************************************************************
 * ȫ�ֺ�������
 ************************************************************************/
/*TX����*/ 
void  TestCan(CAN_TypeDef *CANx,
              u32 ID,
              u8 if_Extended);

/*CANBUS ���� */
void  CAN_Config(struct CANBUSx *CANBUS);

/*CANBUS �Ƿ��ж�*/
void  CAN_NVIC_Config(IRQn_Type CANx_RXx_IRQn,
                      FunctionalState state);

#endif



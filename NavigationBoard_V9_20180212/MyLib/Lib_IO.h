#ifndef __LIB_IO_H
#define __LIB_IO_H

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#ifdef STM32F1XX
#include "stm32f10x_gpio.h"
#elif defined STM32F2XX
#include "stm32f2xx_gpio.h"
#elif defined STM32F4XX
#include "stm32f4xx_gpio.h"
#endif

#define FOR_RELEASE  0

#if FOR_RELEASE  == -1 /*demo �׶�*/
#define USE_FULL_ASSERT
#elif FOR_RELEASE == 0 /*debug �׶�*/
#define USE_FULL_ASSERT
#elif FOR_RELEASE == 1 /*release �׶�*/
#endif


/************************************************************************
* �����
* V1��(unsigned char)����(uint8)
* V2��Χ(0~7)
************************************************************************/
#define set_flag(flag)   setf_ (flag)
#define setf_(V1, V2)    V1 |= (0X01 << V2)

/************************************************************************
* �����
* V1��(unsigned char)����(uint8)
* V2��Χ(0~7)
************************************************************************/
#define clr_flag(flag)   clrf_ (flag)
#define clrf_(V1, V2)    V1 &= ( (0X01 << V2) ^ 0XFF )

/************************************************************************
* �����
* V1��(unsigned char)����(uint8)
* V2��Χ(0~7)
************************************************************************/
#define read_flag(flag)  readf_ (flag)
#define readf_(V1, V2)   ( V1 & (0X01 << V2) )


/***********************************************************************
 * �Զ�������
 ************************************************************************/

//��λmask
typedef enum
{
    pin0    = 0x0000,
    pin1    = 0x0001,
    pin2    = 0x0002,
    pin3    = 0x0003,
    pin4    = 0x0004,
    pin5    = 0x0005,
    pin6    = 0x0006,
    pin7    = 0x0007,
    pin8    = 0x0008,
    pin9    = 0x0009,
    pin10   = 0x000a,
    pin11   = 0x000b,
    pin12   = 0x000c,
    pin13   = 0x000d,
    pin14   = 0x000e,
    pin15   = 0x000f,
    pinAll  = 0x00ff,
    PinNone = 0xeeee,

} PIN_TypeDef;

//KEY״̬
typedef enum
{
    KEY_Status_null  = 0x00,
    KEY_Status_short = 0x01,
    KEY_Status_long  = 0x02

} KEY_StatusType;

/*��SIZE�Ĳ�ͬ����Ҳ��ͬ*/

#define BUF_TYPE(_SIZE_) \
    struct { char buf[_SIZE_]; }


/***********************************************************************
 * IO����
 ************************************************************************/


/***********************************************************************
 * ��궨��
 ************************************************************************/


/***********************************************************************
 * ȫ�ֱ�������
 ************************************************************************/


/***********************************************************************
 * ȫ�ֺ�������
 ************************************************************************/
unsigned int  GPIOx_To_RCC_GPIOx(GPIO_TypeDef *GPIOx);

void          Config_IO(GPIO_TypeDef *GPIOx,
                        PIN_TypeDef PINy,
                        GPIO_InitTypeDef *GPIO_InitStruct);


void  PinOutput(GPIO_TypeDef *GPIOx /*choose Port*/,
                PIN_TypeDef PINy /*choose PINy*/,
                int level /*ctrl status*/);

void  PinToggle(GPIO_TypeDef *GPIOx,
                PIN_TypeDef PINy);

void  PinBlink(GPIO_TypeDef *GPIOx,
               PIN_TypeDef PINy,
               short N,
               char inv,
               volatile int *count);


int   ReadPinOutput(GPIO_TypeDef *GPIOx,
                    PIN_TypeDef PINy);

int   ReadPinInput(GPIO_TypeDef *GPIOx,
                   PIN_TypeDef PINy);

bool  ScanKey(GPIO_TypeDef *GPIOx,
              PIN_TypeDef PINy,
              bool criterion_for_ON /*ͨ·���жϹ���*/);


#endif



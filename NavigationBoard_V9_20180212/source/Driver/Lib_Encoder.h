#ifndef __LIB_ENCODER_H
#define __LIB_ENCODER_H

#ifdef STM32F4XX
#include "stm32f4xx.h"
#endif

#ifdef STM32F1XX
#include "stm32f10x.h"
#endif

enum
{
    TIM1_E = 0,
    TIM2_E = 1,
    TIM3_E = 2,
    TIM4_E = 3,
    TIM5_E = 4,
    TIM6_E = 5,
    TIM7_E = 6,
    TIM8_E = 7,
    TIM9_E = 8,
    TIM10_E = 9,
    TIM11_E = 10,
    TIM12_E = 11,
    TIM13_E = 12,
    TIM14_E = 13,
    TIM15_E = 14,
    TIM16_E = 15,
    TIM17_E = 16,

};


typedef struct 
{
    uint32_t TIMx_E;
    uint32_t overflow;
    uint32_t initValue;

} encoder_t;

void Encoder_Config(encoder_t *encoder_x);

int IsCountUp(encoder_t *encoder_x);

uint32_t EncoderGetCounter(encoder_t *encoder_x);



#endif









#include "Lib_Encoder.h"

#ifdef STM32F4XX
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#endif

#ifdef STM32F1XX
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#endif

static TIM_TypeDef *  timx[14] =
{
    TIM1,
    TIM2,
    TIM3,
    TIM4,
    TIM5,
    TIM6,
    TIM7,
    TIM8,
    TIM9,
    TIM10,
    TIM11,
    TIM12,
    TIM13,
    TIM14,
//    TIM15,
//    TIM16,
//    TIM17
};

static uint32_t rcc[] =
{
    RCC_APB2Periph_TIM1,
    RCC_APB1Periph_TIM2,
    RCC_APB1Periph_TIM3,
    RCC_APB1Periph_TIM4,
    RCC_APB1Periph_TIM5,
    RCC_APB1Periph_TIM6,
    RCC_APB1Periph_TIM7,
    RCC_APB2Periph_TIM8,
    RCC_APB2Periph_TIM9,
    RCC_APB2Periph_TIM10,
    RCC_APB2Periph_TIM11,
    RCC_APB1Periph_TIM12,
    RCC_APB1Periph_TIM13,
    RCC_APB1Periph_TIM14,
};


/**
 *  brief
 *  param
 *  retval
 */
#define TIM_ICPolarity_1 TIM_ICPolarity_Falling
#define TIM_ICPolarity_2 TIM_ICPolarity_Rising
void Encoder_Config(encoder_t *encoder_x)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef        TIM_ICInitStructure;

    //1
    TIM_DeInit(timx[encoder_x->TIMx_E]);
    if(encoder_x->TIMx_E == TIM1_E
        ||encoder_x->TIMx_E == TIM8_E
        ||encoder_x->TIMx_E == TIM9_E
        ||encoder_x->TIMx_E == TIM10_E
        ||encoder_x->TIMx_E == TIM11_E )
    {
        RCC_APB2PeriphClockCmd (rcc[encoder_x->TIMx_E], ENABLE);
    }
    else
    {
        RCC_APB1PeriphClockCmd (rcc[encoder_x->TIMx_E], ENABLE);
    }

    /* Timer configuration in Encoder mode */
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // No prescaling
    TIM_TimeBaseStructure.TIM_Period = encoder_x->overflow;
    TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit (timx[encoder_x->TIMx_E], &TIM_TimeBaseStructure);

    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
    //2
    TIM_ICInitStructure.TIM_Channel     = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_1;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter    = 0x06;

    TIM_ICInit (timx[encoder_x->TIMx_E], &TIM_ICInitStructure);//TIMx_CH1

    TIM_ICInitStructure.TIM_Channel     = TIM_Channel_2;
    TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_2;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter    = 0x06;

    TIM_ICInit (timx[encoder_x->TIMx_E], &TIM_ICInitStructure);//TIMx_CH2

    TIM_EncoderInterfaceConfig (timx[encoder_x->TIMx_E],
                                TIM_EncoderMode_TI12,
                                TIM_ICPolarity_1,
                                TIM_ICPolarity_2);

    // Clear all pending interrupts
//    TIM_ClearFlag (timx[encoder_x->TIMx_E], TIM_FLAG_Update);
//    TIM_ITConfig (timx[encoder_x->TIMx_E], TIM_IT_Update, ENABLE);

    //3//Reset counter
    timx[encoder_x->TIMx_E]->CNT = encoder_x->initValue;

    TIM_Cmd (timx[encoder_x->TIMx_E], ENABLE);
}


#define IS_COUNT_UP(TIM)  ( ( (TIM->CR1) & 0X010 ) == 0 )

int IsCountUp(encoder_t *encoder_x)
{
    return IS_COUNT_UP(timx[encoder_x->TIMx_E]);
}


uint32_t EncoderGetCounter(encoder_t *encoder_x)
{
    return TIM_GetCounter(timx[encoder_x->TIMx_E]);
}



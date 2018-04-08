
#include "ENCODER.h"
#include "stm32f10x_tim.h"
#include "misc.h"
#include "Lib_retarget_printf.h"
#include "clock.h"
#include "Lib_Encoder.h"

/* Private define ------------------------------------------------------------*/


int  encoderOverflow_cord = 0;


encoder_t  encoder_cord =
{
    TIM3_E,
    U16_MAX,
    0
};

void InitEncoderCord(void)
{

    NVIC_InitTypeDef  NVIC_INI;
    encoderOverflow_cord = 0;
    Encoder_Config (&encoder_cord);
    TIM_ClearFlag (TIM3, TIM_IT_Update);
    TIM_ITConfig (TIM3, TIM_IT_Update, ENABLE);
    NVIC_INI.NVIC_IRQChannel    = TIM3_IRQn;
    NVIC_INI.NVIC_IRQChannelCmd = ENABLE;
    NVIC_INI.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_INI.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init (&NVIC_INI);
}


void InitAllEncoder(void)
{
    InitEncoderCord ();
}


/*******************************************************************************
* Function Name  : TIM2_IRQHandler
* Description    : This function handles TIMx Update interrupt request.
*                   Encoder unit connected to TIM2
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM2_IRQHandler(void)
{
    /* Clear the interrupt pending flag */
    TIM_ClearITPendingBit (TIM2, TIM_FLAG_Update);
}


/*******************************************************************************
* Function Name  : TIM3_IRQHandler
* Description    : This function handles TIMx Update interrupt request.
*                   Encoder unit connected to TIM2
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM3_IRQHandler(void)
{
    /* Clear the interrupt pending flag */
    TIM_ClearITPendingBit (TIM3, TIM_FLAG_Update);

    if ( IsCountUp (&encoder_cord) )
    {
        if (encoderOverflow_cord < 100)
        {
            encoderOverflow_cord++;
        }

    }
    else
    {
        if (encoderOverflow_cord > -100)
        {
            encoderOverflow_cord--;
        }
    }
}


/*******************************************************************************
* Function Name  : TIM4_IRQHandler
* Description    : This function handles TIMx Update interrupt request.
*                   Encoder unit connected to TIM2
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM4_IRQHandler(void)
{
    /* Clear the interrupt pending flag */
    TIM_ClearITPendingBit (TIM4, TIM_FLAG_Update);

}



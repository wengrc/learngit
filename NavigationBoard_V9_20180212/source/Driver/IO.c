#include "IO.h"


/**
 * @brief  This function Init all IO
 * @param  None
 * @retval None
 */
void InitAllIO(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;

    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IPD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    Config_IO (ENC_CORD_A, &GPIO_InitStruct);
    Config_IO (ENC_CORD_B, &GPIO_InitStruct);
//    GPIO_PinAFConfig (ENC_CORD_A, GPIO_AF_TIM4);
//    GPIO_PinAFConfig (ENC_CORD_B, GPIO_AF_TIM4);

    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    Config_IO (CAN_TX, &GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IPU;	
    Config_IO (CAN_RX, &GPIO_InitStruct);
//    GPIO_PinAFConfig (CAN_TX, GPIO_AF_CAN1);
//    GPIO_PinAFConfig (CAN_RX, GPIO_AF_CAN1);


    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    Config_IO (O_CHARGE, &GPIO_InitStruct);
    Config_IO (O_BORDCARD_EN, &GPIO_InitStruct);
    Config_IO (O_SYS_CTRL_EN, &GPIO_InitStruct);


    
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    Config_IO (O_LED_HEART, &GPIO_InitStruct);


    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    Config_IO (I_TOP, &GPIO_InitStruct);
    Config_IO (I_BOTTOM, &GPIO_InitStruct);

}



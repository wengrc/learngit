#ifndef __ENCODER_H
#define __ENCODER_H
#include "IO.h"
#include "Lib_encoder.h"


#define U32_MAX           ( (u32)4294967295uL )
#define U16_MAX           ( (u16)65535 )

#define MAX_HIGHT         800
#define HIGHT_TRAY        126
#define MIN_HIGHT         0

void  InitAllEncoder(void);

void  TIM2_IRQHandler(void);
void  TIM3_IRQHandler(void);
void  TIM4_IRQHandler(void);

#endif



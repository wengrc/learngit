#ifndef __CLOCK_H
#define __CLOCK_H

#include "stm32f10x_rcc.h"
#define  PERIOD_BLOCK(delay /*ms*/, ptr_times, func) \
    do {                                             \
        u32 GetSysTick (void);                       \
        static u32  lastTick;                        \
        static u32  s = 0;                           \
        switch (s)                                   \
        {                                            \
        case 0:                                      \
            lastTick = GetSysTick ();                \
            s        = 1;                            \
        case 1:                                      \
            if (GetSysTick () - lastTick >= delay) { \
                s = 0;                               \
                if (*ptr_times != 0)                 \
                {                                    \
                    func;                            \
                }                                    \
                if (*ptr_times > 0) {                \
                    (*ptr_times)--;                  \
                }                                    \
            }                                        \
            break;                                   \
        default:                                     \
            s = 0;                                   \
        }                                            \
    }                                                \
    while (0)
void  Init_CLK(void);


u32   GetSysTick(void);

void  SetSysTick(u32 tick);

void  Delay_us(u32 nus);
void  Delay_ms(u32 nms);


#endif



#ifndef __IAP_H_
#define __IAP_H_

#include "stm32f10x.h"


extern u8  APP_CONFIG_SET_VALUE[];
extern u8  APP_CONFIG_CLEAR_VALUE[];

void AppJumptoIap( u32 iapxaddr );

#endif



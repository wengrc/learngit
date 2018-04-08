#ifndef __CAN_H
#define __CAN_H

#include "Lib_CAN.h"
#include "stm32f10x.h"
#define CAN_ES0 0x301
#define CAN_ES1 0x101
#define CAN_ES2 0x201
#define CAN_ES3 0x202
void InitAllCAN(void);
void ExtId_TX_CAN1( uint32_t ID,char *pbuf, int size );
void ExtId_TX_CAN1_frameHead(uint32_t id,
                             uint16_t crc,
                             int size);

void ExtId_TX_CAN1_frameTail(uint32_t id);

void RxLinux(void);

#endif



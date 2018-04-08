#ifndef CRC_H
#define CRC_H


#include "stm32f10x.h"

uint16_t CRC16(const char *ptr,
           uint8_t len); /* ptr为数据指针,len为数据长度 */

uint16_t GetCrc16(const  char *data, uint8_t len, uint16_t pwd);

#endif



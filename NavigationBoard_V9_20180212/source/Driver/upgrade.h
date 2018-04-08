#ifndef __UPGRADE_H
#define __UPGRADE_H

#define ES              1

#define STM32_FLASH_BASE   0x08000000                              //FLASH的起始地址
#define FLASH_APP1_ADDR    0x08006000                              //应用程序起始地址
#define FLASH_CONFIG_ADDR  (FLASH_APP1_ADDR + 0x1a000 - 0x800 + 4) //配置地址

#endif




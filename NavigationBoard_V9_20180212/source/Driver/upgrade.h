#ifndef __UPGRADE_H
#define __UPGRADE_H

#define ES              1

#define STM32_FLASH_BASE   0x08000000                              //FLASH����ʼ��ַ
#define FLASH_APP1_ADDR    0x08006000                              //Ӧ�ó�����ʼ��ַ
#define FLASH_CONFIG_ADDR  (FLASH_APP1_ADDR + 0x1a000 - 0x800 + 4) //���õ�ַ

#endif




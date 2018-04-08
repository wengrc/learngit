#ifndef __LIB_DMA_H
#define __LIB_DMA_H

#include "stm32f4xx_dma.h"

/*可选的 DMA*/
enum
{
    DMA1_e = 0,
    DMA2_e = 1,
};

/*可选的 stream  */
enum
{
    STREAM0_e = 0,
    STREAM1_e = 1,
    STREAM2_e = 2,
    STREAM3_e = 3,
    STREAM4_e = 4,
    STREAM5_e = 5,
    STREAM6_e = 6,
    STREAM7_e = 7
};

/*可选的 channel*/
enum
{
    CHANNEL0_e = 0,
    CHANNEL1_e = 1,
    CHANNEL2_e = 2,
    CHANNEL3_e = 3,
    CHANNEL4_e = 4,
    CHANNEL5_e = 5,
    CHANNEL6_e = 6,
    CHANNEL7_e = 7

};


/*DMA 配置信息*/
typedef struct
{
    uint8_t  DMA_e;
    uint8_t  STREAM_e;
    uint8_t  CHANNEL_e;
    void    *periph;
    void    *memory;
    uint32_t DMA_DIR_x;
    uint32_t bufsize;

    uint32_t DMA_IT_x; /*DMA_IT_TC*/
    FunctionalState if_ENABLE;

}DMA_t;

/*DMA配置  */
int DMA_Config(DMA_t *dma,
                 DMA_InitTypeDef *DMA_InitStructure);

#endif



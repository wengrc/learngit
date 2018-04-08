
#ifndef __LIB_FIFO_H
#define __LIB_FIFO_H

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>


/***********************************************************************
 * 自定义类型
 ************************************************************************/

//定义FIFO
#define MAX_FIFO_LGTH  247

typedef struct
{
    short begin_pos; //非常重要: 如果大于MAX_FIFO_LGTH读写FIFO溢出 !! 建议初始化为零
    short end_pos;   //非常重要: 如果大于MAX_FIFO_LGTH读写FIFO溢出 !! 建议初始化为零
    short size;
    char *buf;

}fifo;

//定义帧格式
#define MAX_FRAME_LGTH        150
#define FRAME_STATUS_EMPTY    0
#define FRAME_STATUS_LOADING  1
#define FRAME_STATUS_FINISH   2

typedef struct            //发射帧一定要初始化为零
{
    short lgth;           //既是长度也是buf的写索引
    unsigned char status; //0:空帧,等待填充命令头，1等待填充命令体和命令尾 2完成一帧
    short size;
    char *buf;

}frame;


typedef struct
{
    unsigned char size;
    unsigned char free;
    unsigned char used;
    short *buf;

}data_list;


/***********************************************************************
 * 旗标定义
 ************************************************************************/

/***********************************************************************
 * 全局变量声明
 ************************************************************************/

/***********************************************************************
 * 全局函数声明
 ************************************************************************/


#ifdef __cplusplus
extern "C" {
#endif

/*创建循环FIFO。FIFO指定缓冲区名字, buf指定缓冲区首地址，size指定大小*/
void  CreateFIFO(fifo *FIFO,
                 char *buf,
                 short size);

/*创建数据帧。pframe存储帧信息，buf绑定缓存，size指定buf大小  */
void  CreateFrame(frame *pframe,
                  char *buf,
                  short size);

/*写 FIFO , 每次写一个字节 */
void  Write_FIFO(fifo *FIFO,
                 unsigned char ch);

/*读FIFO , 每次读一个字节 ，readPos维护每一次读完后的位置，类似书签功能一次都是
    从上次的位置开始读起。readPos初始化为FIFO相对起始地址
    */
char* Read_FIFO(fifo *FIFO,
                short *readPos);

/*计算FIFO长度，从当前位置curpos算起 */
uint16_t  FIFOLength(fifo *FIFO,
                     short curpos);

/*从当前位置算起，计算FIFO剩余空间  */
uint16_t  FIFORemaind(fifo *FIFO,
                      short curpos);

/*从FIFO里抓取一帧数据，readPos从上次读取位置开始，beginStr 帧的头部匹配，endStr帧的尾部匹配，
  pframe 帧的存放地址
*/
void  FetchFrame_BE(fifo *FIFO,
                    short *readPos,
                    const char *beginStr,
                    const char *endStr,
                    frame *pframe);

/*从FIFO里抓取一帧数据，readPos从上次读取的位置开始，beginStr开始匹配头部的特征，
Pos_dataLgth长度存储位置，invariantLgth扣除数据部分的固定长度，pframe帧数据存储的位置

*/
void  FetchFrame_BL(fifo *FIFO,
                    short *readPos,
                    const char *beginStr,
                    unsigned char Pos_dataLgth,
                    unsigned char invariantLgth,
                    frame *pframe);


/* 将发射帧移入发射FIFO, 由发射FIFO重新提取后自动发射数据

*/
void  Mov_TxFrame_to_TxFifo(const frame *TX_frame_x,
                            fifo *TX_FIFO_x);


#ifdef __cplusplus
}
#endif

#endif



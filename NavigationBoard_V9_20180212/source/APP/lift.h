
#ifndef __LIFT_H
#define __LIFT_H

#include "stm32f10x.h"

/*重新复位高度到最低的位置*/
void  Reinit_Hight(void);

/*上升到相对高度，value的位置*/
//u8 Up(u8 up_speed,u16 up_hight);

/*下降到相对高度，value的位置*/
u8 Down(u8 down_speed,u16 down_hight);

/*获得当前高度*/
int   GetHight(void);

/*到达绝对高度，h单位根据拉绳的脉冲长度而有所区别*/
//u8    TargetHight(int h);

/*发送当前高度*/
void  TxHight(void);

/*停止升降*/
u8    StopLift(void);

/*测试拉绳编码器*/
void  TestEncoder_cord(void);

//void TxMaxhight(void);

int IsLiftBottom(void);

int IsLiftTop(void);

void FeedBackLoop();

#endif



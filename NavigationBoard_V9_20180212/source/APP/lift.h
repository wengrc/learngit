
#ifndef __LIFT_H
#define __LIFT_H

#include "stm32f10x.h"

/*���¸�λ�߶ȵ���͵�λ��*/
void  Reinit_Hight(void);

/*��������Ը߶ȣ�value��λ��*/
//u8 Up(u8 up_speed,u16 up_hight);

/*�½�����Ը߶ȣ�value��λ��*/
u8 Down(u8 down_speed,u16 down_hight);

/*��õ�ǰ�߶�*/
int   GetHight(void);

/*������Ը߶ȣ�h��λ�������������峤�ȶ���������*/
//u8    TargetHight(int h);

/*���͵�ǰ�߶�*/
void  TxHight(void);

/*ֹͣ����*/
u8    StopLift(void);

/*��������������*/
void  TestEncoder_cord(void);

//void TxMaxhight(void);

int IsLiftBottom(void);

int IsLiftTop(void);

void FeedBackLoop();

#endif



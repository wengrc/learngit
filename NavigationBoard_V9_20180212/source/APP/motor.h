
#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"
#include "can.h"

#define CANID_FROM_MAIN      0x212 //来自主板的控制消息
#define CANID_TO_MAIN        0x202 //给主板的反馈消息
#define CANID_FROM_CONTROL   0x203 //来自控制器的反馈消息
#define CANID_TO_CONTROL     0x213 //给控制器的消息
#define CANID_FROM_FAULT     0x207 //来自控制器的故障消息
#define CANID_LIFT_CONTROL   0x814 //叉臂控制
#define CANID_LIFT_FEEDBACK  0x804 //叉臂反馈
#define CANID_FROM_BUTTON    0x401 //来自按键的消息
#define CANID_CHARGE         0X417

typedef struct
{
    int   enable;     //使能
    int   direction;  //行进方向
    short angle;      //行进角度
    short speed;      //行进速度
    int   brake;      //刹车
    int   distance;   //行驶距离
    int   mode;       //控制模式

    u8    walk_fault; //行走故障
    u8    turn_fault; //转向故障

}motor_info_s;

extern  /*static*/ motor_info_s  g_motor_control;      //来自控制器的消息

int motor_init(void);
void message_save(CanRxMsg *canRxMsg);
void message_process(void);
void lift_down(void);
void lift_stop(void);
void lift_check(void);
u8 motor_mode_get(void);

#endif



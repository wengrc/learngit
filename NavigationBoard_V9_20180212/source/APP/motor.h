
#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"
#include "can.h"

#define CANID_FROM_MAIN      0x212 //��������Ŀ�����Ϣ
#define CANID_TO_MAIN        0x202 //������ķ�����Ϣ
#define CANID_FROM_CONTROL   0x203 //���Կ������ķ�����Ϣ
#define CANID_TO_CONTROL     0x213 //������������Ϣ
#define CANID_FROM_FAULT     0x207 //���Կ������Ĺ�����Ϣ
#define CANID_LIFT_CONTROL   0x814 //��ۿ���
#define CANID_LIFT_FEEDBACK  0x804 //��۷���
#define CANID_FROM_BUTTON    0x401 //���԰�������Ϣ
#define CANID_CHARGE         0X417

typedef struct
{
    int   enable;     //ʹ��
    int   direction;  //�н�����
    short angle;      //�н��Ƕ�
    short speed;      //�н��ٶ�
    int   brake;      //ɲ��
    int   distance;   //��ʻ����
    int   mode;       //����ģʽ

    u8    walk_fault; //���߹���
    u8    turn_fault; //ת�����

}motor_info_s;

extern  /*static*/ motor_info_s  g_motor_control;      //���Կ���������Ϣ

int motor_init(void);
void message_save(CanRxMsg *canRxMsg);
void message_process(void);
void lift_down(void);
void lift_stop(void);
void lift_check(void);
u8 motor_mode_get(void);

#endif




#include "string.h"
#include "motor.h"
#include "clock.h"
#include "lift.h"
#include "math.h"
#include "lib_retarget_printf.h"
#include "NAV350.h"

//#define EOUT RTT_printf
//#define DOUT RTT_printf
#define DOUT(...)
#define EOUT(...)
#define CAN_RX_MSG_LGTH      5
#define SPEED_TO_RPM         420


#define MAX_ANGLE            9000
#define MAX_SPEED            3000
#define MAX_DISTANCE         0x8000
#define LIFT_OFFSET_UP       80
#define LIFT_OFFSET_DOWN     10


enum
{
    MOTOR_DISABLE,
    MOTOR_ENABLE

};

enum
{
    MODE_MANUAL,
    MODE_CAN

};

enum
{
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
    MOTOR_NULL

};

enum
{
    BRAKE_DISABLE,
    BRAKE_ENABLE

};

enum
{
    LIFT_IDLE,
    LIFT_UP,
    LIFT_DOWN

};



typedef struct
{
    u8  status;
    u8  speed;
    u16 hight;

}lift_info_s;

typedef struct
{
    unsigned char front;                       //队列头索引
    unsigned char rear;                        //队列尾索引
    CanRxMsg      can_rx_msg[CAN_RX_MSG_LGTH]; //数组

}QueueTypeDef;


/*static*/ motor_info_s  g_motor_info;         //来自主板的消息
/*static*/ motor_info_s  g_motor_control;      //来自控制器的消息
static QueueTypeDef      g_msg_queue;          //来自主板的消息
static QueueTypeDef      g_msg_control;        //来自控制器的消息
static QueueTypeDef      g_msg_button;         //来自按键的消息
static lift_info_s       g_lift_info;          //叉臂控制消息


static int LiftTopTouchFlag = 0;//叉车手动上限位标志

void  motor_send(void);

int motor_init(void)
{
    memset ( &g_motor_info, 0, sizeof(motor_info_s) );
    memset ( &g_motor_control, 0, sizeof(motor_info_s) );
    memset ( &g_msg_queue, 0, sizeof(QueueTypeDef) );
    memset ( &g_msg_control, 0, sizeof(QueueTypeDef) );
    memset ( &g_msg_button, 0, sizeof(QueueTypeDef) );
    memset ( &g_lift_info, 0, sizeof(lift_info_s) );

    g_motor_info.direction = MOTOR_NULL;

    motor_send ();

    return 1;
}


short speed_to_rpm(short speed)
{
    short  rpm = 0;

    rpm = speed * 3.6 * SPEED_TO_RPM / 1000;

    return rpm;
}


short rpm_to_speed(short rpm)
{
    short  speed = 0;

    speed = rpm * 1000 / (3.6 * SPEED_TO_RPM);

    return speed;
}


void motor_send(void)
{
    char  data[8] = { 0 };
    int   rpm     = 0;
    motor_info_s *motor_info = 0;
    lift_info_s  *lift_info  = 0;

    motor_info = &g_motor_info;
    lift_info  = &g_lift_info;
    memset ( data, 0, sizeof(data) );

    if (motor_info->direction == MOTOR_FORWARD)
    {
        data[0] |= 0x01;
        data[0] &= 0xfd;
    }
    else if (motor_info->direction == MOTOR_BACKWARD)
    {
        data[0] |= 0x02;
        data[0] &= 0xfe;
    }
    else
    {
        EOUT ("motor direction unknown\n");
        data[0] |= 0x01;
        data[0] &= 0xfd;
    }

    if (motor_info->enable == MOTOR_DISABLE)
    {
        data[0] &= 0xfb;
    }
    else if (motor_info->enable == MOTOR_ENABLE)
    {
        data[0] |= 0x04;
    }
    else
    {
        EOUT ("motor enable unknown\n");
        data[0] &= 0xfb;
    }

    if (motor_info->mode == MODE_MANUAL)
    {
        data[0] &= 0xf7;
    }
    else if (motor_info->mode == MODE_CAN)
    {
        data[0] |= 0x08;
    }
    else
    {
        EOUT ("motor mode unknown\n");
        data[0] &= 0xf7;
    }

    if (motor_info->brake == BRAKE_DISABLE)
    {
        data[0] &= 0xef;
    }
    else if (motor_info->brake == BRAKE_ENABLE)
    {
        data[0] |= 0x10;
    }
    else
    {
        EOUT ("motor brake unknown\n");
        data[0] |= 0x10;
    }

    data[0] &= 0x3f;

    if (lift_info->status == LIFT_UP)
    {
        data[0] |= 0x40;
    }
    else if (lift_info->status == LIFT_DOWN)
    {
        data[0] |= 0x80;
    }
    else
    {
        data[0] &= 0x3f;
    }

    if (motor_info->angle < -MAX_ANGLE || motor_info->angle > MAX_ANGLE)
    {
        EOUT ("motor angle unknown\n");

        if (motor_info->angle < -MAX_ANGLE)
        {
            motor_info->angle = -MAX_ANGLE;
        }
        else
        {
            motor_info->angle = MAX_ANGLE;
        }
    }
    data[1] = (motor_info->angle & 0xff00) >> 8;
    data[2] = motor_info->angle & 0xff;

    if (motor_info->speed < 0 || motor_info->speed > MAX_SPEED)
    {
        EOUT ("motor speed unknown\n");

        if (motor_info->speed < 0)
        {
            motor_info->speed = 0;
        }
        else
        {
            motor_info->speed = MAX_SPEED;
        }
    }
    rpm     = speed_to_rpm (motor_info->speed);
    data[3] = (rpm & 0xff00) >> 8;
    data[4] = rpm & 0xff;

    DOUT ("%x,%x,%x,%x,%x\n", data[0], data[1], data[2], data[3], data[4]);
		{
			int i = 0,j = 0;
			static char lastData[8];
			for(i = 0;i<5;i++)
			{
					if(lastData[i] != data[i])
					{
						for(j = 0;j<5;j++)
						{
							lastData[j] = data[j];
						}
						ExtId_TX_CAN1 (CANID_TO_CONTROL, data, 5);
						break;
					}
			}
		}
}


void motor_enable(int enable)
{
    if (enable == MOTOR_DISABLE)
    {
        g_motor_info.speed = 0;
    }
    g_motor_info.enable = enable;
    motor_send ();
}


void motor_mode(int mode)
{
    if (g_motor_info.mode != mode)
    {
        g_motor_info.mode = mode;
        motor_send ();
    }
}


void motor_direction(int direction)
{
    g_motor_info.direction = direction;

    if (g_motor_info.enable == MOTOR_ENABLE)
    {
        motor_send ();
    }
}


void motor_angle(short angle)
{
    g_motor_info.angle = angle;

    if (g_motor_info.enable == MOTOR_ENABLE)
    {
        motor_send ();
    }
}


void motor_speed(short speed)
{
    g_motor_info.speed = speed;

    if (g_motor_info.enable == MOTOR_ENABLE)
    {
        motor_send ();
    }
}


void motor_brake(int brake)
{
    if (brake == BRAKE_ENABLE)
    {
        g_motor_info.speed = 0;
    }

    if (g_motor_info.mode == MODE_CAN)
    {
        g_motor_info.brake = brake;
        motor_send ();
    }
}

void motor_reset_distance(void)
{
    g_motor_control.distance = 0;
}

void motor_heatbeat(int ID)
{
		static int count;
    char  data[8] = { 0 };
    data[0]    = 0xFF;
		data[1]    = 0xFF;
		count++;
		if(count > 3)
		{
			count = 0;
			ExtId_TX_CAN1 (ID, data, 2);
		}
}


void motor_message_from_can(CanRxMsg *canRxMsg,
                            QueueTypeDef *msg_queue)
{
    if (!canRxMsg)
    {
        EOUT ("can message is NULL\n");
    }

    if (!msg_queue)
    {
        EOUT ("msg queue is NULL\n");
    }

    if ( (msg_queue->rear + 1) % CAN_RX_MSG_LGTH != msg_queue->front )
    {
        msg_queue->can_rx_msg[msg_queue->rear] = *canRxMsg;
        msg_queue->rear = (msg_queue->rear + 1) % CAN_RX_MSG_LGTH;
    }
    else
    {
        EOUT ("motor message from can too more\n");
    }
}

u8 isCharge = 0;
void ReportBattery(u8 * buf)
{
    char  data[8] = { 0 };
    unsigned int voltage = 0;
    voltage = buf[0]<<8;
    voltage |= buf[1];
    voltage = voltage * 10;
		data[0]    = 0x01;
    data[1]    = 0x01;
    data[2]    = (voltage&0xff00) >> 8;
    data[3]    = voltage & 0xff;
    data[4]    = isCharge;
    ExtId_TX_CAN1 (0x407, data, 5);
}


void message_save(CanRxMsg *canRxMsg)
{
    if (!canRxMsg)
    {
        EOUT ("can message is NULL\n");

        return;
    }

    switch (canRxMsg->ExtId)
    {
    case CANID_FROM_CONTROL:
    case CANID_FROM_FAULT:
        motor_message_from_can (canRxMsg, &g_msg_control);
        break;

    case CANID_FROM_MAIN:
    case CANID_LIFT_CONTROL:
    case CANID_CHARGE:
        motor_message_from_can (canRxMsg, &g_msg_queue);
        break;

    case CANID_FROM_BUTTON:
        motor_message_from_can (canRxMsg, &g_msg_button);
        break;

    default:
        break;
    }
}


void motor_control(u8 *data)
{
    if (!data)
    {
        EOUT ("control data is NULL\n");
    }

    switch (data[0])
    {
    case 0x11:
        motor_enable (data[1]);
        break;

    case 0x12:
        motor_direction (data[1]);
        break;

    case 0x13:
        motor_angle (data[1] << 8 | data[2]);
        break;

    case 0x14:
        motor_speed (data[1] << 8 | data[2]);
        break;

    case 0x15:
        motor_brake (data[1]);
        break;

    case 0x16:
        motor_reset_distance ();
        break;
		
		case 0xff:
        motor_heatbeat (CANID_TO_MAIN);
        break;
    }
}


void lift_run(u8 speed,
              u16 height)
{
    int  cur_lift_height   = 0;
    lift_info_s *lift_info = 0;

    lift_info = &g_lift_info;
    cur_lift_height = GetHight ();

    if ( cur_lift_height > height && !IsLiftBottom () )
    {
        lift_info->status = LIFT_DOWN;
    }
    else if ( cur_lift_height < height && !IsLiftTop () )
    {
        lift_info->status = LIFT_UP;
    }
    else
    {
        lift_info->status = LIFT_IDLE;
    }
    lift_info->hight = height;
    lift_info->speed = speed;
    motor_send ();
}


void lift_stop(void)
{
    lift_info_s *lift_info = 0;

    lift_info = &g_lift_info;
    lift_info->status = LIFT_IDLE;

    motor_send ();
}


void lift_down(void)
{
		int lastHight = 0;
		int hight = 0;
		int count = 0;
		static int limiteError = 0;
    lift_info_s  *lift_info  = 0;
    motor_info_s *motor_info = 0;
    u8  old_mode = MODE_MANUAL;

    lift_info  = &g_lift_info;
    motor_info = &g_motor_info;
    lift_info->status = LIFT_DOWN;

    old_mode = motor_info->mode;
    motor_info->mode = MODE_CAN;
    motor_send ();

    /*while(1){
     *  RTT_printf("%d,%d\n",IsLiftBottom(),IsLiftTop());
     *  Delay_ms(1000);
     *  }*/
    while ( !IsLiftBottom () && limiteError == 0)
		{		
				hight = GetHight();
				if(fabs(lastHight-hight) < 3)
				{ 
						count ++;
						if(count > 30)
						{		
								limiteError = 1;
								{
										char buf[8];
										buf[0] = 0;
										buf[1] = 0;
										buf[2] = 0xff;
										buf[3] = 0;
										ExtId_TX_CAN1(CANID_LIFT_FEEDBACK,buf,4);
								}
							  //send can mes;
								break;
						}
				}
				lastHight = hight;
				Delay_ms (100);
		}
    motor_info->mode = old_mode;
    Delay_ms (3000);
    lift_stop ();
}


void lift_control(u8 *data)
{
    if (!data)
    {
        EOUT ("control data is NULL\n");
    }

    switch (data[0])
    {
    case 0x11: //上升
        lift_run (data[1], data[2] << 8 | data[3]);
        break;

    case 0x12: //下降
        lift_run (data[1], data[2] << 8 | data[3]);
        break;

    case 0x13: //停止
        lift_stop ();
        break;
//		case 0xff:
//        motor_heatbeat (CANID_LIFT_FEEDBACK);
//        break;
    }
}


void lift_check(void)
{
    lift_info_s *lift_info = 0;
    int  cur_lift_height   = 0;

    lift_info = &g_lift_info;
    cur_lift_height = GetHight ();
	 
	  
	  if(LiftTopTouchFlag == 0 && IsLiftTop() && g_motor_info.mode == MODE_MANUAL)
		{
				motor_mode(MODE_CAN);
				motor_brake(MOTOR_DISABLE);
				lift_stop();
			  LiftTopTouchFlag = 5;
		}
		
		else if(LiftTopTouchFlag == 5 && IsLiftTop() && g_motor_info.mode == MODE_CAN)
		{
			LiftTopTouchFlag--;
			lift_run(30,cur_lift_height-50);
		}

	
    if (lift_info->status == LIFT_IDLE)
    {
        return;
    }

    if ( cur_lift_height <= lift_info->hight && IsLiftTop () )
    {
        lift_stop ();
    }

    if ( cur_lift_height >= lift_info->hight && IsLiftBottom () )
    {
        lift_stop ();
    }

    if ((lift_info->status == LIFT_UP) && (cur_lift_height + LIFT_OFFSET_UP >= lift_info->hight))
    {
        lift_stop ();
    }

    if ((lift_info->status == LIFT_DOWN) && (cur_lift_height <= lift_info->hight + LIFT_OFFSET_DOWN))
    {
        if (lift_info->hight != 0)
        {
            lift_stop ();
            SHOW_VOID (lift_stop);
        }
        else
        {
            if (cur_lift_height <= 8)
            {
                lift_stop ();
                SHOW_VOID (lift_stop);
            }
        }
    }
    motor_send ();
}

#include "IO.h"
void SetCharge(u8 * buf)
{
    if(buf[0] == 0x11)
    {
        if(buf[1] == 1)
        {
            PinOutput(O_CHARGE,1);
            isCharge = 1;
        }
        else
        {
            PinOutput(O_CHARGE,0);
            isCharge = 0;
        }
    }
}
void motor_feedback_send(void)
{
    char  data[8] = { 0 };
    motor_info_s *motor_info = 0;

    motor_info = &g_motor_control;
    memset ( data, 0, sizeof(data) );
    data[0]    = 0x01;

    if (motor_info->direction == MOTOR_FORWARD)
    {
        data[1] = 0x01;
    }
    else if (motor_info->direction == MOTOR_BACKWARD)
    {
        data[1] = 0x02;
    }
    else
    {
        data[1] = 0x00;
    }
#if 0

    if (motor_info->angle < -MAX_ANGLE || motor_info->angle > MAX_ANGLE)
    {
        EOUT ("motor angle unknown\n");

        if (motor_info->angle < -MAX_ANGLE)
        {
            motor_info->angle = -MAX_ANGLE;
        }
        else
        {
            motor_info->angle = MAX_ANGLE;
        }
    }
#endif
    data[2] = (motor_info->angle & 0xff00) >> 8;
    data[3] = motor_info->angle & 0xff;
#if 0

    if (motor_info->speed < -MAX_SPEED || motor_info->speed > MAX_SPEED)
    {
        EOUT ("motor speed unknown\n");

        if (motor_info->speed < -MAX_SPEED)
        {
            motor_info->speed = -MAX_SPEED;
        }
        else
        {
            motor_info->speed = MAX_SPEED;
        }
    }
#endif
    data[4] = (motor_info->speed & 0xff00) >> 8;
    data[5] = motor_info->speed & 0xff;

    if (motor_info->distance < -MAX_DISTANCE || motor_info->distance > MAX_DISTANCE)
    {
        EOUT ("motor distance unknown\n");

        if (motor_info->distance < -MAX_DISTANCE)
        {
            motor_info->distance = MAX_DISTANCE;
        }
        else
        {
            motor_info->distance = MAX_DISTANCE - 1;
        }
    }
    data[6] = (motor_info->distance & 0xff00) >> 8;
    data[7] = motor_info->distance & 0xff;

		
		
		
    ExtId_TX_CAN1 (CANID_TO_MAIN, data, 8);
}


void motor_feedback(u8 *data)
{
    int  distance = 0;
    static int lasttick = 0;
    int  timedec  = 0;
    motor_info_s *motor_info = 0;

    motor_info = &g_motor_control;

    if (data[0] & 0x01)
    {
        motor_info->direction = MOTOR_FORWARD;
    }
    else if (data[0] & 0x02)
    {
        motor_info->direction = MOTOR_BACKWARD;
    }
    else
    {
        motor_info->direction = MOTOR_NULL;
    }

    motor_info->angle = data[1] << 8 | data[2];
    motor_info->speed = rpm_to_speed (data[3] << 8 | data[4]);

    if (lasttick == 0)
    {
        lasttick = GetSysTick ();
        distance = 0;
    }
    else
    {
        timedec  = GetSysTick () - lasttick;
        lasttick = GetSysTick ();
        distance = motor_info->speed * timedec / 1000;
    }
    DOUT ("%d,%d,%d\n", timedec, motor_info->speed, distance);
		
		//SendCurrPoseu(1);
		
    motor_info->distance +=  distance;
		
		
    
		motor_feedback_send ();
	  SendCurrPoseu(1);
}


void motor_fault_send(void)
{
    char  data[8] = { 0 };
    motor_info_s *motor_info = 0;

    motor_info = &g_motor_control;
    memset ( data, 0, sizeof(data) );
    data[0]    = 0x02;
    data[1]    = motor_info->walk_fault;
    data[2]    = motor_info->turn_fault;

    ExtId_TX_CAN1 (CANID_TO_MAIN, data, 3);
}


void motor_fault(u8 *data)
{
    motor_info_s *motor_info = 0;

    motor_info = &g_motor_control;
    motor_info->walk_fault = data[0];
    motor_info->turn_fault = data[1];
		if(motor_info->walk_fault || motor_info->turn_fault)
		{
				motor_brake(1);
		}
    motor_fault_send ();
}


void button_process(u8 *data)
{
    if (data[2] == 1)
    {
        motor_enable (MOTOR_ENABLE);
    }
    else if (data[2] == 2)
    {
        motor_enable (MOTOR_DISABLE);
    }

    if (data[3] == 1)      //自动
    {
        motor_mode (MODE_CAN);
    }
    else if (data[3] == 2) //手动
    {
			  if(LiftTopTouchFlag > 0)
				{
					LiftTopTouchFlag--;
				} 
			  if(LiftTopTouchFlag == 0)
				{
				  motor_brake (MOTOR_DISABLE);
					motor_mode (MODE_MANUAL);
					g_lift_info.hight = GetHight ();	
				}
    }
}


void message_process(void)
{
    QueueTypeDef *queue = 0;
    CanRxMsg     *msg   = 0;
		static int lastTick = 0;
		
		if(GetSysTick() - lastTick > 500)
    {
				lastTick = GetSysTick();
				motor_speed(0);
				motor_brake(1);
    }
    queue = &g_msg_queue;

    if (queue->rear != queue->front)
    {
        msg = &(queue->can_rx_msg[queue->front]);

        switch (msg->ExtId)
        {
        case CANID_FROM_MAIN:
						lastTick = GetSysTick();
            motor_control (msg->Data);
            break;

        case CANID_LIFT_CONTROL:
            lift_control (msg->Data);
            break;
        case CANID_CHARGE:
            SetCharge(msg->Data);
            break;
        default:
            break;
        }

        queue->front = (queue->front + 1) % CAN_RX_MSG_LGTH;
    }

    queue = &g_msg_control;

    if (queue->rear != queue->front)
    {
        msg = &(queue->can_rx_msg[queue->front]);

        switch (msg->ExtId)
        {
        case CANID_FROM_CONTROL:
            motor_feedback (msg->Data);
            break;

        case CANID_FROM_FAULT:
            motor_fault (msg->Data);
            ReportBattery(&msg->Data[4]);
            break;

        default:
            break;
        }

        queue->front = (queue->front + 1) % CAN_RX_MSG_LGTH;
    }

    queue = &g_msg_button;

    if (queue->rear != queue->front)
    {
        msg = &(queue->can_rx_msg[queue->front]);

        switch (msg->ExtId)
        {
        case CANID_FROM_BUTTON:
            button_process (msg->Data);
            break;

        default:
            break;
        }

        queue->front = (queue->front + 1) % CAN_RX_MSG_LGTH;
    }
}


u8 motor_mode_get(void)
{
    return g_motor_info.mode;
}



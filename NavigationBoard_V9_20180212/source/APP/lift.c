
#include "lift.h"
#include "CAN.h"
#include "Lib_retarget_printf.h"
#include "IO.h"
#include "encoder.h"
#include "Lib_encoder.h"
#include "clock.h"
#include "motor.h"
extern encoder_t encoder_cord;
extern int encoderOverflow_cord;



void Reinit_Hight(void)
{
		InitAllEncoder();
    lift_down();
		InitAllEncoder();
}

void FeedBackLoop()
{
	int i = 0;
	char  buf[8] = { 0 };
	buf[i++] = 0x03;
	buf[i++] = 0xE8;
	buf[i++] = 0x00;
	buf[i++] = 0x0A;
	
	ExtId_TX_CAN1 (0x217, buf, i);
}

/**
 *  brief
 *  param
 *  retval
 */
int GetHight(void)
{
    
    const int CORD_DIV = 4;
    int  height = 0;
    height = EncoderGetCounter(&encoder_cord);
//    SHOW_INT(height);
    height+= (int)encoderOverflow_cord * (int)(U16_MAX + 1);
    return height/CORD_DIV;
}


/**
 *  brief
 *  param
 *  retval
 */
int IsLiftBottom(void)
{
    return (0==ReadPinInput(I_BOTTOM));
}

/**
 *  brief
 *  param
 *  retval
 */
int IsLiftTop(void)
{
    return (0==ReadPinInput(I_TOP));
}


void TxHight(void)//protocol b4
{
    #define SIZE 5
    static u32 lastTick;
    char  buf[SIZE] = { 0 };
    int hight = 0;

	  lift_check();
		
    if(GetSysTick() - lastTick < 1000)
    {
        return;
    }
    hight = GetHight();
    lastTick = GetSysTick();
    
    buf[0] = 0x01;
    buf[1] = (hight>>8)&0xff;
    buf[2] = hight&0xff;
    if(ReadPinInput(I_BOTTOM) == 0)
    {
        buf[3] = 1;
    }
    else if(ReadPinInput(I_TOP) == 0)
    {
        buf[3] = 2;
    }
    else 
    {
        buf[3] = 0;
    }
    buf[4] = 0;
    ExtId_TX_CAN1 (CANID_LIFT_FEEDBACK, buf, SIZE);
		{
			char  data[8] = { 0 };
			data[0]    = 0xFF;
			data[1]    = 0xFF;
			ExtId_TX_CAN1 (CANID_LIFT_FEEDBACK, data, 2);
		}
    #undef SIZE
}

u8 StopLift(void)
{
    char buf[8];
    buf[0] = 0x04;
    ExtId_TX_CAN1(0x405,buf,1);
    return 1;
}



//u8 LiftSecure(void)
//{
//    static u32 lastTick;
//    static int hight = 0;
//    if(GetHight() -  hight > 20)
//    {
//        lastTick = GetSysTick();
//        hight = GetHight();
//    }
//    if(GetSysTick() - lastTick >  500)
//    {
//        StopLift();
//        return 1;
//    }
//    return 0;
//}


/*******************************************************************************
* Function Name  : up(u16 value)
* Description    :
* Input          : None
* Output         : None
* Return         : High:
*******************************************************************************/



//u8 Up(u8 up_speed,u16 up_hight)
//{
//    char buf[8];
//    buf[0] = 0x02;
//    buf[1] = up_speed;
//    buf[2] = (up_hight >> 8)&0xff;
//    buf[3] = up_hight & 0xff;
//    ExtId_TX_CAN1(0x405£¬buf,4);
//    return 1;
//}

/*******************************************************************************
* Function Name  : down(u16 value)
* Description    :
* Input          : None
* Output         : None
* Return         : High:
*******************************************************************************/

u8 Down(u8 down_speed,u16 down_hight)
{
    char buf[8];
    buf[0] = 0x03;
    buf[1] = down_speed;
    buf[2] = (down_hight >> 8)&0xff;
    buf[3] = down_hight & 0xff;
    ExtId_TX_CAN1(0x405,buf,4);
    return 1;
}



/**
 *  brief
 *  param
 *  retval
 */

//u8 TargetHight(int h)
//{
//    typedef u8 (*FuncUpOrDown)(u8 up_speed,u16 up_hight);
//    static FuncUpOrDown  UpOrDown = Up;
//    int  hight;
//    static int  dh;
//    static u8   done = 1;
//    int deltaHight = 0;
//    hight = GetHight ();

//    if (hight < -2)
//    {
//        void InitEncoderCord(void);
//        InitEncoderCord();
//        hight = GetHight ();
//    }

//    if (labs (h - hight) <= 116)
//    {
//        if (done == 0)
//            done = StopLift();
//        return done;
//    }

//    if ( (h - hight) > 0 )
//    {
//        UpOrDown = Up;

//        if (done)
//        {
//            done = 0;
//            dh   = h - hight;
//          //  RTT_printf ("now Up ,hight = %d,dh = %d\r\n", hight, dh);
//        }
//    }
//    else
//    {
//        UpOrDown = Down;

//        if (done)
//        {
//            done = 0;
//            dh   = h - hight;
//           // RTT_printf ("now Down,higth = %d,dh = %d\r\n", hight, dh);
//        }
//    }

//    done = UpOrDown (dh);

//    for ( ; deltaHight < 800; hight = GetHight (),deltaHight = labs (h - hight) )
//    {
//        if (labs (h - hight) > 116)
//            done = UpOrDown (dh);
//        else
//        {
//            done = StopLift();
//            RTT_printf("Get Target Hight\r\n");
//            break;
//        }
//       if( LiftSecure())
//          return 0;
//    }
//    if( LiftSecure())
//       return 0;
//    
//    return done;
//}



void TestEncoder_cord(void)
{
    int hight;
    static u32 lastTick;
    if(GetSysTick()-lastTick < 1000)
        return;
    lastTick = GetSysTick();
    hight = GetHight();
    RTT_printf("Hight %d\r\n",hight);
}



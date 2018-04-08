
#include "NAV350.h"
#include "stdlib.h"
#include "can.h"
#include "lib_retarget_printf.h"
#include "EncoderNavigation.h"
#include "motor.h"
#include "clock.h"
//#define DOUT RTT_printf
#define DOUT(...)
//#include "RoutePlan.h"
//#include "PID.h"
//#include "Util.h"
//#include "GPIO_Trans.h"
//#include "DAC7571.h"
//#include "task.h"
//#include "ENCODER.H"
//#include "forkLift.h"
#ifndef PI
#define PI             3.14159265358979f
#endif

	
_Nav_Send_Str       Nav_Send_Str;
_Send_Str *         Curr_Pro;

_DeviceInfo         DeviceInfo;                                                                                    //NAV350����
_GetPoseData        PositionStr={0};                                                                                   //��ȡ���Ķ�λ����
//_ROUTE_STATE        RouteState;

u8                  NAV350_State = NAV350_Powerup;
u8                  Init_State   = Login;
u8                  Nav350PowerupFinish = 0;
s32                 PositionNum;
s16                 LastPos;
u8                  AllScanPoint = 1;
u8                  stopflag = 0;

static int8_t       WaitNavPowerupTimerIndex = -1;                                                                 //�ȴ�Nav350�ϵ����
static int8_t       WaitNavRespTimerIndex = -1;                                                                    //�ȴ�Nav350��Ӧ��ʱʱ��
//static int8_t PeriodicalGetPoseTimerIndex = -1;//���ڻ�ȡ��λ����
_Func_Nav           Cmd_List[] = {
  { SetUserLevelStr, &NavFillLogin, &NavParseLogin }, { SetOperatingModeStr, &NavFillStanbyMode, NULL }, { SetCurrentLayerStr, &NavFillSetCurrLayer, NULL }, \
  { SetSlidMeanStr, &NavFillSetSlidMean, NULL }, { SetPositioningDataFormatStr, &NavFillSetPoseDataFormal, NULL }, /*{SetReflectorIdentWindowStr,&NavFillSetReflectorIdentwindow,NULL},*/
  { SetNClosestReflectorStr, &NavFillSetNClosest, NULL }, {SetActionRadiusStr,&NavFillSetActionRadius,NULL},{ SetOperatingModeStr, &NavFillNavigationMode, &NavParseOperatMode },
  { GetPoseDataStr, &NavFillGetPoseData, &NavParseNavigation }, { GetTimestampStr, &NavFillGetTimestamp, &NavParseTimeStamp },/* { GetPoseDataStr, &NavFillGetPoseData, &NavParseNavigation },*/

};
u8                  Cmd_List_Num = sizeof(Cmd_List) / sizeof(_Func_Nav);


//_Nearby_Route_Info *Nearby_Route;

s32                 HoriAngle    = 0;
//PID                 AnglePID;
s32                 AdjAngle     = 0;

void NavSendTestByCan(u8 test1, u8 test2)
{
    char        buf[11] = { 0 };	

	buf[0] = test1;
	buf[1] = test2;

    ExtId_TX_CAN1 (0x172, buf, 2);
}

/****************************************************************************
* ��    �ƣ�Nav350CommInit
* ��    �ܣ��뼤�⴫�����ӿ�
* ��    ������
* ��    �أ���
* ˵    ����
****************************************************************************/
void Nav350CommInit ( void )
{
  USART3_Init ( BAUD_115200 );

  memset ( (u8 *)&Nav_Send_Str, 0x00, sizeof(Nav_Send_Str) );

}


/****************************************************************************
* ��    �ƣ�Nav350GetTxPoint
* ��    �ܣ��ӷ��Ͷ�������ȡ���õ�����
* ��    ������
* ��    �أ���
* ˵    ����
****************************************************************************/
_Send_Str* Nav350GetTxPoint ( void )
{
  return &Nav_Send_Str.Arr[Nav_Send_Str.InIndex];
}


/****************************************************************************
* ��    �ƣ�Nav350AddInIndex
* ��    �ܣ���Ҫ���͵�������ӵ����Ͷ�����
* ��    ������
* ��    �أ���
* ˵    ����
****************************************************************************/
void Nav350AddInIndex ( void )
{
  Nav_Send_Str.InIndex++;
  Nav_Send_Str.TotalNum++;
  if (Nav_Send_Str.InIndex >= MAX_SEND_PACK_NUM)
    Nav_Send_Str.InIndex = 0;
}


/****************************************************************************
* ��    �ƣ�Nav350GetOutIndex
* ��    �ܣ���Ҫ���͵�������ӵ����Ͷ�����
* ��    ������
* ��    �أ���
* ˵    ����
****************************************************************************/
_Send_Str* Nav350GetOutIndex ( void )
{
  _Send_Str *Point;
  Point = &Nav_Send_Str.Arr[Nav_Send_Str.OutIndex];
  Nav_Send_Str.OutIndex++;
  Nav_Send_Str.TotalNum--;
  if (Nav_Send_Str.OutIndex >= MAX_SEND_PACK_NUM)
    Nav_Send_Str.OutIndex = 0;
  return Point;
}


/*******************************************************************************
* Function Name  : WaitNavPowerupTimeout
* Description    :�ȴ�NAV350�ϵ����
* Input          :
* Output         :
* Return         : None
*******************************************************************************/
int8_t WaitNavPowerupTimeout ( void *cbptr )
{
  Nav350PowerupFinish = 1;
  NavSendTestByCan(0,1);
  return 0;
}


/*******************************************************************************
* Function Name  : WaitNavRespTimeout
* Description    :�ȴ�NAV350��Ӧ��ʱʱ��
* Input          :
* Output         :
* Return         : None
*******************************************************************************/
int8_t WaitNavRespTimeout ( void *cbptr )
{
  UART3_Str.Send_Finish = 1;
  NavSendTestByCan(Init_State,156);
  return 0;
  
}


/*******************************************************************************
* Function Name  : PeriodicalGetPoseTimeout
* Description    :���ڻ�ȡ��λ����
* Input          :
* Output         :
* Return         : None
*******************************************************************************/
int8_t PeriodicalGetPoseTimeout ( void *cbptr )
{
  Curr_Pro = Nav350GetTxPoint ();
  NavFillGetPoseData ( Curr_Pro );
  Nav350AddInIndex ();
  Nav_Send_Str.SendState = SEND_RDY;
  AutoNavProcess ();
	
  return 0;
}


/****************************************************************************
* ��    �ƣ�Nav350LoopPro
* ��    �ܣ���NAV350����Ҫ��������
* ��    ������
* ��    �أ���
* ˵    ����
****************************************************************************/
void  WorkFlow ( void );
void  OneSecondOneTime( void );

typedef struct
{
	int dataHead;
	int dataLenght;
}_Mag;

char tempData[100] = {0};



void Nav350LoopPro ( void )
{
  u8    i;
	
	int j = 0;
	int frameNum = 0;
	_Mag mag[10] = {0};
  char *begin;

  switch (NAV350_State)
  {
    case NAV350_Powerup:
      if (WaitNavPowerupTimerIndex == -1)
        Malloc_Timer ( 15 /*8*/ * 1000, Only_Mode, WaitNavPowerupTimeout, NULL, &WaitNavPowerupTimerIndex );
			
      if (Nav350PowerupFinish == 1) 
			{
        Nav350PowerupFinish = 0;
        Init_State          = Login;
        NAV350_State        = NAV350_Init;
        //IncPIDInit ( &AnglePID );
      }
      break;

    case NAV350_Init:
      if (Nav_Send_Str.SendState == SEND_RDY || Nav_Send_Str.SendState == SEND_TIMEOUT) 
			{
        if (Nav_Send_Str.SendState == SEND_RDY) 
				{
          Curr_Pro = Nav350GetTxPoint ();
          Cmd_List[Init_State].SendFunc ( Curr_Pro ); //Ӧ���Ƿ��͵�¼��Ϣ����������ַ���
          //Nav350AddInIndex();
          //if(Init_State == Login)
					//Delay_ms(20000);
        }
        Nav_Send_Str.SendState = SENDING;
				
        if (WaitNavRespTimerIndex == -1)
          Malloc_Timer ( 1000, Only_Mode, WaitNavRespTimeout, NULL, &WaitNavRespTimerIndex );
				
        USART3_Send_Str ( Curr_Pro->Data, Curr_Pro->SendLen );
        Curr_Pro->Data[Curr_Pro->SendLen] = 0x00;
        //USART2_Send_Str(Curr_Pro->Data,Curr_Pro->SendLen);
        DOUT("%s\r\n",Curr_Pro->Data);
      }
			else if (Nav_Send_Str.SendState == SENDING) 
			{
        if (UART3_Str.Send_Finish && WaitNavRespTimerIndex == -1) 
				{
          if (WaitNavRespTimerIndex != -1)
            Free_Timer ( &WaitNavRespTimerIndex );
					
          Nav_Send_Str.SendState = SEND_FINISH;
        }
      } 
			else if (Nav_Send_Str.SendState == SEND_FINISH) 
			{
        Nav_Send_Str.SendState = SEND_RDY;
        if (Init_State < GetTimeStamp) 
				{
          Init_State++;
        } 
				else 
				{
          //NAV350_State = NAV350_Navigation;
        }
      }
      if (DeviceInfo.AccessAuthFlag == 1 && DeviceInfo.OperateMode == 4 && Init_State == GetTimeStamp) 
			{
        NAV350_State = NAV350_Navigation;
      }
      break;

    case NAV350_Navigation:
//      CountLoop();
      //WorkFlow ();
      if (PeriodicalGetPoseTimerIndex == -1)
        Malloc_Timer ( 190, Auto_Mode, PeriodicalGetPoseTimeout, NULL, &PeriodicalGetPoseTimerIndex );
      
			if (Nav_Send_Str.TotalNum && Nav_Send_Str.SendState == SEND_RDY) 
			{
        Curr_Pro = Nav350GetOutIndex ();
        Nav_Send_Str.SendState = SENDING;

        //if(WaitNavRespTimerIndex == -1)
        //	Malloc_Timer(200,Only_Mode,WaitNavRespTimeout,NULL,&WaitNavRespTimerIndex);
        USART3_Send_Str ( Curr_Pro->Data, Curr_Pro->SendLen );
        Curr_Pro->Data[Curr_Pro->SendLen] = 0x00;

        DOUT("%s\r\n",Curr_Pro->Data);
      } 
			else if (Nav_Send_Str.SendState == SENDING) 
			{
      }
			else if (Nav_Send_Str.SendState == SEND_FINISH) 
			{
        Nav_Send_Str.SendState = SEND_RDY;
      }
      break;

    case NAV350_Abort:
      if (PeriodicalGetPoseTimerIndex != -1)
        Free_Timer ( &PeriodicalGetPoseTimerIndex );
      NAV350_State = NAV350_Powerup;
      break;

    default:
      NAV350_State = NAV350_Powerup;
      break;
  }


	if(isWriteForUart3 == 0)
	{
		if(uart3_Recv_Buf[0].Recv_Flag == 1)
		{
			memset(&Recv_Buf,0,sizeof(Recv_Str));
			memcpy(&Recv_Buf,&uart3_Recv_Buf[0],sizeof(Recv_Str));
			memcpy(&uart3_Recv_Buf[0],&uart3_Recv_Buf[1],sizeof(Recv_Str)*(BufSize - 1));
			memset(&uart3_Recv_Buf[BufSize - 1],0,sizeof(Recv_Str));
#if Debug_Flag
			RTT_printf("Recv_Buf ---> %s \n",Recv_Buf.data);
#endif
		}

		
	  if(Recv_Buf.Recv_Flag) 
		{
			Recv_Buf.Recv_Flag = 0;
			Recv_Buf.data[Recv_Buf.Recv_Len] = 0x00;
			//DOUT("%s\r\n",Recv_Buf.data);
			
			//�󷵻���֡ͷ��λ��
			for(j = 0; j < Recv_Buf.Recv_Len; j++)
			{
				if(Recv_Buf.data[j] == 0x02 && Recv_Buf.data[j+1] == 's')
				{
					mag[frameNum].dataHead = j;
					frameNum++;
				}
			}
		
			//��ÿ��֡�ĳ���
			for(j=0; j<frameNum; j++)
			{
				if(mag[j+1].dataHead != 0)
				{
					mag[j].dataLenght = mag[j+1].dataHead - mag[j].dataHead;
				}
				else
				{
					mag[j].dataLenght = Recv_Buf.Recv_Len - mag[j].dataHead;
				}
			}
			
			//����ִ��ÿһ֡
			for(j=0; j<frameNum; j++)
			{
				memset(tempData,0,sizeof(char)*100);
				memcpy(tempData,&Recv_Buf.data[mag[j].dataHead],mag[j].dataLenght);
#if Debug_Flag		
				RTT_printf("tempData --->%s\n",tempData);
#endif
				for(i = 0; i < Cmd_List_Num; i++) 
				{
					if(( begin = strstr (tempData, Cmd_List[i].Cmd )) != NULL ) 
					{
						begin += strlen ( Cmd_List[i].Cmd );
						begin += 1;
						if(Cmd_List[i].RecvFunc != NULL) 
						{
							Cmd_List[i].RecvFunc ( begin );
						}
					}
				}
			}
		}
	}
}




/****************************************************************************
* ��    �ƣ�AutoNavProcess
* ��    �ܣ��Զ���������
* ��    ������
* ��    �أ���
* ˵    ����
****************************************************************************/
void AutoNavProcess ( void )
{
    Nav350FindRoute ();
}


extern s16  lastAngle;

/****************************************************************************
* ��    �ƣ�Nav350FindRoute
* ��    �ܣ�NAV350����
* ��    ������
* ��    �أ���
* ˵    ����
****************************************************************************/
/*extern*/ u8 modeflag;
extern u32 sysTick;
void Nav350FindRoute ( void )
{
  static u8  flag = 255;
  static u32 lastTick;
  if(modeflag != 2){ 
    flag = 255;
    return;
  }
  if (PositionStr.Position_Mode == CONTINUOUS_POSE
    && modeflag == 2) { //����������ģʽ
    
    if (flag != 1){
      flag        = 1;
      lastTick = sysTick;
	  DOUT("nav...\n");	  
    }
    if(sysTick - lastTick >= 5000)
      ;
  } else {
    if (flag != 0) {
      flag = 0;
      DOUT("change...\n");
    }
  }
  

}


/****************************************************************************
* ��    �ƣ�CalcPID
* ��    �ܣ�����PID
* ��    ������
* ��    �أ���
* ˵    ����
****************************************************************************/
void CalcPID ( u32 HoriAngle )
{

}


/****************************************************************************
* ��    �ƣ�NavFillLogin
* ��    �ܣ�����¼Ȩ����֤
* ��    ������
* ��    �أ���
* ˵    ����
****************************************************************************/
void NavFillLogin ( _Send_Str *Str )
{
  char *    begin;
  u16       len   = 0;

  _ASC_Pro *Point = (_ASC_Pro *)&Str->Data;
  Point->Stx   = ASC_STX;
  begin        = Point->Data;
  sprintf ( begin, "%s %s ", sMN, SetUserLevelStr );

  //sprintf(begin,"sMI 5 ");
  len          = strlen ( begin );
  begin       += strlen ( begin );
  sprintf ( begin, "3 F4724744" );
  len         += strlen ( begin );
  begin       += strlen ( begin );
  *begin       = ASC_ETX;

  Str->SendLen = len + 2;
  NavSendTestByCan(Init_State,DeviceInfo.AccessAuthFlag);
  
}


/****************************************************************************
* ��    �ƣ�NavParseLogin
* ��    �ܣ�������¼Ȩ����֤
* ��    ������
* ��    �أ���
* ˵    ����
****************************************************************************/
void NavParseLogin ( char *data )
{
  DeviceInfo.AccessAuthFlag = strtol ( data, NULL, 10 );
  NavSendTestByCan(Init_State,DeviceInfo.AccessAuthFlag);
}


/****************************************************************************
* ��    �ƣ�NavFillLogin
* ��    �ܣ����Stanbyģʽ����
* ��    ������
* ��    �أ���
* ˵    ����
****************************************************************************/
void NavFillStanbyMode ( _Send_Str *Str )
{
  char *    begin;
  u16       len   = 0;

  _ASC_Pro *Point = (_ASC_Pro *)&Str->Data;
  Point->Stx   = ASC_STX;
  begin        = Point->Data;
  sprintf ( begin, "%s %s 1", sMN, SetOperatingModeStr );
  len          = strlen ( begin );
  begin       += strlen ( begin );
  *begin       = ASC_ETX;

  Str->SendLen = len + 2;
  
  NavSendTestByCan(Init_State,DeviceInfo.OperateMode);
}


/****************************************************************************
* ��    �ƣ�NavParseOperatMode
* ��    �ܣ���������ģʽ����
* ��    ������
* ��    �أ���
* ˵    ����
****************************************************************************/
void NavParseOperatMode ( char *data )
{
  int  Opeatr = 0;
	
	if (strlen ( data ) < 3)
	{
		return;
	}

	Opeatr = data[2] - 0x30;
  DeviceInfo.OperateMode = Opeatr;
	
  
  /*
	char *begin;
	if ( ( begin = strstr ( data, SetOperatingModeStr ) ) != NULL )
	{
    begin += strlen ( SetOperatingModeStr );
    sscanf ( begin, " %*x %x", &Opeatr );
    DeviceInfo.OperateMode = Opeatr;
  }
  */
  NavSendTestByCan(Init_State,DeviceInfo.OperateMode);
}


/****************************************************************************
* ��    �ƣ�NavFillSetCurrLayer
* ��    �ܣ��������ͼ��
* ��    ������
* ��    �أ���
* ˵    ����
****************************************************************************/
void NavFillSetCurrLayer ( _Send_Str *Str )
{
  char *    begin;
  u16       len   = 0;

  _ASC_Pro *Point = (_ASC_Pro *)&Str->Data;
  Point->Stx   = ASC_STX;
  begin        = Point->Data;
  sprintf ( begin, "%s %s 0", sWN, SetCurrentLayerStr );
  len          = strlen ( begin );
  begin       += strlen ( begin );
  *begin       = ASC_ETX;

  Str->SendLen = len + 2;
}


/****************************************************************************
* ��    �ƣ�NavFillSetSlidMean
* ��    �ܣ����
* ��    ������
* ��    �أ���
* ˵    ����
****************************************************************************/
void NavFillSetSlidMean ( _Send_Str *Str )
{
  char *    begin;
  u16       len   = 0;

  _ASC_Pro *Point = (_ASC_Pro *)&Str->Data;
  Point->Stx   = ASC_STX;
  begin        = Point->Data;
  sprintf ( begin, "%s %s 8", sWN, SetSlidMeanStr );
  len          = strlen ( begin );
  begin       += strlen ( begin );
  *begin       = ASC_ETX;

  Str->SendLen = len + 2;
}


/****************************************************************************
* ��    �ƣ�NavFillSetPoseDataFormal
* ��    �ܣ�������ö�λ���ݸ�ʽ
* ��    ������
* ��    �أ���
* ˵    ����
****************************************************************************/
void NavFillSetPoseDataFormal ( _Send_Str *Str )
{
  char *    begin;
  u16       len   = 0;

  _ASC_Pro *Point = (_ASC_Pro *)&Str->Data;
  Point->Stx   = ASC_STX;
  begin        = Point->Data;
  sprintf ( begin, "%s %s 1 1", sWN, SetPositioningDataFormatStr );
  len          = strlen ( begin );
  begin       += strlen ( begin );
  *begin       = ASC_ETX;

  Str->SendLen = len + 2;
}


/****************************************************************************
* ��    �ƣ�NavFillSetReflectorIdentwindow
* ��    �ܣ�������÷�����ʶ�𴰿�
* ��    ������
* ��    �أ���
* ˵    ����
****************************************************************************/
void NavFillSetReflectorIdentwindow ( _Send_Str *Str )
{
  char *    begin;
  u16       len   = 0;

  _ASC_Pro *Point = (_ASC_Pro *)&Str->Data;
  Point->Stx   = ASC_STX;
  begin        = Point->Data;
  sprintf ( begin, "%s %s 300 300 500 70000", sWN, SetReflectorIdentWindowStr );
  len          = strlen ( begin );
  begin       += strlen ( begin );
  *begin       = ASC_ETX;

  Str->SendLen = len + 2;
}


/****************************************************************************
* ��    �ƣ�NavFillSetNClosest
* ��    �ܣ�������ü������������
* ��    ������
* ��    �أ���
* ˵    ����
****************************************************************************/
void NavFillSetNClosest ( _Send_Str *Str )
{
  char *    begin;
  u16       len   = 0;

  _ASC_Pro *Point = (_ASC_Pro *)&Str->Data;
  Point->Stx   = ASC_STX;
  begin        = Point->Data;
  sprintf ( begin, "%s %s 0", sWN, SetNClosestReflectorStr );
  len          = strlen ( begin );
  begin       += strlen ( begin );
  *begin       = ASC_ETX;

  Str->SendLen = len + 2;
}


/****************************************************************************
* ��    �ƣ�NavFillSetNClosest
* ��    �ܣ�������ü������������
* ��    ������
* ��    �أ���
* ˵    ����
****************************************************************************/
void NavFillSetActionRadius ( _Send_Str *Str )
{
  char *    begin;
  u16       len   = 0;

  _ASC_Pro *Point = (_ASC_Pro *)&Str->Data;
  Point->Stx   = ASC_STX;
  begin        = Point->Data;
  sprintf ( begin, "%s %s 400 70000", sWN, SetActionRadiusStr );
  len          = strlen ( begin );
  begin       += strlen ( begin );
  *begin       = ASC_ETX;

  Str->SendLen = len + 2;
}


/****************************************************************************
* ��    �ƣ�NavFillNavigationMode
* ��    �ܣ�������ý��뵼��ģʽ
* ��    ������
* ��    �أ���
* ˵    ����
****************************************************************************/
void NavFillNavigationMode ( _Send_Str *Str )
{
  char *    begin;
  u16       len   = 0;

  _ASC_Pro *Point = (_ASC_Pro *)&Str->Data;
  Point->Stx   = ASC_STX;
  begin        = Point->Data;
  sprintf ( begin, "%s %s 4", sMN, SetOperatingModeStr );
  len          = strlen ( begin );
  begin       += strlen ( begin );
  *begin       = ASC_ETX;

  Str->SendLen = len + 2;
}


/****************************************************************************
* ��    �ƣ�NavFillGetTimestamp
* ��    �ܣ�����ȡͬ��ʱ���
* ��    ������
* ��    �أ���
* ˵    ����
****************************************************************************/
void NavFillGetTimestamp ( _Send_Str *Str )
{
  char *    begin;
  u16       len   = 0;

  _ASC_Pro *Point = (_ASC_Pro *)&Str->Data;
  Point->Stx   = ASC_STX;
  begin        = Point->Data;
  sprintf ( begin, "%s %s", sMN, GetTimestampStr );
  len          = strlen ( begin );
  begin       += strlen ( begin );
  *begin       = ASC_ETX;

  Str->SendLen = len + 2;
}


/****************************************************************************
* ��    �ƣ�NavParseTimeStamp
* ��    �ܣ�����NAV350�ڲ�ʱ���
* ��    ������
* ��    �أ���
* ˵    ����
****************************************************************************/
void NavParseTimeStamp ( char *data )
{
  DeviceInfo.Timestamp = strtoul ( data, NULL, 16 );
  TickCount = DeviceInfo.Timestamp;

}


/****************************************************************************
* ��    �ƣ�NavFillGetPoseData
* ��    �ܣ�����ȡ��λ����
* ��    ������
* ��    �أ���
* ˵    ����
****************************************************************************/
void NavFillGetPoseData ( _Send_Str *Str )
{
  char *    begin;
  u16       len   = 0;

  _ASC_Pro *Point = (_ASC_Pro *)&Str->Data;
  Point->Stx   = ASC_STX;
  begin        = Point->Data;
  sprintf ( begin, "%s %s %x", sMN, GetPoseDataStr, GETPOSETYPE_WAIT );
  len          = strlen ( begin );
  begin       += strlen ( begin );
  *begin       = ASC_ETX;

  Str->SendLen = len + 2;
}

void NavSendPoseByCan(void)
{
	static int flag = 0;
  char       buf[11] = { 0 };
	int        i = 0;
	u32 angle = PositionStr.CurrPose.angle/10;
	

	buf[i++] = (PositionStr.CurrPose.X_Axis & 0x00ff0000)>>16;
	buf[i++] = (PositionStr.CurrPose.X_Axis & 0x0000ff00)>>8;
	buf[i++] =  PositionStr.CurrPose.X_Axis & 0x000000ff;

	buf[i++] = (PositionStr.CurrPose.Y_Axis & 0x00ff0000)>>16;
	buf[i++] = (PositionStr.CurrPose.Y_Axis & 0x0000ff00)>>8;
	buf[i++] =  PositionStr.CurrPose.Y_Axis & 0x000000ff;
	
	buf[i++] = (angle & 0x0000ff00)>>8;
	buf[i++] =  angle & 0x000000ff;

	flag++;
	
	ExtId_TX_CAN1 (0x102, buf, i);
}


void NavSendErrorByCan(void)
{
  char        buf[3] = { 0 };

	buf[0] = 0x02;
  buf[1] = PositionStr.Position_Mode;
  buf[2] = PositionStr.ErrCode;
  ExtId_TX_CAN1 (0x102, buf, 3);
}



/****************************************************************************
* ��    �ƣ�NavParseNavigationData
* ��    �ܣ�������������
* ��    ������
* ��    �أ���
* ˵    ����
****************************************************************************/
void NavParseNavigation ( char *data )
{
  char * begin, *end;
  float  a, b;
  begin = data;
  if (strlen ( data ) < 5)
    return;

//	if( (begin = strstr(data,GetPoseDataStr)) == NULL)
//		return;
  PositionStr.Version = strtol ( begin, &end, 16 );
  begin = end;
  PositionStr.ErrCode = strtol ( begin, &end, 16 );
  begin = end;
  PositionStr.wait = strtol ( begin, &end, 16 );
  begin = end;
  PositionStr.Follow = strtol ( begin, &end, 16 );
  begin = end;
  if (PositionStr.Follow == 1) {
    PositionStr.CurrPose.X_Axis = strtoul ( begin, &end, 16 );
    begin = end;
    PositionStr.CurrPose.Y_Axis = strtoul ( begin, &end, 16 );
    begin = end;
    PositionStr.CurrPose.angle  = strtol ( begin, &end, 16 );
	  PositionStr.CurrPose.angle = (PositionStr.CurrPose.angle+180000)%360000;
	  begin = end;
    a = PositionStr.CurrPose.angle * PI / 1000 / 180;
    b = PositionStr.CurrPose.angle * PI / 1000 / 180;
    a = 0;//412 * cos ( a );
    b = 0;//412 * sin ( b );
    PositionStr.CurrPose.X_Axis = PositionStr.CurrPose.X_Axis + a;
    PositionStr.CurrPose.Y_Axis = PositionStr.CurrPose.Y_Axis + b;
    PositionStr.OptFlag         = strtol ( begin, &end, 16 );
    begin = end;
    if (PositionStr.OptFlag == 1) {
      PositionStr.OutputMode        = strtol ( begin, &end, 16 );
      begin = end;
      PositionStr.Timestamp         = strtoul ( begin, &end, 16 );
      begin = end;
      PositionStr.MeanDev           = strtoul ( begin, &end, 16 );
      begin = end;
      PositionStr.Position_Mode     = strtol ( begin, &end, 16 );
      begin = end;
      PositionStr.InfoState         = strtoul ( begin, &end, 16 );
      begin = end;
      PositionStr.UsedReflectorsNum = strtol ( begin, &end, 16 );
      begin = end;
    }
  }
	
//	NavSendPoseByCan();
//	NavSendErrorByCan();
	
		SendCurrPoseu(2);
}


void SendCurrPoseu(int type)
{
	InformationForUpdatePose updataPose;
	POSE newPose;
	
	motor_info_s *motor_info = 0;
  motor_info = &g_motor_control;
	
	updataPose.dataType = type;
	updataPose.agvrotangle = motor_info->angle;
	updataPose.agvspeed = motor_info->speed;//rpm
	updataPose.laserx = PositionStr.CurrPose.X_Axis;
	updataPose.lasery = PositionStr.CurrPose.Y_Axis;
	updataPose.laserangle = PositionStr.CurrPose.angle;
	
	if(type == 1)
	{
		//RTT_printf("EncoderData,%10d,%10d,%10d\n",GetSysTick(),updataPose.agvrotangle,updataPose.agvspeed);
		RTT_printf("1,%10d,%10d,%10d\n",GetSysTick(),updataPose.agvrotangle,updataPose.agvspeed);
	}
	if(type == 2)
	{
		//RTT_printf("LaserOri   ,%10d,%10d,%10d,%10d\n",GetSysTick(),updataPose.laserx,updataPose.lasery,updataPose.laserangle);
		RTT_printf("2,%10d,%10d,%10d,%10d\n",GetSysTick(),updataPose.laserx,updataPose.lasery,updataPose.laserangle);
	}
	

	newPose = updateCurrentPose(updataPose,GetSysTick());
	
	PositionStr.CurrPose.X_Axis = newPose.x;
	PositionStr.CurrPose.Y_Axis = newPose.y;
	PositionStr.CurrPose.angle = newPose.angle;
	
	

	if(1 == newPose.dataValidFlag)
	{
		//RTT_printf("CalData    ,%10d,%10d,%10d,%10d\n",GetSysTick(),newPose.x,newPose.y,newPose.angle);
	  RTT_printf("3,%10d,%10d,%10d,%10u\n",GetSysTick(),newPose.x,newPose.y,newPose.angle);
		NavSendPoseByCan();
	}
}

void ShowTime(void)
{
	static u32 lastSysTick = 0;
  static u32 decSysTick  = 0;
	if(lastSysTick != sysTick)
	{
		decSysTick = sysTick - lastSysTick;
		RTT_printf("******> time interval = %d <*******\n",decSysTick);
		lastSysTick = sysTick;
	}
}


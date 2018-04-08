
#ifndef __NAV350_H
#define __NAV350_H

#include "stm32f10x.h"
#include "UART3.h"
#include "Timer.h"
#include "string.h"
#include "stdint.h"
#include "stdio.h"
#include "math.h"


#define BIN_STX "\x02\x02\x02\x02"   //BINЭ����ʼ��־
#define ASC_STX 0x02
#define ASC_ETX 0x03
#define SEPARATE_SYMBOL "\x20"             //�ָ���



#define sRN  "sRN"
#define sWN  "sWN"
#define sMN  "sMN"
#define sAN  "sAN"

#define sRA  "sRA"
#define sWA  "SWA"
#define sMA  "sMA"

#define DeviceIdentStr            		"DeviceIdent"
#define SerialNumberStr 							"SerialNumber"
#define SetCurrentLayerStr 						"NEVACurrLayer"
#define SetSlidMeanStr    						"NPOSSlidingMean"
#define SetPositioningDataFormatStr   "NPOSPoseDataFormat"
#define SetActionRadiusStr            "NLMDActionRadius"
#define SetNClosestReflectorStr				"NLMDnClosest"
#define SetReflectorIdentWindowStr  	"NCORIdentWindow"
#define SetReflectorSizeStr    				"NLMDReflSize"
#define SetReflectorTypeStr						"NLMDReflType"
#define SetOperatingModeStr						"mNEVAChangeState"
#define SetUserLevelStr               "SetAccessMode"
#define GetTimestampStr 							"mNAVGetTimestamp"
#define GetPoseDataStr   							"mNPOSGetPose"
#define DeviceResetStr 								"mNAVReset"


#define GETPOSETYPE_INSTANTLY   0     //������ȡ���һ�ζ�λ����
#define GETPOSETYPE_WAIT        1     //�ȴ����µĶ�λ���ݲ���

#define MAX_SEND_PACK_NUM   3  //����Ͷ���
 static int8_t PeriodicalGetPoseTimerIndex = -1;

__packed typedef struct
{
	char Stx[4];
	u32 Lenth;
	char Data[200];
	u8 CheckSum;
}_BIN_Pro;
__packed typedef struct
{
	char Stx;
//	char Type[3];
//	char Symbol;
//	char Cmd[32];
	char Data[200];
	char Etx;
}_ASC_Pro;

enum
{
	SEND_RDY,  //׼����,���Է�������
	SENDING,   //�ȴ�ģ���Ӧ������
	SEND_FINISH,//�յ�ģ���Ӧ
	SEND_TIMEOUT,//�ȴ�ģ���Ӧ���ݳ�ʱ
};
enum
{
	NAV350_Powerup,  //�ϵ�
	NAV350_Init,	   //��ʼ��
	NAV350_Navigation,//����ģʽ
	NAV350_Abort,     //�쳣����
};

enum //NAV350��ʼ��ģʽ�������õĲ���
{
	Login,
	StandbyMode,
	SetCurrentLayer,
	SetslidingMean,
	SetPoseDataFormat,
	SetRfWindow,
	SetNclosestRf,
	SetOperationMode,
	GetPoseData,
	GetTimeStamp,
};

__packed typedef struct
{
	u16 SendLen;
	char Data[500];
}_Send_Str;

__packed typedef struct
{
	u8 TotalNum;
	u8 InIndex;
	u8 OutIndex;
	u8 SendState;
	_Send_Str Arr[MAX_SEND_PACK_NUM];
}_Nav_Send_Str;
extern _Nav_Send_Str Nav_Send_Str;

typedef void (_Send_Func)(_Send_Str *);
typedef void (_Recv_Func)(char *);
typedef struct
{
	char Cmd[32];
	_Send_Func *SendFunc;
	_Recv_Func *RecvFunc;
}_Func_Nav;

__packed typedef struct
{
	u8 SizeNum;
	char SerialNumber[16];
	u16 Layer_Number;
	u8 OutputMode;
	u8 ShowOptParam;
	u16 ReflectorSize;
	u16 ReflectorType;
	u32 Timestamp;     //NAV350�ڲ�ʱ���
	u8 OperateMode;    //����ģʽ
	u8 AccessAuthFlag; //��¼Ȩ����ȷ
	
}_DeviceInfo;
extern _DeviceInfo DeviceInfo;

__packed typedef struct
{
	u8 ErrCode;
	u32 Timestamp;
}_GetTimestamp;
__packed typedef struct
{
	s32 X_Axis;  /*X������*/
	s32 Y_Axis;  /*Y������*/
	u32 angle;   /*��X��ļн�*/
}_Position;




__packed typedef struct
{
	u16 Version;//��λ���ݰ汾��
	u8 ErrCode; //�������
	u8 wait;    //��ȡ��λ���ݷ���.0:������ȡ;1:�ȴ��¶�λ����
	u16 Follow; //��Ч��λ���ݸ���.0:û�ж�λ����;1.�ж�λ����
	_Position CurrPose;//��ǰ�Ķ�λ�����
	u8 OptFlag;    //�Ƿ��п�ѡ�Ĳ���
	u8 OutputMode;
	u32 Timestamp;
	s32 MeanDev;
	u8  Position_Mode;
	u32 InfoState;
	u8  UsedReflectorsNum;
}_GetPoseData;
enum
{
	INITIAL_POSE,
	CONTINUOUS_POSE,
	VIRTUAL_POSE,
	STOP_POSE,
	INVALID_POSE,
	EXTERNAL_POSE,
};

__packed typedef struct
{
	s16 X_Velocity;    //X���ٶ�
	s16 Y_Velocity;    //Y���ٶ�
	s32 Angular_Velocity;//���ٶ�
	u32 Timestamp;  
	u8  CoordBase; //�����׼.0:�ֲ�����;1:��������
}_SetSpeed;


extern _GetPoseData PositionStr;//��ȡ���Ķ�λ����

void Nav350CommInit(void);
void Nav350LoopPro(void);

void AutoNavProcess(void);
void Nav350FindRoute(void);

void NavFillLogin(_Send_Str *Str);
void NavParseLogin(char *data);
void NavFillStanbyMode(_Send_Str *Str);
void NavParseOperatMode(char *data);
void NavFillSetCurrLayer(_Send_Str *Str);
void NavFillSetSlidMean(_Send_Str *Str);
void NavFillSetPoseDataFormal(_Send_Str *Str);
void NavFillSetReflectorIdentwindow(_Send_Str *Str);
void NavFillSetNClosest(_Send_Str *Str);
void NavFillSetActionRadius(_Send_Str *Str);
void NavFillNavigationMode(_Send_Str *Str);
void NavFillGetTimestamp(_Send_Str *Str);
void NavParseTimeStamp(char *data);
void NavFillGetPoseData(_Send_Str *Str);
void NavParseNavigation(char *data);
int8_t PeriodicalGetPoseTimeout(void *cbptr);
void ShowTime(void);
void SendCurrPoseu(int type);

#endif

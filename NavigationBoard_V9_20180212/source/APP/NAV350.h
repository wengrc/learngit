
#ifndef __NAV350_H
#define __NAV350_H

#include "stm32f10x.h"
#include "UART3.h"
#include "Timer.h"
#include "string.h"
#include "stdint.h"
#include "stdio.h"
#include "math.h"


#define BIN_STX "\x02\x02\x02\x02"   //BIN协议起始标志
#define ASC_STX 0x02
#define ASC_ETX 0x03
#define SEPARATE_SYMBOL "\x20"             //分隔符



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


#define GETPOSETYPE_INSTANTLY   0     //立即读取最近一次定位数据
#define GETPOSETYPE_WAIT        1     //等待最新的定位数据产生

#define MAX_SEND_PACK_NUM   3  //最大发送队列
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
	SEND_RDY,  //准备好,可以发送数据
	SENDING,   //等待模块回应数据中
	SEND_FINISH,//收到模块回应
	SEND_TIMEOUT,//等待模块回应数据超时
};
enum
{
	NAV350_Powerup,  //上电
	NAV350_Init,	   //初始化
	NAV350_Navigation,//导航模式
	NAV350_Abort,     //异常处理
};

enum //NAV350初始化模式下需设置的参数
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
	u32 Timestamp;     //NAV350内部时间戳
	u8 OperateMode;    //操作模式
	u8 AccessAuthFlag; //登录权限正确
	
}_DeviceInfo;
extern _DeviceInfo DeviceInfo;

__packed typedef struct
{
	u8 ErrCode;
	u32 Timestamp;
}_GetTimestamp;
__packed typedef struct
{
	s32 X_Axis;  /*X轴坐标*/
	s32 Y_Axis;  /*Y轴坐标*/
	u32 angle;   /*与X轴的夹角*/
}_Position;




__packed typedef struct
{
	u16 Version;//定位数据版本号
	u8 ErrCode; //错误代码
	u8 wait;    //获取定位数据方法.0:立即获取;1:等待新定位数据
	u16 Follow; //有效定位数据跟随.0:没有定位数据;1.有定位数据
	_Position CurrPose;//当前的定位坐标点
	u8 OptFlag;    //是否有可选的参数
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
	s16 X_Velocity;    //X轴速度
	s16 Y_Velocity;    //Y轴速度
	s32 Angular_Velocity;//角速度
	u32 Timestamp;  
	u8  CoordBase; //座标基准.0:局部坐标;1:绝对坐标
}_SetSpeed;


extern _GetPoseData PositionStr;//获取到的定位数据

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

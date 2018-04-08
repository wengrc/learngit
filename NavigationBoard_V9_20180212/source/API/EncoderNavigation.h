#ifndef _ENCODE_NAV_H
#define _ENCODE_NAV_H

#include "stm32f10x.h"

typedef struct {
	
	 int   IsLocation;
   float DriverRadius ; 
   float DriverLength ;
   float DriverOffset ; 
   float SpeedRatio ;
   float LaserLength; 
	 float RotMTEncoderOffset;
}AGVBasicValue_t; //定义见C文件中


enum
{
    encoder = 1,
    laser = 2,
};

typedef struct
{
    float x;
    float y;
}POINT;

typedef struct
{
    s32 x;
    s32 y;
    u32 angle;
    int   dataValidFlag;
}POSE;

typedef struct
{
    int  dataType;
    short agvspeed; //rpm
    short agvrotangle; //100 multiple
    s32 laserx;
    s32 lasery;
    u32 laserangle; //1000 multiple

} InformationForUpdatePose;


extern void EncoderNavigation(void);
extern POSE updateCurrentPose(InformationForUpdatePose allData , int systime);

#endif



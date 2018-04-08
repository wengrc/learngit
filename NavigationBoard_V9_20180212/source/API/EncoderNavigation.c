#include "EncoderNavigation.h"
#include "math.h"



#define PI             3.14159265358979f
#define DRIVERLENGTH  1200//舵轮中心点到后轮中心点的垂直距离mm
#define DRIVEOFFSET  0.0//舵轮到后轮中心的水平距离（安装偏差）
#define DRIVERRADIUS  130//舵轮半径mm
#define SPEEDRATIO  14//减速比
#define LASERLENGTH  870//激光的投影点到后轮距离
#define ROTMENCODEROFFSET -1.57//degree


AGVBasicValue_t AGVValue;
POINT currentPoint; //record the calculated rear center point;
float currentTheta;//record calculated carAngle


float Pk = 0; //协方差
float Q = 0.25;//预测噪声-应该是方差 //0.5
float R = 0.01;//测量噪声-应该是方差   //0.1

//x, y数据
float Pk1 = 0.0f; //协方差
float Pk2 = 0.0f;
float Q1 = 100.0;//预测噪声-应该是方差 //10
float R1 = 9.0;//测量噪声-应该是方差 //5

int lastSysTime = 0;//record last runtime



enum
{
	FALSE = 0,
	TRUE = 1,
};

//获取叉车速度 单位：mm/s (input: rpm)
float getCarSpeed(short int speed)
{

	speed = (speed/AGVValue.SpeedRatio/30.0f*PI*AGVValue.DriverRadius); //1秒走的距离
	return speed;

}

//获取叉车舵轮角度 单位 ：度
float  getCarSteerAngle(short int rotangle)
{
  return (float)rotangle/100.0f;
}


//返回激光头测量的角度 单位： 度
float getLaserTheta(int laserangle )
{

	return (float)laserangle/1000.0f;
}


//获取激光头转换到后轮的坐标？
POINT getRearPoint(int laserx,int lasery,int laserangle)
{
	float angleRad ;
    POINT rearPoint;
		angleRad  =  (float)laserangle / 1000.0f* PI / 180.0f ;
    rearPoint.x =(float)laserx - AGVValue.LaserLength * cosf(angleRad);
    rearPoint.y =(float)lasery - AGVValue.LaserLength * sinf(angleRad);
	
    return rearPoint;
   
}
//angle 激光原始角度 //transfer to rearCenterPoint
void SetLocationAngle(int X, int Y, int angle)
{

	float angleRad;
	if(angle <= 0)
		return;

	angleRad = (float)angle / 1000.0f* PI / 180.0f ;
	currentPoint.x =(float)X - AGVValue.LaserLength * cosf(angleRad);
	currentPoint.y =(float)Y - AGVValue.LaserLength * sinf(angleRad);
	currentTheta  = (float)angle/1000.0f;
	AGVValue.IsLocation = 1;

}

//lastTheta 单位是角度 //speed mm/s //steerAngle 单位是角度
float predictCurrentAngle(float lastTheta, float speed, float duration, float steerAngle)
{
    float theta;
    float distance;
    float deltaTheta;
    //speed = speed * RATIO; //mm/s
    //duration = duration / 1000; //ms tansform to second

    //距离 = 速度 * 时间
    distance = speed * duration;
    
    steerAngle = steerAngle + AGVValue.RotMTEncoderOffset;
    if(steerAngle > 90.0)
    {
        steerAngle = 90.0;
    }
    else if(steerAngle < -90.0)
    {
        steerAngle = -90.0;
    }

    //度转换成弧度
    steerAngle = steerAngle * PI / 180.0f;
    
    //获取车身角度增量
    deltaTheta = (distance / AGVValue.DriverLength) * sinf(steerAngle);
    deltaTheta = deltaTheta * 180.0f / PI;

    theta = lastTheta + deltaTheta; //单位是角度
    
    //special process
    if(theta < 0.0)
    {
       theta = theta + 360.0;
    }
    else if(theta >= 360.0)
    {
       theta = theta - 360.0;
    }

    return theta;
}


POINT predictCurrentLocation(POINT curPoint, float theta, float speed, float time, float steerAngle)
{
    float runDistance;
    //角度转换成弧度
    theta = theta * PI / 180.0f;
    steerAngle = steerAngle + AGVValue.RotMTEncoderOffset;
    if(steerAngle > 90.0)
    {
        steerAngle = 90.0;
    }
    else if(steerAngle < -90.0)
    {
        steerAngle = -90.0;
    }

    //度转换成弧度
    steerAngle = steerAngle * PI / 180.0f;
    runDistance = speed * time * cosf(steerAngle);
    //根据编码器信息推算出新的坐标位置---(车辆中心点的坐标？）
    curPoint.x = curPoint.x + runDistance * cosf(theta);
    curPoint.y = curPoint.y + runDistance * sinf(theta);

    return curPoint;
}


float KalmanFilterForAngle(float predict, float measurement)
{
    //根据预测的车身角度theta和测量车身角度进行卡尔曼滤波得到更新的车身角度信息
    float Z = measurement;
    //预测阶段
    float xk_ = predict;
    float Pk_ = Pk + Q;

		float xk;

    //测量更新阶段
    //计算卡尔曼系数
    float K = Pk_ / (Pk_ + R);

    float diff = Z - xk_;

    //车身角度在0度在360度会发生跳变，所以应该要特殊处理一下，保证连续性
    if(diff > 270.0f)
    {
        diff = diff - 360.0f;
        //xk_ = xk_ + 360.0f;
    }
    else if(diff < -270.0f)
    {
        diff = diff + 360.0f;
        //xk_ = xk_ - 360.0f;
    }
    //数据融合
    xk = xk_ + K * diff;
    //更新协方差
    Pk = (1 - K) * Pk_;

    //special process
    if(xk < 0.0)
    {
        xk = xk + 360.0;
    }
    else if(xk >= 360.0)
    {
        xk = xk - 360.0;
    }

    return xk;
}


//For x,y seperately;
float KalmanFilterForDistance(float *Pk1, float predict, float measurement)
{
    //根据预测的坐标和测量坐标进行卡尔曼滤波得到更新的坐标
    float Z1 = measurement;
    //预测阶段
    float xk1_ = predict;
    float Pk1_ = *Pk1 + Q1;

    float xk1;

    //测量更新阶段
    //计算卡尔曼系数
    float K1 = Pk1_ / (Pk1_ + R1);

    float diff1 = Z1 - xk1_;

    //数据融合
    xk1 = xk1_ + K1 * diff1;
    //更新协方差
    *Pk1 = (1 - K1) * Pk1_;

    return xk1;
}


POSE updateCurrentPose(InformationForUpdatePose allData, int systime)
{
    int dataType = allData.dataType;
    short int agvspeed = allData.agvspeed;
    short int agvrotangle = allData.agvrotangle;//100 multiple
    int laserx = allData.laserx;
    int lasery = allData.lasery;
    int laserangle = allData.laserangle;//1000 multiple
    float laserTheta;
    float carSteerAngle;
    float carSpeed;
    float angleRad ;

    float predictTheta;
    float lastTheta;
    POINT predictPoint;
    POINT currentRearPoint;
    POSE currentPose; //laser point and angle
    float internalTime;

    if(AGVValue.IsLocation == 0)
    {
        if(laser == dataType)
        {
            SetLocationAngle(laserx, lasery, laserangle);
            angleRad = (float)laserangle / 1000.0f* PI / 180.0f ;
            //transfer back to laserPoint coordination
            currentPose.x =currentPoint.x + AGVValue.LaserLength * cosf(angleRad);
            currentPose.y =currentPoint.y + AGVValue.LaserLength * sinf(angleRad);
            currentPose.angle = currentTheta * 1000;
            currentPose.dataValidFlag = 1;
        }
        else
        {
            currentPose.x = 0;
            currentPose.y = 0;
            currentPose.angle = 0.0;
            currentPose.dataValidFlag = 0;
        }

        lastSysTime = systime;
        return currentPose;
    }

    //在进行激光头数据更新时，进行卡尔曼滤波器处理，将编码器推算出来的数据和激光头的数据进行数据融合
    //根据编码器数据进行推算下一时刻的角度
    if(encoder == dataType)
    {
        //获取叉车速度mm/s
        carSpeed = getCarSpeed(agvspeed);
        //获取叉车舵轮角度 单位：度
        carSteerAngle = getCarSteerAngle(agvrotangle);

        //根据运动学模型计算当前车身角度
        lastTheta = currentTheta;
        internalTime = (systime - lastSysTime)/1000.0f;
        currentTheta = predictCurrentAngle(currentTheta, carSpeed, internalTime, carSteerAngle);
        currentPoint = predictCurrentLocation(currentPoint, (lastTheta + currentTheta) / 2.0f, carSpeed, internalTime, carSteerAngle);

    } //当激光头数据到来时刻，进行数据融合
    else
    {
        //获取叉车速度
        carSpeed = getCarSpeed(agvspeed);
        //获取叉车舵轮角度
        carSteerAngle = getCarSteerAngle(agvrotangle);

        //先根据编码器信息推算出预测的车身角度theta
        lastTheta = currentTheta;
        internalTime = (systime - lastSysTime)/1000.0f;
        predictTheta = predictCurrentAngle(currentTheta, carSpeed, internalTime, carSteerAngle);
        predictPoint = predictCurrentLocation(currentPoint, (lastTheta + predictTheta) / 2.0f, carSpeed, internalTime, carSteerAngle);
        //获取激光头测量的角度
        laserTheta = getLaserTheta(laserangle);
        //获取激光头转换到后轮的坐标
        currentRearPoint = getRearPoint(laserx,lasery,laserangle);
        //卡尔曼滤波
        currentTheta = KalmanFilterForAngle(predictTheta, laserTheta);
        currentPoint.x = KalmanFilterForDistance(&Pk1, predictPoint.x, currentRearPoint.x);
        currentPoint.y = KalmanFilterForDistance(&Pk2, predictPoint.y, currentRearPoint.y);
    }
    angleRad  = currentTheta * PI / 180.0f ;
    //transfer back to laserPoint coordination
    currentPose.x =currentPoint.x + AGVValue.LaserLength * cosf(angleRad);
    currentPose.y =currentPoint.y + AGVValue.LaserLength * sinf(angleRad);
    //printf("currentPoint.y:%f,currentPose.y:%d,angleRad:%f.\n",currentPoint.y,currentPose.y,angleRad);
    currentPose.angle = currentTheta * 1000;
    currentPose.dataValidFlag = 1;
    lastSysTime = systime;
    return currentPose;
}



void EncoderNavigation()
{
	AGVValue.IsLocation = 0;
    AGVValue.DriverLength = DRIVERLENGTH; //舵轮中心点到后轮中心点的垂直距离
    AGVValue.DriverOffset = DRIVEOFFSET;  //舵轮到后轮中心的水平距离（安装偏差）
    AGVValue.DriverRadius = DRIVERRADIUS; //舵轮半径(230.0f / 2)
    AGVValue.SpeedRatio = SPEEDRATIO; //减速比
    AGVValue.LaserLength = LASERLENGTH; //激光的投影点到后轮距离
    AGVValue.RotMTEncoderOffset = ROTMENCODEROFFSET;  //轮子没磨损的0.01931f; //  磨损的0.027736f;??
}


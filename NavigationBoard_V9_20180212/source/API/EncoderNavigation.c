#include "EncoderNavigation.h"
#include "math.h"



#define PI             3.14159265358979f
#define DRIVERLENGTH  1200//�������ĵ㵽�������ĵ�Ĵ�ֱ����mm
#define DRIVEOFFSET  0.0//���ֵ��������ĵ�ˮƽ���루��װƫ�
#define DRIVERRADIUS  130//���ְ뾶mm
#define SPEEDRATIO  14//���ٱ�
#define LASERLENGTH  870//�����ͶӰ�㵽���־���
#define ROTMENCODEROFFSET -1.57//degree


AGVBasicValue_t AGVValue;
POINT currentPoint; //record the calculated rear center point;
float currentTheta;//record calculated carAngle


float Pk = 0; //Э����
float Q = 0.25;//Ԥ������-Ӧ���Ƿ��� //0.5
float R = 0.01;//��������-Ӧ���Ƿ���   //0.1

//x, y����
float Pk1 = 0.0f; //Э����
float Pk2 = 0.0f;
float Q1 = 100.0;//Ԥ������-Ӧ���Ƿ��� //10
float R1 = 9.0;//��������-Ӧ���Ƿ��� //5

int lastSysTime = 0;//record last runtime



enum
{
	FALSE = 0,
	TRUE = 1,
};

//��ȡ�泵�ٶ� ��λ��mm/s (input: rpm)
float getCarSpeed(short int speed)
{

	speed = (speed/AGVValue.SpeedRatio/30.0f*PI*AGVValue.DriverRadius); //1���ߵľ���
	return speed;

}

//��ȡ�泵���ֽǶ� ��λ ����
float  getCarSteerAngle(short int rotangle)
{
  return (float)rotangle/100.0f;
}


//���ؼ���ͷ�����ĽǶ� ��λ�� ��
float getLaserTheta(int laserangle )
{

	return (float)laserangle/1000.0f;
}


//��ȡ����ͷת�������ֵ����ꣿ
POINT getRearPoint(int laserx,int lasery,int laserangle)
{
	float angleRad ;
    POINT rearPoint;
		angleRad  =  (float)laserangle / 1000.0f* PI / 180.0f ;
    rearPoint.x =(float)laserx - AGVValue.LaserLength * cosf(angleRad);
    rearPoint.y =(float)lasery - AGVValue.LaserLength * sinf(angleRad);
	
    return rearPoint;
   
}
//angle ����ԭʼ�Ƕ� //transfer to rearCenterPoint
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

//lastTheta ��λ�ǽǶ� //speed mm/s //steerAngle ��λ�ǽǶ�
float predictCurrentAngle(float lastTheta, float speed, float duration, float steerAngle)
{
    float theta;
    float distance;
    float deltaTheta;
    //speed = speed * RATIO; //mm/s
    //duration = duration / 1000; //ms tansform to second

    //���� = �ٶ� * ʱ��
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

    //��ת���ɻ���
    steerAngle = steerAngle * PI / 180.0f;
    
    //��ȡ����Ƕ�����
    deltaTheta = (distance / AGVValue.DriverLength) * sinf(steerAngle);
    deltaTheta = deltaTheta * 180.0f / PI;

    theta = lastTheta + deltaTheta; //��λ�ǽǶ�
    
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
    //�Ƕ�ת���ɻ���
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

    //��ת���ɻ���
    steerAngle = steerAngle * PI / 180.0f;
    runDistance = speed * time * cosf(steerAngle);
    //���ݱ�������Ϣ������µ�����λ��---(�������ĵ�����ꣿ��
    curPoint.x = curPoint.x + runDistance * cosf(theta);
    curPoint.y = curPoint.y + runDistance * sinf(theta);

    return curPoint;
}


float KalmanFilterForAngle(float predict, float measurement)
{
    //����Ԥ��ĳ���Ƕ�theta�Ͳ�������ǶȽ��п������˲��õ����µĳ���Ƕ���Ϣ
    float Z = measurement;
    //Ԥ��׶�
    float xk_ = predict;
    float Pk_ = Pk + Q;

		float xk;

    //�������½׶�
    //���㿨����ϵ��
    float K = Pk_ / (Pk_ + R);

    float diff = Z - xk_;

    //����Ƕ���0����360�Ȼᷢ�����䣬����Ӧ��Ҫ���⴦��һ�£���֤������
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
    //�����ں�
    xk = xk_ + K * diff;
    //����Э����
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
    //����Ԥ�������Ͳ���������п������˲��õ����µ�����
    float Z1 = measurement;
    //Ԥ��׶�
    float xk1_ = predict;
    float Pk1_ = *Pk1 + Q1;

    float xk1;

    //�������½׶�
    //���㿨����ϵ��
    float K1 = Pk1_ / (Pk1_ + R1);

    float diff1 = Z1 - xk1_;

    //�����ں�
    xk1 = xk1_ + K1 * diff1;
    //����Э����
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

    //�ڽ��м���ͷ���ݸ���ʱ�����п������˲���������������������������ݺͼ���ͷ�����ݽ��������ں�
    //���ݱ��������ݽ���������һʱ�̵ĽǶ�
    if(encoder == dataType)
    {
        //��ȡ�泵�ٶ�mm/s
        carSpeed = getCarSpeed(agvspeed);
        //��ȡ�泵���ֽǶ� ��λ����
        carSteerAngle = getCarSteerAngle(agvrotangle);

        //�����˶�ѧģ�ͼ��㵱ǰ����Ƕ�
        lastTheta = currentTheta;
        internalTime = (systime - lastSysTime)/1000.0f;
        currentTheta = predictCurrentAngle(currentTheta, carSpeed, internalTime, carSteerAngle);
        currentPoint = predictCurrentLocation(currentPoint, (lastTheta + currentTheta) / 2.0f, carSpeed, internalTime, carSteerAngle);

    } //������ͷ���ݵ���ʱ�̣����������ں�
    else
    {
        //��ȡ�泵�ٶ�
        carSpeed = getCarSpeed(agvspeed);
        //��ȡ�泵���ֽǶ�
        carSteerAngle = getCarSteerAngle(agvrotangle);

        //�ȸ��ݱ�������Ϣ�����Ԥ��ĳ���Ƕ�theta
        lastTheta = currentTheta;
        internalTime = (systime - lastSysTime)/1000.0f;
        predictTheta = predictCurrentAngle(currentTheta, carSpeed, internalTime, carSteerAngle);
        predictPoint = predictCurrentLocation(currentPoint, (lastTheta + predictTheta) / 2.0f, carSpeed, internalTime, carSteerAngle);
        //��ȡ����ͷ�����ĽǶ�
        laserTheta = getLaserTheta(laserangle);
        //��ȡ����ͷת�������ֵ�����
        currentRearPoint = getRearPoint(laserx,lasery,laserangle);
        //�������˲�
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
    AGVValue.DriverLength = DRIVERLENGTH; //�������ĵ㵽�������ĵ�Ĵ�ֱ����
    AGVValue.DriverOffset = DRIVEOFFSET;  //���ֵ��������ĵ�ˮƽ���루��װƫ�
    AGVValue.DriverRadius = DRIVERRADIUS; //���ְ뾶(230.0f / 2)
    AGVValue.SpeedRatio = SPEEDRATIO; //���ٱ�
    AGVValue.LaserLength = LASERLENGTH; //�����ͶӰ�㵽���־���
    AGVValue.RotMTEncoderOffset = ROTMENCODEROFFSET;  //����ûĥ���0.01931f; //  ĥ���0.027736f;??
}


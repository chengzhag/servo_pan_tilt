#ifndef __SERVO_PAN_TILT
#define __SERVO_PAN_TILT

#include "ebox.h"
#include "servo.h"
#include "uart_num.h"
#include "PID.hpp"
#include "mpu9250.h"

#define INF_FLOAT 3.402823466e+38F

#define __SERVO_PAN_TILT_DEBUG

class ServoPanTilt
{
	Servo servoY, servoP;
	//UartNum uartM;
	Uart*  uartD;
	sky::PID pidY, pidP;
	float refreshInt;
public:
	float angleY, angleP, angleR;

	ServoPanTilt(Gpio* pinServoYaw, Gpio* pinServoPitch, float refreshInterval = 0.02, Uart* uartDebug = &uart1) :
		servoY(pinServoYaw, 100, 0.7, 2.3),
		servoP(pinServoPitch, 100, 0.7, 2.3),
		//uartM(uartMpu),
		uartD(uartDebug),
		refreshInt(refreshInterval)
	{

	}
	
	//初始化舵机和串口
	void begin()
	{
		servoY.begin();
		servoP.begin();
		servoP.setPct(9);
		//uartM.begin(500000);
		uartD->begin(115200);
		mpu9250Init();

		//初始化yawPID
		pidY.setRefreshInterval(refreshInt);
		pidY.setWeights(0.05, 0.05, 0.001);
		pidY.setOutputLowerLimit(-INF_FLOAT);
		pidY.setOutputUpperLimit(INF_FLOAT);
		pidY.setISeperateThres(2);
		pidY.setDesiredPoint(0);

		//初始化pitchPID
		pidP.setRefreshInterval(refreshInt);
		pidP.setWeights(0.05, 0.05, 0.001);
		pidP.setOutputLowerLimit(-INF_FLOAT);
		pidP.setOutputUpperLimit(INF_FLOAT);
		pidY.setISeperateThres(2);
		pidP.setDesiredPoint(0);
	}

	//刷新pid
	void refresh()
	{
		float pctY = servoY.getPct();
		float pctP = servoP.getPct();

		mpu9250Read(angleP, angleR, angleY);

		pctY += pidY.refresh(angleY);
		pctP += pidP.refresh(angleP);

		servoY.setPct(pctY);
		servoP.setPct(pctP);

#ifdef __SERVO_PAN_TILT_DEBUG
		uartD->printf("yaw:%.1f\tpitch:%.1f\trow:%.1f\tyawOut:%.1f\tpitchOut:%.1f\r\n",
			angleY, angleP, angleR, pctY, pctP);
#endif // __SERVO_PAN_TILT_DEBUG
	}

	//重置目标角度为当前角度
	void reset()
	{
		float angleY, angleP, angleR;

		mpu9250Read(angleP, angleR, angleY);
		//while (uartM.recievedNum != 3);
		pidY.setDesiredPoint(angleY);
		pidP.setDesiredPoint(angleR);
	}
};

#endif
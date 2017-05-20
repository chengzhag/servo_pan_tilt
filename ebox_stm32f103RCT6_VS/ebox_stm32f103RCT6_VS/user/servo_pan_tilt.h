#ifndef __SERVO_PAN_TILT
#define __SERVO_PAN_TILT

#include "ebox.h"
#include "servo.h"
#include "uart_num.h"
#include "PID.hpp"

#define INF_FLOAT 3.402823466e+38F

#define __SERVO_PAN_TILT_DEBUG

class ServoPanTilt
{
	Servo servoY, servoP;
	UartNum uartM;
	Uart*  uartD;
	greg::PID pidY, pidP;
	float refreshInt;
public:
	ServoPanTilt(Gpio* pinServoYaw, Gpio* pinServoPitch, Uart* uartMpu, float refreshInterval = 0.02, Uart* uartDebug = &uart1) :
		servoY(pinServoYaw, 100, 0.7, 2.3),
		servoP(pinServoPitch, 100, 0.7, 2.3),
		uartM(uartMpu),
		uartD(uartDebug),
		refreshInt(refreshInterval)
	{

	}
	
	//初始化舵机和串口
	void begin()
	{
		servoY.begin();
		servoP.begin();
		uartM.begin(500000);
		uartD->begin(9600);

		//初始化yawPID
		pidY.setRefreshInterval(refreshInt);
		pidY.setWeights(0.10, 0.15, 0.005);
		pidY.setOutputLowerLimit(-INF_FLOAT);
		pidY.setOutputUpperLimit(INF_FLOAT);
		pidY.setDesiredPoint(0);

		//初始化pitchPID
		pidP.setRefreshInterval(refreshInt);
		pidP.setWeights(0.12, 0.10, 0.004);
		pidP.setOutputLowerLimit(-INF_FLOAT);
		pidP.setOutputUpperLimit(INF_FLOAT);
		pidP.setDesiredPoint(0);
	}

	//刷新pid
	void refresh()
	{
		if (uartM.recievedNum==3)
		{
			float pctY = servoY.getPct();
			float pctP = servoP.getPct();
			float angleY = uartM.numsBuf[0];
			float angleP = uartM.numsBuf[1];

			pctY += pidY.refresh(angleY);
			pctP += pidP.refresh(angleP);

			servoY.setPct(pctY);
			servoP.setPct(pctP);

#ifdef __SERVO_PAN_TILT_DEBUG
			uartD->printf("yaw:%.1f\tpitch:%.1f\tyawOut:%.1f\tpitchOut:%.1f\r\n",
				angleY, angleP, pctY, pctP);
#endif // __SERVO_PAN_TILT_DEBUG
			
		}
	}

	//重置目标角度为当前角度
	void reset()
	{
		while (uartM.recievedNum != 3);
		pidY.setDesiredPoint(uartM.numsBuf[0]);
		pidP.setDesiredPoint(uartM.numsBuf[1]);
	}
};

#endif
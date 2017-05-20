#ifndef __SERVO_H
#define __SERVO_H
#include "ebox.h"

//舵机驱动
//支持端口：    
//TIM1 PA8  PA9  PA10 PA11 
//TIM2 PA0  PA1  PA2  PA3
//TIM3 PA6  PA7  PA0  PA1
//TIM4 PB6  PB7  PB8  PB9
class Servo
{
	Pwm pwm;
	uint32_t frq;
	uint16_t limLow,limHigh;

	template<typename T>
	static void limit(T &num, T limL, T limH);

public:

	//构建舵机对象，默认频率50Hz
	//标准50Hz时，占空比范围3%~13%
	//标准舵机高电平范围0.5ms~2.5ms
	//此处取limLowMs=0.6ms~limHighMs=2.4ms，留一些余量
	Servo(Gpio* pin, uint32_t frequency = 50, float limLowMs = 0.6, float limHighMs = 2.4);

	//初始化pwm
	void begin();

	//设置百分比
	void setPct(float percent);
};

template<typename T>
void Servo::limit(T &num, T limL, T limH)
{
	if (num < limL)
	{
		num = limL;
	}
	else if (num > limH)
	{
		num = limH;
	}
}

#endif
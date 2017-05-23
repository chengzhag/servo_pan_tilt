/**
******************************************************************************
* @file   : *.cpp
* @author : shentq
* @version: V1.2
* @date   : 2016/08/14

* @brief   ebox application example .
*
* Copyright 2016 shentq. All Rights Reserved.
******************************************************************************
*/

#include "ebox.h"
#include "uart_num.h"
#include "led.h"
#include "servo_pan_tilt.h"
#include "mpu9250.h"

Led led(&PC13, 1);
ServoPanTilt panTilt(&PB8, &PB9, 0.01);

void setup()
{
	ebox_init();
	led.begin();
	panTilt.begin();
	//panTilt.reset();
}

float angleY, angleP, angleR;
int main(void)
{
	setup();
	uint64_t time=0;

	while (1)
	{
		led.toggle();
		delay_ms(8);
		panTilt.refresh();

		//ÏÔÊ¾Ë¢ÐÂÂÊ
		uart1.printf("fps:%.1f\t", 1.0 / (millis() - time) * 1000);
		time = millis();
	}

}



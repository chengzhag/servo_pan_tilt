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
ServoPanTilt panTilt(&PB8, &PB9, 0.02);

void setup()
{
	ebox_init();
	led.begin();
	panTilt.begin();
	//panTilt.reset();
}

int main(void)
{
	setup();

	while (1)
	{
		led.toggle();
		delay_ms(20);
		panTilt.refresh();
	}

}



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
#include "led.h"
#include "servo.h"

Led led(&PC13, 1);
Servo servo(&PB9, 100);
void setup()
{
	ebox_init();
	uart1.begin(9600);
	led.begin();
	servo.begin();
}
int main(void)
{
	setup();
	float i = 100;
	while (1)
	{
		//²âÊÔ¶æ»ú·¶Î§
		i=i-0.1;
		if (i < 0)
		{
			i = 100;
		}
		servo.setPct(i);

		uart1.printf("%f\r\n", i);
		delay_ms(10);
		led.toggle();
	}

}



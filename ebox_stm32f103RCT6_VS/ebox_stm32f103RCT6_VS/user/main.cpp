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

UartNum uartNum(&uart2);
Led led(&PC13, 1);
ServoPanTilt panTilt(&PB8, &PB9, &uart2, 0.02);

void setup()
{
	ebox_init();
	uartNum.begin(500000);
	uart1.begin(9600);
	led.begin();
	panTilt.begin();
	//panTilt.reset();
}

int main(void)
{
	setup();

	while (1)
	{
		//uartNum.printf("ok\n");
		led.toggle();
		delay_ms(20);
		panTilt.refresh();
		//uart1.printf("%d: ", uartNum.recievedNum);
		//if (uartNum.recievedNum == 3)
		//{
		//	for (int i = 0; i < uartNum.recievedNum; i++)
		//	{
		//		uart1.printf("%f ", uartNum.numsBuf[i]);
		//	}
		//}
		//uart1.printf("\r\n");
	}

}



#include "uart_num.h"

void UartNum::rxEvent()
{
	uint8_t c;
	c = uart->read();
	if (isBegin)
	{
		recievedNum = 0;
		charBufIndex = 0;
		numBufIndex = 0;
		isTrans = false;
		isBegin = false;
	}
	//不在反转义状态
	if (!isTrans)
	{
		if (c == '\\')
		{
			isTrans = true;
			return;
		}
		else if (c == '\n')
		{
			//当未进入反转义时，遇到'\n'即结束该帧的结束
			recievedNum = numBufIndex;
			isBegin = true;

#ifdef __UART_NUM_DEBUG
			printf("转换结果%d个：", recievedNum);
			for (int j = 0; j < recievedNum; j++)
			{
				printf("%f ", numsBuf[j]);
			}
			printf("\n");
#endif
			return;
		}
		else
		{
			//当前位是数据位
			charsBuf.c[charBufIndex] = c;
			charBufIndex++;
		}
	}
	//在反转义状态
	else
	{
		//当前位是数据位
		charsBuf.c[charBufIndex] = c;
		charBufIndex++;
		isTrans = false;
	}

	//判断已接收数据位个数，如果单个数据接收完毕，存储数据
	if (charBufIndex >= UART_NUM_BUF_SIZE_CHAR)
	{
		if (numBufIndex >= UART_NUM_BUF_SIZE_NUM)
		{
			recievedNum = charBufIndex;
			printf("Num buffer overflow!!!\n");
			isBegin = true;
		}
		numsBuf[numBufIndex] = charsBuf.f;
		numBufIndex++;
		charBufIndex = 0;
	}
}

UartNum::UartNum(Uart *uartX) :
	uart(uartX),
	isTrans(false),
	charBufIndex(0),
	numBufIndex(0),
	//isBusy(false),
	recievedNum(0),
	isBegin(true)
{

}

void UartNum::begin(uint32_t baud_rate, uint8_t data_bit, uint8_t parity, float stop_bit, uint8_t _use_dma)
{
	uart->begin(baud_rate, data_bit, parity, stop_bit, _use_dma);
	uart->attach(this, &UartNum::rxEvent, RxIrq);
}

void UartNum::begin(uint32_t baud_rate, uint8_t _use_dma /*= 1*/)
{
	uart->begin(baud_rate, _use_dma);
	uart->attach(this, &UartNum::rxEvent, RxIrq);
}

void UartNum::printf(const char *fmt, ...)
{
	uart->printf(fmt);
}

#ifndef __UART_NUM
#define __UART_NUM

#include "ebox.h"

//#define __UART_NUM_DEBUG

#define UART_NUM_BUF_SIZE_CHAR 4
#define UART_NUM_BUF_SIZE_NUM 10

class UartNum
{
	Uart *uart;
	//用于存储接收数据位，转换为数字
	union num
	{
		float f;
		unsigned char c[UART_NUM_BUF_SIZE_CHAR];
	}charsBuf;

	int charBufIndex;//接收数据位的index
	int numBufIndex;//接收数字的index
	bool isTrans;//标志上一次是否是转义字符
	
	//bool isBusy;//标志是否正在接收
	bool isBegin;//标志是否处于帧头

	void rxEvent();
public:
	float numsBuf[UART_NUM_BUF_SIZE_NUM];//用于存储接收的数据
	int recievedNum;
	//构建基于uartX的UartNum类
	UartNum(Uart *uartX);
	//初始化uart、绑定中断函数
	void begin(uint32_t baud_rate, uint8_t _use_dma = 1);
	//初始化uart、绑定中断函数
	void begin(uint32_t baud_rate, uint8_t data_bit, uint8_t parity, float stop_bit, uint8_t _use_dma);
	void printf(const char *fmt, ...);
	////获取接收数据
	//void getNums(float* nums)
	//{
	//	for (int i = 0; i < recievedNum; i++)
	//	{
	//		nums[i] = numsBufOld[i];
	//	}
	//}
	////设置为正在接收
	//void setBusy()
	//{
	//	isBusy = true;
	//}
	////是否正在接收
	//bool busy()
	//{
	//	return isBusy;
	//}
};

#endif
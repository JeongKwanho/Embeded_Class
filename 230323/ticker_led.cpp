#include “mbed.h”

Ticker tic;
DigitalOut led1(LED1);
DigitalOut led2(D7);

void cb()
{
	led2 = !led2;
}

int main()
{
	tic.attach(&cb, 0.1);

	while(1)
	{
		led1 = !led1;
		wait(1);
	}
}

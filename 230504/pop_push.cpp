#include "mbed.h"
#define BUF_SIZE 10

DigitalOut led(LED1);
CircularBuffer<char, BUF_SIZE>buff;
RawSerial pc(USBTX, USBRX);
Ticker tickman;

void dataArrived()
{
    if(pc.readable())
	{
        charc =pc.getc();
        if(!buff.full()) buff.push(c);
	}
}

void pusher()
{
    if(!buff.full()) buff.push('!');
}

int main()
{
    tickman.attach(&pusher,1.0);
    pc.attach(&dataArrived);
    charc;

    while(true)
	{
        if(!buff.empty())
		{
            buff.pop(c);
            pc.printf("size: %u got: '%c'",buff.size(),c);
		}
		led=buff.full()?1:0;
       wait(0.5);
	}
}

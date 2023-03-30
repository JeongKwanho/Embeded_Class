#include "mbed.h"

Timeout tmo;
Ticker tic;
DigitalOut led1(LED1);
DigitalOut led2(D7);

void flip()
{
    led1 =! led1;
}

void off()
{
    led2 = 0;

    tic.detach();
}


Timer watch;

int main()
{
    tic.attach(&flip, 0.02);
    tic.attach(&off, 10);

    while(1)
    {

    }
}

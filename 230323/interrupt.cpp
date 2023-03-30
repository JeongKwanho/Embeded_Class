#include "mbed.h"

DigitalOut led1(LED1);
DigitalOut led2(D7);
InterruptIn btn(USER_BUTTON);

void flip()
{
    led1 = !led1;
}

int main()
{
    btn.fall(&flip);

    while(1)
    {
        led2 = !led2;
        
        wait(0.5);
    }
}

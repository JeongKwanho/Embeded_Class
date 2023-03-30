#include "mbed.h"

Ticker tic;
DigitalOut led1(LED1);
DigitalOut led2(D7);

class Blinker
{
    DigitalOut led;
    Ticker tic;

    void blink()
    {
        led = !led;
    }

    public:
    Blinker(PinName pin):led(pin){}

    void begin(float s)
    {
        tic.attach(callback(this, &Blinker::blink), s)
    }
}

int main()
{
    Blinker led1(LED1), led2(D7);

    led1.begin(0.1);
    led2.begin(0.02);

    while(true){}
}

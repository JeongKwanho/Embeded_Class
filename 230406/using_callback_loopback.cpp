#inlcude "mbed.h"

Serial pc(USBTX, USBRX, 115200);
DigitalOut led(LED1);

void ISR()
{
    if(pc.readable())
    {
        pc.putc(pc.getc());
    }
}

int main()
{
    pc.printf("Loopback test!!!\n");
    pc.attach(&ISR);

    while(1)
    {
        led =! led;
        wait(0.5);
    }
}

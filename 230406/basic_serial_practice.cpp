#inlcude "mbed.h"

Serial pc(USBTX, USBRX, 115200);

int main()
{
    pc.printf("Loopback test!!!\n");

    while(1)
    {
        if(pc.readable())
        {
            pc.putc(pc.getc());
        }
    }
}

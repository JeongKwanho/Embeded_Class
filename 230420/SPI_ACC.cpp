#include "mbed.h"
#define WHO_AM_I 0x75
#define ACC_L 0x3c
#define ACC_H 0x3b
#define SPI_READ 0x80

Serial pc(USBTX, USBRX, 115200);

class dum
{
    SPI _spi;
    DigitalOut _ss;

    public:
    dum(PinName mosi, PinName miso, PinName clk, PinName ss):
    _spi(mosi, miso, clk), _ss(ss)
    {
        _ss = 1;
        _spi.format(8, 3);
        _spi.frequency(15000000);
    }

    uint8_t readByte(uint8_t address)
    {
        _ss = 0;
        _spi.write(address | SPI_READ);
        int data = _spi.write(0x33);
        _ss = 1;
        
        return data;
    }
};


// main() runs in its own thread in the OS
int main()
{
    while(true)
    {
        int acc_l = dum.readByte(ACC_L);
        int acc_h = dum.readByte(ACC_H);
        
        int acc = (acc_h << 8) + acc_l;
        
        pc.print("%d\n", acc);
        wait(1);
    }
}

#include "mbed.h"
#include "Adafruit_SSD1306.h"

template<class T>
T map(T x, T in_min, T in_max, T out_min, T out_max);
void turn(PwmOut &rc, float deg);

int sw = 0;

typedef enum Action
{
    SHRINK = 1, UP, EXTEND, DOWN
}Action;

I2C i2c(I2C_SDA, I2C_SCL);
Adafruit_SSD1306_I2c myOled(i2c, D4, 0x78, 32, 128);
Ticker tic;
DigitalOut LED(D7);
InterruptIn btn(D5, PullUp);
PwmOut rcServo(D6);

void btn_toggle()
{
    sw = 1;
}

void toggle()
{
    LED = !LED;
}

void turn(PwmOut &rc,float deg){
    uint16_t pulseW=map<float>(deg,0.,180.,600.,2400.);
    rc.pulsewidth_us(pulseW);
}

template <class T>
T map(T x, T in_min, T in_max, T out_min, T out_max){
    return (x - in_min) * (out_max - out_min)
        / (in_max - in_min) + out_min;
}

int main()
{
    float angle = 0.;
    float inc = 1.;

    btn.rise(&btn_toggle);
    rcServo.period_ms(10);
    i2c.frequency(400000);
    myOled.begin();
    Action action = SHRINK;

    turn(rcServo, 0);

    while(true)
    {
        switch(action)
        {
            case SHRINK:
                angle = 0.;
                tic.detach();
                turn(rcServo, 0);
                LED = 0;
                myOled.printf("SHRINK       \r");
                myOled.display();

                if(sw == 1)
                {
                    tic.attach(&toggle, 0.1);
                    action = UP;
                    sw = 0;
                }
                break;

            case UP:
                myOled.printf("GOING UP         \r");
                myOled.display();

                turn(rcServo, angle);
                wait_ms(20);
                angle += inc;
                
                if(angle > 180.f)
                {
                    angle = 180;
                    action = EXTEND;
                }

                if(sw == 1)
                {
                    turn(rcServo, 180);
                    angle = 180;
                    wait_ms(350);
                    sw = 0;
                }

                break;

            case EXTEND:
                tic.detach();
                turn(rcServo, 180);
                LED = 1;
                myOled.printf("EXTEND       \r");
                myOled.display();

                if(sw == 1)
                {
                    tic.attach(&toggle, 0.1);
                    action = DOWN;
                    sw = 0;
                }
                break;

            case DOWN:
                myOled.printf("GOING DOWN       \r");
                myOled.display();

                turn(rcServo, angle);
                wait_ms(20);
                angle -= inc;

                if(angle < 0.f)
                {
                    angle = 0;
                    action = SHRINK;
                }

                if(sw == 1)
                {
                    turn(rcServo, 0);
                    angle = 0;
                    wait_ms(350);
                    sw = 0;
                }

                
                break;
        }
    }
}

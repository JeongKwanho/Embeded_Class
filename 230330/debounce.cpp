#include "mbed.h"

class Debounce
{
    InterruptIn btn;
    Timeout tmo;
  
    float interval;
    volatile bool state, ready;
  
    void push()
    {
       if (ready)
       {
            ready=false;
            tmo.attach_us(callback(this,&Debounce::check),interval);
       }
    }
  
    void check()
    {
        tmo.detach();
        state=btn;
        ready=true;
    }
   public:
    Debounce(PinName pin,int intv=20000,PinMode m=PullNone):btn(pin,m)
    {
        interval=intv; state=1; ready=true;
        btn.fall(callback(this,&Debounce::push));
    }
    bool read()
    {
        bool temp=state;
        state=1;
        return temp;
    }
    operator int()
    {
        return read();
    }
};

int main()
{
    Debounce db(USER_BUTTON,PullNone);
    int cnt=0;
  
    while(1)
    {
        if (!db)
        {
            cnt++;
            printf("count = %d\n",cnt);
        }
    }
}

#include <mbed.h>

PwmOut lamp(LED1);

int main()
{
	int cnt = 0;
	lamp.period_us(250);

	while(1)
	{
		float v = (float)(cnt%101)/100;
		
		lamp = v;
		wait_ms(100);

		cnt++;
	}
}

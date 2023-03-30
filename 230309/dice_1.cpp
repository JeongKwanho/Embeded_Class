#include "mbed.h"

DigitalOut led(LED1);
DigitalIn but(D2, PullUp);

int main()
{
	srand(time(0));
	
	while(1)
	{
		if(!but)
		{
			printf("%d\n", rand()%6+1);
			wait_ms(100);
		}
	}
}

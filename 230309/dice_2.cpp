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
			printf("%c", '[');

			for(int i = 0; i < rand()%6 + 1; i++)
			{
				printf("%c", '.');
			}
			printf("%c", ']');
			wait_ms(100);
		}
	}
}

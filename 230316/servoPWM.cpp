#include "mbed.h"

PwmOut rcServo(D6);

int main()
{
	float ang = 0., inc = 0.1;
	
	rcServo.period_ms(10);
	rcServo.pulsewidth_us(600);

	while(1)
	{
		uint16_t pw = ang*10 + 600;
		reServo.pulsewidth_us(pw);
		wait_ms(10);
		ang += inc;

		if(ang>180.f || ang < 0.f)
		{
			inc = -inc;
		}
	}
}

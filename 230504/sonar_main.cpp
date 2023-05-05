int main()
{
    son.start();

    while(true)
	{
       if(son.inUpdated())
		{
            pc.printf("%7.2f cm", Sonar.read());
		}
		led = !led;
       wait_ms(20);
	}
}

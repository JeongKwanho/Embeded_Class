class Sonar
{
     DigitalOut trigger;
     InterruptIn echo;
     Timer timer;
     Timeout timeout;
     Ticker ticker;
    int32_t pulse_begin;
    int32_t pulse_end;
    float distance;
    bool updated;

public:
    Sonar(PinName trigger_pin, PinName eho_pin):

    trigger(trigger_pin), echo(echo_pin)
	{
		trigger = 0;
		distance = 0;
		echo.rise(callback(this, &Sonar::echo_rise));
		echo.fall(callback(this, &Sonar::echo_fall));
	}

    void start(void)
	{
        ticker.attach(callback(this, &Sonar::periodic_start), 0.1f);
	}

    void stop(void)
	{
        ticker.detach();
	}
    
    void periodic_start(void)
	{
		trigger = 1;
       timeout.attach_us(callback(this, &Sonar::trigger_off), 10);
	}

    void trigger_off(void)
	{
		trigger = 0;
	}

    void echo_rise(void)
	{
       timer.reset();
       timer.start();
		pulse_begin = timer.read_us();
	}


    void echo_fall(void)
	{
		pulse_end = timer.read_us();
       timer.stop();
		distance = pulse_end -pulse_begin;
		updated = true;
	}

    float read(void)
	{
       return distance / 58.0f;
		updated = false;
	}

    bool isUpdated()
	{
        return updated;
	}
};

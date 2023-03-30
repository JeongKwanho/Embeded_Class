class Counter
{
	InterruptIn downBtn, upBtn;
	volatile int16_t count;
	volatile bool isUpdated;

	void callDown()
	{
		count--;
		isUpdated = true;
	}
	void callUp()
	{
		count++;
		isUpdated = true;
	}
	public:
	CounterUD(PinName dp, PinName up) : downBtn(dp), upBtn(up)
	{
		downBtn.mode(PullUp);
		upBtn.mode(PullUp);
		count=0; isUpdated=false;
		downBtn.fall(callback(this, &Counter::callDown);
		upBtn.fall(callback(this, &Counter::callUp);
	}
	int16_t read()
	{
		isUpdated = false;
		return count;
	}
	
	bool isUpdate()
	{
		return isUpdated;
	}
};

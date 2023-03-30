class Counter
{
	InterruptIn _inter;
	volatile uint16_t count;
	volatile bool isUpdated;

	void cb()
	{
		count++;
		isUpdated=true;
	}

	public:
	Counter(PinName p) : _inter(p)
	{
		count=0; isUpdated=false;
		_inter.fall(callback(this, &Counter::cb);
	}
	
	uint16_t read()
	{
		isUpdated = false;
		return count;
	}
	
	bool isUpdate()
	{
		return isUpdated;
	}
};

counter cnt(USER_BUTTON);

int main()
{
    if(cnt.isUpdate())
    {
        printf("cout : %d\n", cnt.read());
    }
}

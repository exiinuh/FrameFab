#include "Timer.h"

Timer::Timer()
{
	Reset();
}


Timer::~Timer()
{
}


void Timer::Start()
{
	start_time_ = std::chrono::system_clock::now();
}


void Timer::Stop()
{
	end_time_ = std::chrono::system_clock::now();
	sum_time_ += (end_time_ - start_time_).count();
	count_++;
}


void Timer::Reset()
{
	sum_time_ = 0;
	count_ = 0;
}


void Timer::Print()
{
	printf(" total-time:%12.2lf   count:%4d   avg-time:%10.2lf\n", 
		sum_time_, count_, sum_time_ / count_);
}


std::string Timer::ToString() const
{
	std::string s = std::to_string(sum_time_);
	s.append(" s");
	return s;
}


std::ostream & operator << (std::ostream& os, const Timer& t)
{
	os << t.ToString();
	return os;
}
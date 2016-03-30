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
}


void Timer::Reset()
{
	sum_time_ = 0;
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
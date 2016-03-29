#include "Timer.h"

Timer::Timer()
{
	;
}

void Timer::start()
{
	startTime = std::chrono::system_clock::now();
}

void Timer::stop()
{
	endTime = std::chrono::system_clock::now();
}

std::string Timer::to_string() const
{
	Duration d = endTime - startTime;
	std::string s = std::to_string(d.count());
	s.append(" s");
	return s;
}

std::ostream & operator << (std::ostream& os, const Timer& t)
{
	os << t.to_string();
	return os;
}
#pragma once

#ifndef FIBER_PRINT_TIMER_H
#define FIBER_PRINT_TIMER_H

#include <chrono>
#include <ctime>
#include <iostream>
#include <string>

class Timer
{
public:
	typedef std::chrono::time_point<std::chrono::system_clock> TimePoint;
	typedef std::chrono::duration<double> Duration;

	Timer();
	void start();
	void stop();
	std::string to_string() const;

private:
	TimePoint startTime;
	TimePoint endTime;
};

std::ostream& operator << (std::ostream & os, const Timer& t);


#endif //Timer.h
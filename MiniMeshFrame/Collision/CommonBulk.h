#pragma once

#include"BaseBulk.h"


class CommonBulk: public BaseBulk
{
public:
	CommonBulk(); 
	CommonBulk(ExtruderCone *extruder, point start, point end);
	~CommonBulk();

public:
	double			Angle(point p);								//this point can not be start or end point 
	bool				Inside(point p);
	point			AboveDownCol(point p);
	bool				IfAboveDownCol(point p);
	bool				IfAboveUpCol(point p);
	point			AboveUpCol( point p);


	point			BelowDownCol(point p);
	bool				IfBelowDownCol( point p);
	bool				IfBelowUpCol(point p);
	point			BelowUpCol(point p);

public:
	Parallelogram			*Check_top_;
	Triangle					*Check_top_left_;
	Triangle					*Check_top_right_;


	/*
	Triangle		front_;									// 0
	Triangle		back_;									// 1
	Triangle		corner_start_right_;				// 2
	Triangle		corner_start_left_;					// 3
	Triangle		corner_end_right_;				// 4
	Triangle		corner_end_left_;					// 5

	Triangle       top_left_;                              //6
	Triangle       top_right_;                           //7

    Parallelogram slider_;                              //8
	Parallelogtam slider_left_;						//9
	Parallelogtam slider_right_;                     //10

	Parallelogtam top_;                                 //11

	Parallelogram	right_;							   // 12
	Parallelogram	left_;								   // 13
	*/
};


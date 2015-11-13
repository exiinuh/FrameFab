#pragma once

#include <GTEnginePCH.h>
#include <Mathematics/GteDistSegment3Triangle3.h>
#include <Mathematics/GteIntrSegment3Cone3.h>

#include "Polyface.h"
#include "Triangle.h"
#include "Parallelogram.h"
#include "ExtruderCone.h"


class Bulk
{
public:
	Bulk(); 
	Bulk(ExtruderCone *extruder, point start, point end);
	~Bulk();

public:
	Polyface	*Face(int i);
	point		StartPoint();
	point		EndPoint();
	double		Angle(point p);													//this point can not be start or end point 

	point		TriIntersect(int i, point target_start, point target_end);
	point		ParaIntersect(int i, point target_start, point target_end);
	bool		IfTriIntersect(int i, point target_start, point target_end);
	bool		IfParaIntersect(int i, point target_start, point target_end);

	gte::Segment<3, float>	Segement_(point target_start, point target_end);
	gte::Triangle<3, float> Triangle_(Polyface *face);

   
private:


	
	ExtruderCone		*extruder_;

	point				start_;
	point				end_;

	GeoV3				vector_t_;
	GeoV3				vector_z_;
	GeoV3				vector_tz_;
	GeoV3				vector_tzz_;

public: 
	vector<Polyface*>	face_list_;

	

	/*
	Triangle		front_;								// 0
	Triangle		back_;								// 1
	Triangle		corner_start_right_;				// 2
	Triangle		corner_start_left_;					// 3
	Triangle		corner_end_right_;					// 4
	Triangle		corner_end_left_;					// 5

	Triangle		top_right_t0_;						// 6
	Triangle		top_right_t1_;						// 7
	Triangle		top_left_t0_;						// 8
	Triangle		top_left_t1_;						// 9

	Parallelogram	right_;								// 10
	Parallelogram	left_;								// 11
	Parallelogram	top_;								// 12
	*/
};


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
	vector<Polyface*>	face_list_;
	ExtruderCone		*extruder_;

	point				start_;
	point				end_;

	GeoV3				vector_t_;
	GeoV3				vector_z_;
	GeoV3				vector_tz_;
	GeoV3				vector_tzz_;
};


#pragma once

#include <GTEnginePCH.h>
#include <Mathematics/GteDistSegment3Triangle3.h>
#include <Mathematics/GteIntrSegment3Cone3.h>

#include "Polyface.h"
#include "Triangle.h"
#include "Parallelogram.h"
#include "ExtruderCone.h"

const double MAX = 1000000;
const double eps = 0.0001;

class BaseBulk
{
public:
	BaseBulk(){};
	~BaseBulk(){};

public:
	Polyface		*Face(int i);
	point			StartPoint();
	point			EndPoint();
	//this point can not be start or end point 

	point		TriIntersect(int i, point target_start, point target_end);
	point		ParaIntersect(int i, point target_start, point target_end);


	bool			IfTriIntersect(int i, point target_start, point target_end);
	bool			IfParaIntersect(int i, point target_start, point target_end);
	bool			IfLine2Tri(point o, GeoV3 v, Triangle* face);
	point		Line2Tri(point o, GeoV3 v, Triangle* face);
	bool			IfLine2Para(point o, GeoV3 v, Parallelogram* face);
	point		Line2Para(point o, GeoV3 v, Parallelogram* face);


	gte::Segment<3, float>		Segement_(point target_start, point target_end);
	gte::Triangle<3, float>			Triangle_(Polyface *face);
	gte::Line<3, float>				Line_(point p, GeoV3 vector);

	virtual double			Angle(point p){ return  0; };
	virtual bool				Inside(point p){ return 1; };
	virtual point				AboveDownCol(point p) { return point(0, 0, 0); };
	virtual bool				IfAboveDownCol(point p){ return 1; };
	virtual bool				IfAboveUpCol(point p){ return 1; };
	virtual point				AboveUpCol(point p) { return point(1, 0, 0); };


	virtual point				BelowDownCol(point p){ return point(0, 0, 0); };
	virtual bool				IfBelowDownCol(point p){ return 1; };
	virtual bool				IfBelowUpCol(point p){ return 1; };
	virtual point				BelowUpCol(point p){ return point(0, 0, 0); };

	point						ParallelCol(point p);

	void							Print();

public:
	int							flag;                        //CommonBulk ==1, SpecialBulk==2
	ExtruderCone			*extruder_;
	
	point						start_;
	point						end_;

	GeoV3						vector_t_;
	GeoV3						vector_z_;
	GeoV3						vector_tz_;
	GeoV3						vector_tzz_;

	vector<Polyface*>	face_list_;

	Parallelogram			*Check_top_;
	Triangle					*Check_top_left_;
	Triangle					*Check_top_right_;
};


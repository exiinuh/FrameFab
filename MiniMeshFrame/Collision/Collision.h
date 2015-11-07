#pragma once
#include <iostream>
#include <vector>
#include <math.h>

#include "WireFrame\WireFrame.h"
#include "FiberPrint\DualGraph.h"
#include "Geometry.h"
#include "Bulk.h"


typedef Geometry::Vector3d GeoV3;

const double MAX = 1000000;
const double eps = 0.0001;


typedef struct Range
{
	double right_begin;
	double right_end;
	double left_begin;
	double left_end;
};


class Collision
{
public:

	Collision();
	Collision(WireFrame  *ptr_frame, DualGraph *ptr_dualgraph);
	//Collision(ExtruderCone extruder, point start, point end);
	~Collision();

public:
	void	DetectFrame();
	int		DetectCollision(Bulk *bulk, point target_start, point target_end);  //0 no intersection ;1 some intersection; 2 always intersection; -1 bug

	/*
	bool	IfIntersect(Triangle face, point start, point end);
	bool	IfIntersect(Parallelogram face, point start, point end);
	point	Intersect(Triangle face, point start, point end);
	point	Intersect(Parallelogram face, point start, point end);
	bool	Inside(point test);
	*/

	bool	Inside(Bulk *bulk, point p);
	bool	JointAngle(double angle_1, double angle_2);
	//double	Distance(Triangle face, point start, point end);
	//double	Distance(Parallelogram face, point start, point end);

	bool	CheckPoint(point temp, vector<point>collision_point);		//check is there the same point in the list

	void	ConeSegementTest();
	void	SegementTriangleTest();

	void	Test();
	void	Print();
	void	Print(point i){ cout << i.x() << " " << i.y() << " " << i.z() << " " << endl; }
	void	Print(Range a){ cout << a.right_begin / pi * 180 << " " << a.right_end / pi * 180 << " " <<
									a.left_begin / pi * 180 << " " << a.left_end / pi * 180 << endl; }

	inline double Min(double min_0, double min_1, double min_2, double min_3)
	{
		double min = min_0;
		if (min > min_1)
			min = min_1;
		if (min > min_2)
			min = min_2;
		if (min > min_3)
			min = min_3;
		return min_3;
	}
	inline double	Min(double x, double y){ return (x > y) ? y : x; }
	inline double	Max(double x, double y){ return (x > y) ? x : y; }
	inline bool		Equal(point x, point y){ return((x - y).length() < eps) ? true : false; }
	inline bool		Equal(double x, double y){ return(abs(x - y) < eps) ? true : false; }

	vector<vector<Range*>>	GetRangeList(){ return range_list_; }
	vector<vector<int>>		GetRangeState(){ return range_state_; }

private:
	ExtruderCone	*extruder_;

	vector<point>	collision_point_;
	vector<int>		collision_state_;
	Range			allowed_angle_;
	
public:
	//input
	WireFrame		*ptr_frame_;
	DualGraph		*ptr_dualgraph_;

	//output  by Test()
	vector<vector<Range*>>      range_list_; //           
	vector<vector<int>>			range_state_;         // -1- vertical pi/2; 0 all angle; 1 some angle; 2  no angle 
	vector<Bulk*>				bulk_list_;
};
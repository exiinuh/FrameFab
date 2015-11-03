#pragma once
#include <iostream>
#include <vector>
#include <math.h>

#include <GTEnginePCH.h>
#include <Mathematics/GteDistSegment3Triangle3.h>
#include <Mathematics/GteIntrSegment3Cone3.h>

#include "GTEngine.h"
#include "WireFrame\WireFrame.h"
#include "FiberPrint\DualGraph.h"
#include "GTEngine.h"
#include "ExtruderCone.h"
#include "Geometry.h"
#include "Triangle.h"
#include "Parallelogram.h"


using namespace std;
using namespace Geometry;

typedef Geometry::Vector3d GeoV3;

const double MAX = 1000000;
const double EPS = 0.0001;


typedef struct Range
{
	double right_begin;
	double right_end;
	double left_begin;
	double left_end;
};

typedef struct Bulk
{
	GeoV3 start_;
	GeoV3 end_;
	GeoV3 front_face_right_;
	GeoV3 front_face_left_;
	GeoV3 back_face_right_;
	GeoV3 back_face_left_;
	GeoV3 start_left_;
	GeoV3 end_left_;
	GeoV3 start_right_;
	GeoV3 end_right_;
};



class Collision
{
public:

	Collision();
	Collision(ExtruderCone extruder, point start, point end);
	~Collision();

public:
	void	DetectFrame();
	int		DetectCollision();											// 0 no intersection ;1 some intersection; 2 always intersection; -1 bug
	void	GeneralFace();
	void	JudgeIntersection();

	bool	IfIntersect(Triangle face, point start, point end);
	bool	IfIntersect(Parallelogram face, point start, point end);
	point	Intersect(Triangle face, point start, point end);
	point	Intersect(Parallelogram face, point start, point end);

	bool	Inside(point test);
	bool	JointAngle(double angle_1, double angle_2);

	double	Angle(point intersection);									// this point can not be start or end point 
	double	Distance(Triangle face, point start, point end);
	double	Distance(Parallelogram face, point start, point end);

	bool	CheckPoint(point temp, vector<point>collision_point);		// check is there the same point in the list
	void	CheckConllisionlist();

	void	ConeSegementTest();
	void	SegementTriangleTest();

	gte::Segment<3, float>	Segement_(point target_start, point target_end);
	gte::Triangle<3, float> Triangle_(Triangle face);

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
	inline bool		Equal(point x, point y){ return((x - y).length() < EPS) ? true : false; }
	inline bool		Equal(double x, double y){ return(abs(x - y) < EPS) ? true : false; }

private:
	point			start_;
	point			end_;
	point			target_start_;
	point			target_end_;
	
	GeoV3			vector_t_;
	GeoV3			vector_z_;
	GeoV3			vector_tz_;
	GeoV3			vector_tzz_;

	ExtruderCone	extruder_;

	Triangle		front_;								// 0
	Triangle		back_;								// 1
	Triangle		corner_start_right_;				// 2
	Triangle		corner_start_left_;					// 3
	Triangle		corner_end_right_;					// 4
	Triangle		corner_end_left_;					// 5

	Parallelogram	left_;								// 6
	Parallelogram	right_;								// 7
	Parallelogram	top_;								// 8
	Parallelogram	top_left_;							// 9
	Parallelogram	top_right_;							// 10

	Triangle		top_left_t0;						// 11
	Triangle		top_left_t1;						// 12
	Triangle		top_right_t0;						// 13
	Triangle		top_right_t1;						// 14


	vector<point>	collision_point_;
	vector<int>		collision_state_;

	Range			allowed_angle_;
	Bulk            move_bulk;

	DualGraph		*ptr_dualgraph_;

public:
	//input
	WireFrame		*ptr_frame_;
	
	//output  by Test()
	vector<vector<Range*>>	range_list_;				//           
	vector<vector<int>>		range_state_;				// -1- vertical pi/2; 0 all angle; 1 some angle; 2  no angle 

};
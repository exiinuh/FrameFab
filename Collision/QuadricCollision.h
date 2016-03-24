#pragma once
#include "Collision\Geometry.h"
#include "Collision\ExtruderCone.h"
#include "Collision\Triangle.h"
#include "WireFrame\WireFrame.h"
#include "FiberPrint\DualGraph.h"

#include <vector>
#include <GTEnginePCH.h>
#include <Mathematics/GteIntrSegment3Cone3.h>

typedef unsigned long long lld;

using namespace Geometry;
using namespace std;

// ¦È=(0,180), ¦Õ=(0,360)


class QuadricCollision
{
public:
	QuadricCollision();
	QuadricCollision(WireFrame *ptr_frame);
	~QuadricCollision();

public:
	void	DetectCollision(WF_edge *target_e, DualGraph *ptr_subgraph); // 
	void	DetectCollision(WF_edge *target_e, WF_edge *order_e);

	void Init();
	void	Init(WF_edge *target_e);
	void Init(vector<lld> &angle_state);

private:
	void	DetectEdge(WF_edge *order_e);
	bool	DetectBulk(WF_edge *order_e, double ¦È, double ¦Õ);
	bool	DetectAngle(GeoV3 connect, GeoV3 end, GeoV3 target_end, GeoV3 normal);

	bool	Case(GeoV3 target_start, GeoV3 target_end,
				GeoV3 order_start, GeoV3 order_end, GeoV3 normal);
	
	bool SpecialCase(GeoV3 connect, GeoV3 target_s, GeoV3 order_s, GeoV3 normal);

	bool	ParallelCase(GeoV3 target_start, GeoV3 target_end,
				GeoV3 order_start, GeoV3 order_end, GeoV3 normal);



	bool	DetectCone(GeoV3 start, GeoV3 normal, GeoV3 target_start, GeoV3 target_end);
	bool	DetectCylinder(GeoV3 start, GeoV3 normal, GeoV3 target_start, GeoV3 target_end);
	bool	DetectTriangle(Triangle	 triangle, GeoV3 target_start, GeoV3 target_end);

	void	GenerateVolume(GeoV3 start, GeoV3 end, GeoV3 target_start, GeoV3 target_end, GeoV3 normal);
	void	GenerateVolume(GeoV3 connect, GeoV3 end, GeoV3 target_end, GeoV3 normal);

	bool	Parallel(GeoV3 a, GeoV3 b);

	gte::Segment<3, float>		Seg(point target_start, point target_end);
	gte::Segment<3, float>		Seg(GeoV3 target_start, GeoV3 target_end);
	gte::Triangle<3, float>		Tri(GeoV3 a, GeoV3 b, GeoV3 c);

	GeoV3 Orientation(double ¦È, double ¦Õ)
	{ 
		return GeoV3(sin(¦È)*cos(¦Õ), sin(¦È)*sin(¦Õ), cos(¦È)); 
	}

public:
	//output
	void AngleState(vector<lld> &angle_state)	
	{
		angle_state.resize(3);
		angle_state[0] = state_map_[0];
		angle_state[1] = state_map_[1];
		angle_state[2] = state_map_[2];
	}

	void ModifyAngle(vector<lld> &angle_state)
	{
		for (int i = 0; i < 3; i++)
		{
			angle_state[i] |= state_map_[i];
		}
	}

	int ColFreeAngle()
	{
		int sum_angle = 0;
		for (int j = 0; j < 62; j++)
		{
			lld mask = (1 << j);
			for (int i = 0; i < 3; i++)
			{
				if ((state_map_[i] & mask) == 0)
				{
					sum_angle++;
				}
			}
		}

		return sum_angle;
	}

	int ColFreeAngle(vector<lld> &angle_state)
	{
		int sum_angle = 0;
		for (int j = 0; j < 62; j++)
		{
			lld mask = (1 << j);
			for (int i = 0; i < 3; i++)
			{
				if ((angle_state[i] & mask) == 0)
				{
					sum_angle++;
				}
			}
		}

		return sum_angle;
	}

	int Divide()
	{
		return 18 * 18 + 2;
	}

	void			Debug();

public:
	WireFrame		*ptr_frame_;
	WF_edge			*target_e_;

private:
	ExtruderCone		extruder_;
	vector<Triangle>	bulk_;
	vector<lld>			state_map_;
	int					divide_;

};


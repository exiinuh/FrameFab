#pragma once
#include"Collision\Geometry.h"
#include "WireFrame\WireFrame.h"
#include"Collision\ExtruderCone.h"
#include <GTEnginePCH.h>
#include<Mathematics/GteIntrSegment3Cone3.h>
#include"Collision\Triangle.h"
#include "FiberPrint\DualGraph.h"
typedef unsigned long long lld;
using namespace Geometry;

// ��=(0,180), ��=(0,360)



class QuadricCollision
{


public:
	QuadricCollision();
	QuadricCollision(WireFrame *ptr_frame);
	~QuadricCollision();

public:
	void		DetectCollision(WF_edge *target_e, DualGraph *ptr_subgraph);
	void		DetectEdge(WF_edge *target_e,WF_edge *order_e);


private:
	//input
	GeoV3 start_, end_;                       //Printing edge
	GeoV3 target_start_, target_end_; // Considering collision edge in the structure
	double ��_, ��_;                                   //��_ means angle with Z axis, ��_ means angle with X axis; Rad

public:
	//output
	vector<lld>		Angle()			{ return state_map_; }
	int		ColFreeAngle();
	int		ColFreeAngle(vector<lld> state_map);


private:
	ExtruderCone extruder_;
	std::vector<Triangle > bulk_;
	GeoV3 normal_;
	bool ifcollision_;


public:

	bool Run(GeoV3 start, GeoV3  end, GeoV3 target_start, GeoV3  target_end, double ��, double ��);//true menas collision
	bool Run(GeoV3 connect, GeoV3 end, GeoV3 target_end, double ��, double ��);
	bool Run(GeoV3 start, GeoV3 end, GeoV3 target_start, GeoV3 target_end, GeoV3 normal);

	bool Run();
	void Debug();
private:
	int ClassfyModel(GeoV3 start, GeoV3  end, GeoV3 target_start, GeoV3  target_end, double ��, double ��);
	

	GeoV3 Orientation(double ��, double ��){ return GeoV3(sin(��)*cos(��), sin(��)*sin(��), cos(��)); };
	void GenerateVolume(GeoV3 start, GeoV3  end, GeoV3 target_start, GeoV3  target_end, double ��, double ��);
	void GenerateVolume(GeoV3 connect, GeoV3 end, GeoV3 target_end, GeoV3 normal);
	

	bool IfColCone(GeoV3 start, GeoV3 normal, GeoV3 target_start, GeoV3 target_end);
	bool IfColCylin(GeoV3 start, GeoV3 normal, GeoV3 target_start, GeoV3 target_end);
	bool IfColTri(Triangle	 triangle, GeoV3 target_start,GeoV3 target_end );
	bool IfParallet(GeoV3 a, GeoV3 b);

	bool IfColAngle(GeoV3 connect, GeoV3 end, GeoV3 target_end, GeoV3 normal);

	gte::Segment<3, float>		Segement_(point target_start, point target_end);
	gte::Segment<3, float>		Segement_(GeoV3 target_start, GeoV3 target_end);
	gte::Triangle<3, float>       Triangle_(GeoV3 a, GeoV3 b, GeoV3 c);
	
	
//similar to Collision.h
private:
	
	WireFrame			*ptr_frame_;
	WF_edge				*target_e_;
	vector<lld> state_map_;
	int    divide_;


private:
	void		Init(WF_edge *target_e);
	void		DetectEdge(WF_edge *order_e);
};


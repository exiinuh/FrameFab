#pragma once
#include"WireFrame\WireFrame.h"
#include"Collision\Geometry.h"
#include"Collision\ExtruderCone.h"


class ResolveAngle
{
public:
    ResolveAngle();
	ResolveAngle( vector< Geometry::Vector3d> list );
	~ResolveAngle();


	Geometry::Vector3d dec;
	double wave;
	vector< Geometry::Vector3d> a_;
	vector< Geometry::Vector3d> b_;
	vector< Geometry::Vector3d> c_;

	ExtruderCone extruder_;
	vector< Geometry::Vector3d> list_;
	vector< Geometry::Vector3d> Resolve();

	
	
	void Dec();

	Geometry::Vector3d Ave(vector<Geometry::Vector3d> t);
};


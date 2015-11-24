#pragma once
#include <windows.h>
#include <GLU.h>
#include <glut.h>
#include "Geometry.h"

using namespace std;

typedef Geometry::Vector3d GeoV3;


class Polyface
{
public:
	Polyface(){}
	~Polyface(){}

public:
	point Trans(GeoV3 V)
	{
		point v;
		v.x() = V.getX();
		v.y() = V.getY();
		v.z() = V.getZ();
		return v;
	}

	point v0()
	{
		return vert_list_[0];
	}

	point v1()
	{
		return vert_list_[1];
	}

	point v2()
	{
		return vert_list_[2];
	}

	point v3()
	{
		return vert_list_[3];
	}

	point Normal()
	{
		return normal_;
	}

	void Normal_()
	{
		GeoV3 normal;
		normal = Geometry::cross(v1() - v0(), v2() - v0());

		if (normal.norm() == 0)
			return;
		normal.normalize();
		normal_ = Trans(normal);
	}

	virtual void				Print() {}
	virtual void				Render(WireFrame* ptr_frame, double alpha) {}

public:
	vector<point>			vert_list_;
	point						normal_;
};


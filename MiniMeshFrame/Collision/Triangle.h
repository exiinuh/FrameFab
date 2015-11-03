#pragma once
#include"WireFrame\WireFrame.h"

typedef Geometry::Vector3d GeoV3;


class Triangle
{
public:
	Triangle() 
		: v0_(point{ 1, 0, 0 }), v1_(point{ 0, 1, 0 }), v2_(point{ 0, 0, 1 }) {}
	Triangle(point v0, point v1, point v2)
		: v0_(v0), v1_(v1), v2_(v2) {}
	Triangle(GeoV3 v0, GeoV3 v1, GeoV3 v2)
		: v0_(Trans(v0)), v1_(Trans(v1)), v2_(Trans(v2)) {}
	~Triangle(){};

public:
	point Trans(GeoV3 V)
	{
		point v; 
		v.x() = V.getX(); 
		v.y() = V.getY(); 
		v.z() = V.getZ(); 
		return v;
	}

	void Print()
	{
		std::cout << v0_.x() << ", " << v0_.y() << ", " << v0_.z() << std::endl;
		std::cout << v1_.x() << ", " << v1_.y() << ", " << v1_.z() << std::endl;
		std::cout << v2_.x() << ", " << v2_.y() << ", " << v2_.z() << std::endl;
	}

public:
	point	v0_;
	point	v1_;
	point	v2_;
};


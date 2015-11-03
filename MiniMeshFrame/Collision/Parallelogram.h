#pragma once
#include"WireFrame\WireFrame.h"
#include"Triangle.h"

typedef Geometry::Vector3d GeoV3;


class Parallelogram
{
public:
	Parallelogram()
		: v0_(point{ 0, 0, 0 }), v1_(point{ 1, 0, 0 }), v2_(point{ 1, 1, 0 }), v3_(point{ 0, 1, 0 }),
		t0_(Triangle(v2_, v0_, v1_)), t1_(Triangle(v3_, v0_, v2_)) {}
	Parallelogram(point v0, point v1, point v2, point v3)
		: v0_(v0), v1_(v1), v2_(v2), v3_(v3), t0_(Triangle(v2, v0, v1)), t1_(Triangle(v3, v0, v2)) {}
	Parallelogram(GeoV3 v0, GeoV3 v1, GeoV3 v2, GeoV3 v3)
		: v0_(Trans(v0)), v1_(Trans(v1)), v2_(Trans(v2)), v3_(Trans(v3)), 
		t0_(Triangle(v2, v0, v1)), t1_(Triangle(v3, v0, v2)) {}
	~Parallelogram(){}

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
		std::cout << v0_.x() << " ," << v0_.y() << ", " << v0_.z() << std::endl;
		std::cout << v1_.x() << " ," << v1_.y() << ", " << v1_.z() << std::endl;
		std::cout << v2_.x() << ", " << v2_.y() << " ," << v2_.z() << std::endl;
		std::cout << v3_.x() << ", " << v3_.y() << ", " << v3_.z() << std::endl;
	}

public:
	point		v0_;
	point		v1_;
	point		v2_;
	point		v3_;

	Triangle	t1_;
	Triangle	t0_;
};


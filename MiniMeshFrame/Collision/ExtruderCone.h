#pragma once
#include"WireFrame\WireFrame.h"

using namespace std;

static const double pi = 3.1415926;

class ExtruderCone
{
public:
	ExtruderCone();
	ExtruderCone(double height, point  base_point, Vec3f normal, double angle);
	~ExtruderCone();

public:
	void	Default();
	void	Test();

public:
	double	height_;
	point	base_point_;
	Vec3f	normal_;
	double	angle_;  
	double	wave_angle_;

};
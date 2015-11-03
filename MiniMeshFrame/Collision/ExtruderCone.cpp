#include"ExtruderCone.h"


ExtruderCone::ExtruderCone()
{
}


ExtruderCone::ExtruderCone(double height, point  base_point, Vec3f normal, double angle)
{
	height_ = height_;
	base_point_ = base_point_;
	normal_ = normal_;
	angle_ = angle_;
}


ExtruderCone::~ExtruderCone()
{
}


void ExtruderCone::Default()
{
	normal_ = Vec3f(0, 0, 1);
	angle_ = pi/ 3; //60 degree for extruder
	height_ = 10; // 1 means 10mm
	wave_angle_ = pi / 18;  // 10 degree for wave 
}


void ExtruderCone::Test()
{
	cout << angle_ << " " << normal_.x() << normal_.z()<<endl;
}
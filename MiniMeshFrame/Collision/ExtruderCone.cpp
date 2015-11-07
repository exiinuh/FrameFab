#include"ExtruderCone.h"


ExtruderCone::ExtruderCone()
{
	normal_ = Vec3f(0, 0, 1);
	angle_ = pi / 3;						//60 degree for extruder
	height_ = 10;						// 1 means 10mm
	wave_angle_ = pi / 18;				// 10 degree for wave 
}


ExtruderCone::ExtruderCone(double height, point  base_point, Vec3f normal, double angle)
{
	height_ = height;
	base_point_ = base_point;
	normal_ = normal;
	angle_ = angle;
}


ExtruderCone::~ExtruderCone()
{
}


double ExtruderCone::Height()
{
	return height_;
}


double ExtruderCone::Angle()
{
	return angle_;
}


double ExtruderCone::WaveAngle()
{
	return wave_angle_;
}


point ExtruderCone::BasePoint()
{
	return base_point_;
}


Vec3f ExtruderCone::Normal()
{
	return normal_;
}


void ExtruderCone::Test()
{
	cout << angle_ << " " << normal_.x() << normal_.z()<<endl;
}


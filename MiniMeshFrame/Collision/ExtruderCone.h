#pragma once
#include "GCommon.h"
#include "WireFrame\WireFrame.h"
#include "Triangle.h"

using namespace std;

class ExtruderCone
{
public:
	ExtruderCone();
	ExtruderCone(double height, point  base_point, Vec3f normal, double angle);
	~ExtruderCone();

public:
	double	Height();
	double	Angle();
	double	WaveAngle();
	point	BasePoint();
	Vec3f	Normal();
	void	Test();
	void Rotation(double angle,point start, point end);

	void RotateTri(Triangle temp);
    point Muti(point s);
	void Render(WireFrame* ptr_frame, double alpha);
private:
	double	angle_;
	double	height_;
	double	wave_angle_;
    vector<Triangle>  side_;

	vector<Triangle> side_end_;
	vector<point> top_;

	void GeneCone();
	int divide_;
	point	base_point_;
	Vec3f	normal_;
	float rotate_[4][4];
	point start_,end_;
	
};
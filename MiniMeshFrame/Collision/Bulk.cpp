#include "Bulk.h"


Bulk::Bulk()
{
	extruder_ = new ExtruderCone();
}


Bulk::Bulk(ExtruderCone *extruder, point start, point end)
{
	extruder_ = extruder;
	start_    = start;
	end_	  = end;

	// Currently, we only print upwards
	if (end.z() < start.z())
	{
		swap(start, end);
	}

	double height	  =	extruder_->Height();
	double angle	  =	extruder_->Angle();
	double wave_angle = extruder_->WaveAngle();
	double generatrix = height / cos(angle);
	double radii      =	height * tan(angle);

	// Printing edge direction
	vector_t_   = end - start;
	vector_t_.normalize();
	vector_z_   = point{ 0.0, 0.0, 1.0 };
	vector_tz_  = cross(vector_t_,  vector_z_);
	vector_tzz_ = cross(vector_tz_, vector_z_);

	GeoV3 _start = GeoV3(start);
	GeoV3 _end   = GeoV3(end);

	// Using pyramid to approximate cone
	// Triangle front;
	GeoV3 front_face_right = _start + vector_tzz_*(radii) + vector_z_*(height*cos(wave_angle))
								+ vector_tz_*(height*sin(wave_angle));
	GeoV3 front_face_left = _start + vector_tzz_*(radii)+vector_z_*(height*cos(wave_angle))
								+ vector_tz_*(-height*sin(wave_angle));
	face_list_.push_back(new Triangle(_start, front_face_right, front_face_left));

	// Triangle back;
	GeoV3 back_face_right = _end +    vector_tzz_*(-radii) + vector_z_*(height*cos(wave_angle))
								+ vector_tz_*(height*sin(wave_angle));
	GeoV3 back_face_left  = _end  +   vector_tzz_*(-radii) + vector_z_*(height*cos(wave_angle))
								+ vector_tz_*(-height*sin(wave_angle));
	face_list_.push_back(new Triangle(_end, back_face_left, back_face_right));


	GeoV3 start_right = _start + vector_z_*(generatrix*cos(angle + wave_angle))
		+ vector_tz_*(generatrix*sin(angle + wave_angle));
	GeoV3 end_right = _end + vector_z_*(generatrix*cos(angle + wave_angle))
		+ vector_tz_*(generatrix*sin(angle + wave_angle));
	GeoV3 start_left = _start + vector_z_*(generatrix*cos(angle + wave_angle))
		+ vector_tz_*(-generatrix*sin(angle + wave_angle));
	GeoV3 end_left   = _end   + vector_z_*(generatrix*cos(angle + wave_angle))
		+ vector_tz_*(-generatrix*sin(angle + wave_angle));

	// triangle corner_start_right 
	face_list_.push_back(new Triangle(_start, start_right, front_face_right));

	// triangle corner_start_left
	face_list_.push_back(new Triangle(_start, front_face_left, start_left));

	// triangle corner_end_right 
	face_list_.push_back(new Triangle(_end, back_face_right, end_right));

	// triangle corner_end_left;
	face_list_.push_back(new Triangle(_end, end_left, back_face_left));

	// Overtop Seal plane
	// parallelogram top_left
	face_list_.push_back(new Triangle(back_face_left, end_left, front_face_left));
	face_list_.push_back(new Triangle(front_face_left, end_left, start_left));

	// parallelogram top_right
	face_list_.push_back(new Triangle(back_face_right, front_face_right, end_right));
	face_list_.push_back(new Triangle(end_right, front_face_right, start_right));

	// parallelogram right;
	face_list_.push_back(new Parallelogram(start_right, _start, _end, end_right));

	// parallelogram left;
	face_list_.push_back(new Parallelogram(start_left, end_left, _end, _start));

	// parallelogram top
	face_list_.push_back(new Parallelogram(front_face_left, front_face_right, back_face_right, back_face_left));

	/*
	// triangle  top
	face_list_.push_back(new Triangle(back_face_left, end_left, front_face_left));
	face_list_.push_back(new Triangle(front_face_left, end_left, start_left));

	face_list_.push_back(new Triangle(back_face_right, front_face_right, end_right));
	face_list_.push_back(new Triangle(end_right, front_face_right, start_right));
	current_bulk_ = temp;
	return temp;
	*/
}


Bulk::~Bulk()
{
}


Polyface *Bulk::Face(int i)
{
	return face_list_[i];
}


point Bulk::StartPoint()
{
	return start_;
}


point Bulk::EndPoint()
{
	return end_;
}


double Bulk::Angle(point p)
{
	GeoV3 temp = GeoV3(p - start_);
	temp = temp - vector_t_*(Geometry::dot(temp, vector_t_));
	return Geometry::angle(temp, vector_tz_);
}

point Bulk::TriIntersect(int i, point start, point end)
{
	Polyface *face = face_list_[i];
	gte::FIQuery<float, gte::Segment3<float>, gte::Triangle3<float>> intersection;
	auto result = intersection(Segement_(start, end), Triangle_(face));
	point temp(result.point[0], result.point[1], result.point[2]);
	//temp.x = result.point[0]; temp.y = result.point[1]; temp.z() = result.point[2];
	return temp;
}


point Bulk::ParaIntersect(int i, point start, point end)
{
	Polyface *face = face_list_[i];
	gte::FIQuery<float, gte::Segment3<float>, gte::Triangle3<float>> intersection;
	auto result = intersection(Segement_(start, end), Triangle_(new Triangle(face->v2(), face->v0(), face->v1())));
	point temp;
	if (result.intersect)
	{
		point temp(result.point[0], result.point[1], result.point[2]);
		//temp.x = result.point[0]; temp.y = result.point[1]; temp.z() = result.point[2];
		return temp;
	}
	else
	{
		auto result = intersection(Segement_(start, end), Triangle_(new Triangle(face->v3(), face->v0(), face->v2())));
		point temp(result.point[0], result.point[1], result.point[2]);
		//temp.x = result.point[0]; temp.y = result.point[1]; temp.z() = result.point[2];
		return temp;
	}
}


bool Bulk::IfTriIntersect(int i, point start, point end)
{
	Polyface *face = face_list_[i];
	gte::FIQuery<float, gte::Segment3<float>, gte::Triangle3<float>> intersection;
	auto result = intersection(Segement_(start, end), Triangle_(face));
	return result.intersect;
}


bool Bulk::IfParaIntersect(int i, point start, point end)
{
	Polyface *face = face_list_[i];
	gte::FIQuery<float, gte::Segment3<float>, gte::Triangle3<float>> intersection;
	auto result = intersection(Segement_(start, end), Triangle_(new Triangle(face->v2(), face->v0(), face->v1())));
	if (result.intersect == 1)
	{
		return result.intersect;
	}
	result = intersection(Segement_(start, end), Triangle_(new Triangle(face->v3(), face->v0(), face->v2())));
	return result.intersect;
}


gte::Segment<3, float> Bulk::Segement_(point target_start, point target_end)
{
	gte::Segment<3, float> segment;
	segment.p[0][0] = target_start.x();
	segment.p[0][1] = target_start.y();
	segment.p[0][2] = target_start.z();
	segment.p[1][0] = target_end.x();
	segment.p[1][1] = target_end.y();
	segment.p[1][2] = target_end.z();
	return segment;
}


gte::Triangle<3, float>  Bulk::Triangle_(Polyface *face)
{
	gte::Triangle<3, float> triangle;
	triangle.v[0][0] = face->v0().x();
	triangle.v[0][1] = face->v0().y();
	triangle.v[0][2] = face->v0().z();
	triangle.v[1][0] = face->v1().x();
	triangle.v[1][1] = face->v1().y();
	triangle.v[1][2] = face->v1().z();
	triangle.v[2][0] = face->v2().x();
	triangle.v[2][1] = face->v2().y();
	triangle.v[2][2] = face->v2().z();
	return triangle;
}
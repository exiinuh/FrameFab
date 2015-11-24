#include "CommonBulk.h"


CommonBulk::CommonBulk()
{
	extruder_ = new ExtruderCone();
	flag = 1;
}


CommonBulk::CommonBulk(ExtruderCone *extruder, point start, point end)
{
	flag = 1;
	extruder_ = extruder;
	start_ = start;
	end_ = end;

	if (end.z() < start.z())
	{
		swap(start, end);
	}

	double height = extruder_->Height();
	double angle = extruder_->Angle();
	double wave_angle = extruder_->WaveAngle();
	double generatrix = height / cos(angle);
	double radii = height*tan(angle);

	vector_t_ = end - start;
	vector_t_.normalize();
	vector_z_ = point{ 0.0, 0.0, 1.0 };
	vector_tz_ = cross(vector_t_, vector_z_);
	vector_tz_.normalize();
	vector_tzz_ = cross(vector_tz_, vector_z_);
	vector_tzz_.normalize();
	GeoV3 _start = Geometry::Vector3d(start);
	GeoV3 _end = GeoV3(end);

	//triangle front; 
	GeoV3 front_face_right = _start + vector_tzz_*(radii)+vector_z_*(height*cos(wave_angle))
		+ vector_tz_*(height*sin(wave_angle));
	GeoV3 front_face_left = _start + vector_tzz_*(radii)+vector_z_*(height*cos(wave_angle))
		+ vector_tz_*(-height*sin(wave_angle));
	face_list_.push_back(new Triangle(_start, front_face_right, front_face_left));

	//triangle back;
	GeoV3 back_face_right = _end + vector_tzz_*(-radii) + vector_z_*(height*cos(wave_angle))
		+ vector_tz_*(height*sin(wave_angle));
	GeoV3 back_face_left = _end + vector_tzz_*(-radii) + vector_z_*(height*cos(wave_angle))
		+ vector_tz_*(-height*sin(wave_angle));
	face_list_.push_back(new Triangle(_end, back_face_left, back_face_right));


	GeoV3 start_right = _start + vector_z_*(generatrix*cos(angle + wave_angle))
		+ vector_tz_*(generatrix*sin(angle + wave_angle));
	GeoV3 end_right = _end + vector_z_*(generatrix*cos(angle + wave_angle))
		+ vector_tz_*(generatrix*sin(angle + wave_angle));
	GeoV3 start_left = _start + vector_z_*(generatrix*cos(angle + wave_angle))
		+ vector_tz_*(-generatrix*sin(angle + wave_angle));
	GeoV3 end_left = _end + vector_z_*(generatrix*cos(angle + wave_angle))
		+ vector_tz_*(-generatrix*sin(angle + wave_angle));

	GeoV3 slider_end_left = front_face_left + _end - _start;
	GeoV3 slider_end_right = front_face_right + _end - _start;

	// triangle corner_start_right 
	face_list_.push_back(new Triangle(_start, start_right, front_face_right));

	// triangle corner_start_left
	face_list_.push_back(new Triangle(_start, front_face_left, start_left));

	// triangle corner_end_right 
	face_list_.push_back(new Triangle(_end, back_face_right, end_right));

	// triangle corner_end_left;
	face_list_.push_back(new Triangle(_end, end_left, back_face_left));

	//triangle top_left_;
	face_list_.push_back(new Triangle(slider_end_left, back_face_left, end_left));

	//triangle top_right_
	face_list_.push_back(new Triangle(slider_end_right, end_right, back_face_right));

	//paralleogram slider_
	face_list_.push_back(new Parallelogram(front_face_right, slider_end_right, slider_end_left, front_face_left));

	//parallelogram slider_left_
	face_list_.push_back(new Parallelogram(front_face_left, slider_end_left, end_left, start_left));

	//parallelogram slider_right_
	face_list_.push_back(new Parallelogram(start_right, end_right, slider_end_right, front_face_right));

	//parallelogram top_
	face_list_.push_back(new Parallelogram(slider_end_right, back_face_right, back_face_left, slider_end_left));

	// parallelogram right_;
	face_list_.push_back(new Parallelogram(start_right, _start, _end, end_right));

	// parallelogram left_;
	face_list_.push_back(new Parallelogram(start_left, end_left, _end, _start));

	//Check

	GeoV3 back_slider_right = _end + vector_tzz_*(-radii) + vector_z_*(height*cos(wave_angle))
		+ vector_tz_*(height*sin(wave_angle));
	GeoV3 back_slider_left = _end + vector_tzz_*(-radii) + vector_z_*(height*cos(wave_angle))
		+ vector_tz_*(-height*sin(wave_angle));

	Check_top_ = new  Parallelogram(front_face_left, back_slider_left, back_slider_right, front_face_right);
	Check_top_left_ = new Triangle(start_left, front_face_left, back_slider_left);

	Check_top_right_ = new Triangle(back_slider_right, front_face_right, start_right);

	// parallelogram top_right
	//face_list_.push_back(new Triangle(back_face_left, end_left, front_face_left));
	//face_list_.push_back(new Triangle(front_face_left, end_left, start_left));

	// parallelogram top_left
	//face_list_.push_back(new Triangle(back_face_right, front_face_right, end_right));
	//face_list_.push_back(new Triangle(end_right, front_face_right, start_right));

	// parallelogram right;
	//face_list_.push_back(new Parallelogram(start_right, _start, _end, end_right));

	// parallelogram left;
	//face_list_.push_back(new Parallelogram(start_left, end_left, _end, _start));

	// parallelogram top
	//face_list_.push_back(new Parallelogram(front_face_left, front_face_right, back_face_right, back_face_left));

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


CommonBulk::~CommonBulk()
{
}



double CommonBulk::Angle(point p)
{
	GeoV3 temp = GeoV3(p - start_);
	if ((p - start_) == point(0, 0, 0))
		return pi / 2;

	double len = Geometry::dot(temp, vector_tz_);
	GeoV3 v = temp - vector_tz_*len;
	len = -Geometry::dot(v, vector_tzz_);
	double angle = -Geometry::angle(vector_t_, vector_tzz_);

	if (abs(angle - pi / 2) <= eps)
		return pi / 2;

	len = len / cos(angle);
	temp = temp - vector_t_*len;
	temp = temp - vector_t_*(Geometry::dot(temp, vector_t_));
	return Geometry::angle(temp, vector_tz_);

	//if ((p - start_) == point(0, 0, 0))
	//	return pi / 2;

	//GeoV3 temp = Geometry::Vector3d(p - start_);

	//double len = Geometry::dot(temp, vector_t_);
	//temp = temp - vector_t_*len;
	//return  Geometry::angle(temp, vector_tz_);

}





// Above face  p to pMax
bool CommonBulk::IfAboveUpCol(point p)
{
	point p_end(p.x(), p.y(), MAX);

	for (int i = 6; i < 8; i++)
	{
		if (IfTriIntersect(i, p, p_end))
		{
			return true;
		}
	}

	for (int i = 8; i < 12; i++)
	{
		if (IfParaIntersect(i, p, p_end))
		{
			return true;
		}
	}
	return false;
}

//Above face p to  p(-Max)
point CommonBulk::AboveDownCol(point p)
{
	point p_end(p.x(), p.y(), -MAX);

	for (int i = 6; i < 8; i++)
	{
		if (IfTriIntersect(i, p, p_end))
		{
			point temp = TriIntersect(i, p, p_end);
			return temp;
		}
	}
	for (int i = 8; i < 12; i++)
	{
		if (IfParaIntersect(i, p, p_end))
		{
			point temp = ParaIntersect(i, p, p_end);
			return temp;
		}
	}
}

// Above face p tp p(-Max)
bool  CommonBulk::IfAboveDownCol(point p)
{
	point p_end(p.x(), p.y(), -MAX);

	for (int i = 6; i < 8; i++)
	{
		if (IfTriIntersect(i, p, p_end))
		{
			return true;
		}
	}
	for (int i = 8; i < 12; i++)
	{
		if (IfParaIntersect(i, p, p_end))
		{
			return true;
		}
	}
	return false;
}

point  CommonBulk::AboveUpCol(point p)
{
	point p_end(p.x(), p.y(), MAX);

	for (int i = 6; i < 8; i++)
	{
		if (IfTriIntersect(i, p, p_end))
		{
			point temp = TriIntersect(i, p, p_end);
			return temp;
		}
	}
	for (int i = 8; i < 12; i++)
	{
		if (IfParaIntersect(i, p, p_end))
		{
			point temp = ParaIntersect(i, p, p_end);
			return temp;
		}
	}

}

// For Below face
point  CommonBulk::BelowDownCol(point p)
{
	point p_end(p.x(), p.y(), -MAX);

	for (int i = 0; i < 6; i++)
	{
		if (IfTriIntersect(i, p, p_end))
		{
			point temp = TriIntersect(i, p, p_end);
			return temp;
		}
	}
	for (int i = 12; i < 14; i++)
	{
		if (IfParaIntersect(i, p, p_end))
		{
			point temp = ParaIntersect(i, p, p_end);
			return temp;
		}
	}


}


bool  CommonBulk::IfBelowDownCol(point p)
{
	point p_end(p.x(), p.y(), -MAX);

	for (int i = 0; i < 6; i++)
	{
		if (IfTriIntersect(i, p, p_end))
		{
			return true;
		}
	}

	for (int i = 12; i < 14; i++)
	{
		if (IfParaIntersect(i, p, p_end))
		{
			return true;
		}
	}
	return false;
}


bool  CommonBulk::IfBelowUpCol(point p)
{
	point p_end(p.x(), p.y(), MAX);

	for (int i = 0; i < 6; i++)
	{
		if (IfTriIntersect(i, p, p_end))
		{
			return true;
		}
	}

	for (int i = 12; i < 14; i++)
	{
		if (IfParaIntersect(i, p, p_end))
		{
			return true;
		}
	}
	return false;
}


point  CommonBulk::BelowUpCol(point p)
{
	point p_end(p.x(), p.y(), MAX);

	for (int i = 0; i < 6; i++)
	{
		if (IfTriIntersect(i, p, p_end))
		{
			point temp = TriIntersect(i, p, p_end);
			return temp;
		}
	}
	for (int i = 12; i < 14; i++)
	{
		if (IfParaIntersect(i, p, p_end))
		{
			point temp = ParaIntersect(i, p, p_end);
			return temp;
		}
	}

}


bool  CommonBulk::Inside(point p)
{
	if (IfAboveUpCol(p) && IfBelowDownCol(p))
		return true;
	else
		return false;

	//vector<point> inside_collision;
	//vector<int> state;
	//point temp;
	//point p_end(p.x(), p.y(), MAX);

	//for (int i = 0; i < 10; i++)
	//{
	//	if (bulk->IfTriIntersect(i, p, p_end))
	//	{
	//		point temp = bulk->TriIntersect(i, p, p_end);
	//		if (!CheckPoint(temp, inside_collision))
	//		{
	//			inside_collision.push_back(temp);
	//			state.push_back(i);
	//		}
	//	}
	//}

	//for (int i = 10; i < 13; i++)
	//{
	//	if (bulk->IfParaIntersect(i, p, p_end))
	//	{
	//		point temp = bulk->ParaIntersect(i, p, p_end);

	//		if (!CheckPoint(temp, inside_collision))
	//		{
	//			inside_collision.push_back(temp);
	//			state.push_back(i);
	//		}
	//	}
	//}

	//if (state.size() == 1)
	//	return true;

	//return false;
}
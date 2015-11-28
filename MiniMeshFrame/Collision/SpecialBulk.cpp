#include "SpecialBulk.h"


SpecialBulk::SpecialBulk()
{
	flag = 2;
}


SpecialBulk::~SpecialBulk()
{
}

SpecialBulk::SpecialBulk(ExtruderCone *extruder, point start, point end)
{
	     flag = 2;
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
		Geometry::Vector3d _vector = _end - _start;

GeoV3 front_face_right = _start + vector_tzz_*(radii)+vector_z_*(height*cos(wave_angle))
			+ vector_tz_*(height*sin(wave_angle));
GeoV3 front_face_left = _start + vector_tzz_*(radii)+vector_z_*(height*cos(wave_angle))
+ vector_tz_*(-height*sin(wave_angle));

GeoV3 back_slider_right = _end + vector_tzz_*(-radii) + vector_z_*(height*cos(wave_angle))
+ vector_tz_*(height*sin(wave_angle));
GeoV3 back_slider_left = _end + vector_tzz_*(-radii) + vector_z_*(height*cos(wave_angle))
+ vector_tz_*(-height*sin(wave_angle));

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

GeoV3 start_slider_left = back_slider_left - _vector;
GeoV3 start_slider_right = back_slider_right - _vector;

//Triangle		front_;								// 0
face_list_.push_back(new Triangle(_start, front_face_right, front_face_left));

//Triangle		back_;								// 1
face_list_.push_back(new Triangle(_start, start_slider_left, start_slider_right));

//Triangle		corner_start_right_;				// 2
face_list_.push_back(new Triangle(_start, start_right, front_face_right));

//Triangle		corner_start_left_;					// 3
face_list_.push_back(new Triangle(_start, front_face_left, start_left));

//Triangle		corner_back_right_;					// 4
face_list_.push_back(new Triangle(_start, start_slider_right, start_right));

//Triangle		corner_back_left_;					// 5
face_list_.push_back(new Triangle(_start, start_left, start_slider_left));

//Triangle       top_left_;                              //6
face_list_.push_back(new Triangle(end_left, slider_end_left, back_slider_left));

//Triangle       top_right_;                            //7
face_list_.push_back(new Triangle(back_slider_right, slider_end_right, end_right));


//Parallelogram slider_;                               //8
face_list_.push_back(new Parallelogram(front_face_right, slider_end_right, slider_end_left, front_face_left));


//Parallelogram slider_left_;                    //9
face_list_.push_back(new Parallelogram(start_left, front_face_left, slider_end_left, end_left));

//Parallelogram slider_right_;                     //10
face_list_.push_back(new Parallelogram(start_right, end_right, slider_end_right, front_face_right));
//Parallelogram top_;                                    //11
face_list_.push_back(new Parallelogram(slider_end_right,back_slider_right, back_slider_left, slider_end_left));

//Parallelogram slider_back_;                         //12
face_list_.push_back(new Parallelogram(start_slider_left, back_slider_left, back_slider_right, start_slider_right));

//Parallelogram slider_back_left_;    //13
face_list_.push_back(new Parallelogram(start_left, end_left, back_slider_left, start_slider_left));

//Parallelogram slider_back_right_; //14
face_list_.push_back(new Parallelogram(start_slider_right, back_slider_right, end_right, start_right));


//Check

Check_top_ =new  Parallelogram(front_face_left, back_slider_left, back_slider_right, front_face_right);
 Check_top_left_ =new Triangle(start_left, front_face_left, back_slider_left);

 Check_top_right_ =new Triangle(back_slider_right, front_face_right, start_right);

	


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



// Above face  p to pMax
bool SpecialBulk::IfAboveUpCol(point p)
{
	point p_end(p.x(), p.y(), MAX);

	for (int i = 6; i<8; i++)
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
point SpecialBulk::AboveDownCol(point p)
{
	point p_end(p.x(), p.y(), -MAX);

	for (int i = 6; i<8; i++)
	{
		if (IfTriIntersect(i, p, p_end))
		{
			return TriIntersect(i, p, p_end);
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
bool  SpecialBulk::IfAboveDownCol(point p)
{
	point p_end(p.x(), p.y(), -MAX);

	for (int i = 6; i<8; i++)
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

point  SpecialBulk::AboveUpCol(point p)
{
	point p_end(p.x(), p.y(), MAX);

	for (int i = 6; i<8; i++)
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
point  SpecialBulk::BelowDownCol(point p)
{
	point p_end(p.x(), p.y(), -MAX);

	for (int i = 0; i<6; i++)
	{
		if (IfTriIntersect(i, p, p_end))
		{
			point temp = TriIntersect(i, p, p_end);
			return temp;
		}
	}
	for (int i = 12; i < 15; i++)
	{
		if (IfParaIntersect(i, p, p_end))
		{
			point temp =ParaIntersect(i, p, p_end);
			return temp;
		}
	}


}


bool  SpecialBulk::IfBelowDownCol(point p)
{
	point p_end(p.x(), p.y(), -MAX);

	for (int i = 0; i<6; i++)
	{
		if (IfTriIntersect(i, p, p_end))
		{
			return true;
		}
	}

	for (int i = 12; i < 15; i++)
	{
		if (IfParaIntersect(i, p, p_end))
		{
			return true;
		}
	}
   return false;
}


bool  SpecialBulk::IfBelowUpCol(point p)
{
	point p_end(p.x(), p.y(), MAX);

	for (int i = 0; i<6; i++)
	{
		if (IfTriIntersect(i, p, p_end))
		{
			return true;
		}
	}

	for (int i = 12; i < 15; i++)
	{
		if (IfParaIntersect(i, p, p_end))
		{
			return true;
		}
	
	}
	return false;
}


point  SpecialBulk::BelowUpCol(point p)
{
	point p_end(p.x(), p.y(), MAX);

	for (int i = 0; i<6; i++)
	{
		if (IfTriIntersect(i, p, p_end))
		{
			point temp = TriIntersect(i, p, p_end);
			return temp;
		}
	}
	for (int i = 12; i < 15; i++)
	{
		if (IfParaIntersect(i, p, p_end))
		{
			point temp = ParaIntersect(i, p, p_end);
			return temp;
		}
	}

}




double SpecialBulk::Angle(point p)
{

	if ((p - start_) == point(0, 0, 0))
		return pi / 2;
	GeoV3 temp= Geometry::Vector3d(p - start_);

	if (Geometry::dot(temp, vector_tz_) == 0)
		return pi / 2;
	
	double l = -Geometry::dot(temp, vector_tzz_);
	double angle = pi-Geometry::angle(vector_tzz_, vector_t_);

	GeoV3 t = vector_t_*(l / cos(angle));
	temp = temp - t;

	return  Geometry::angle(temp, vector_tz_);

}

bool  SpecialBulk::Inside(point p)
{
	if (IfAboveUpCol(p) && IfBelowDownCol(p))
		return true;
	else
		return false;

}
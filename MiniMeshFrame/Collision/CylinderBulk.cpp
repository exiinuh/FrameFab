#include "CylinderBulk.h"

const double threhold=50;

CylinderBulk::CylinderBulk(point start, point end)
{
	start_ = start;
	end_ = end;
	cylinder_ = Cylinder_(start_, end_);	
}

gte::Cylinder3<float> CylinderBulk::Cylinder_(point a, point b)
{
	gte::Cylinder3<float> cylinder;
	cylinder.axis = CyLine(a, b);
	cylinder.height = (b - a).length() + 2 * float(tan(extruder_.Angle())*extruder_.Height());
	cylinder.radius = float(extruder_.Height());
	return cylinder;
}

gte::Line<3, float> CylinderBulk::CyLine(point a, point b)
{
	point t = b - a;

	a -  float(tan(extruder_.Angle())*extruder_.Height())*t;
	b + float(tan(extruder_.Angle())*extruder_.Height())*t;

	return Line_(a - float(tan(extruder_.Angle())*extruder_.Height())*t, b + float(tan(extruder_.Angle())*extruder_.Height())*t);
}

bool CylinderBulk::IsColCyl(point target_start, point target_end)
{
	target_edge_ = Segement_(target_start, target_end);

	gte::Segment3<float> printing_edge_ = Segement_(start_, end_);

	gte::DCPQuery<float, gte::Segment3<float>, gte::Segment3<float>> intersection;
	auto result = intersection(target_edge_, printing_edge_);

	/* if the distance between the printing edge 
	*  and target edge is too big, we claim their 
	*  influence on each other are neglectable
	*/
	if (result.distance > threhold)
		return false;
	else
		return true;
}


CylinderBulk::CylinderBulk(point start, point end, point target_start, point target_end)
{
	double max;

	start_    = start;
	end_	  = end;
	cylinder_ = Cylinder_(start_, end_);

	/* Distance computation */
	gte::DCPQuery<float, gte::Vector3<float>, gte::Line3<float>> query;
	
	gte::Line<3, float>   line = Line_(start, end);

	/* find the maximal distance between target endpoints and printing edge */
	auto result = query(Point_(target_start), line);
	max = result.distance;
	result = query(Point_(target_end), line);

	if (result.distance > max)
	{
		max = result.distance;
	}
	
	cylinder_.radius = max;

	/* target edge and current edge are same */
	if ( (start == target_start) && (end == target_end))
	{
		is_collision_ = 0;
		return;
	}

	if ((end == target_start )&&( start == target_end))
	{
		is_collision_ = 0;
		return;
	}

	// angle

	if ((end == target_start))
	{
		if (Geometry::angle(start - end, target_end - target_start) > Threshold_angle)
		{
			is_collision_ = 0;
			return;
		}
	}

	if ((end == target_end))
	{
		if (Geometry::angle(start - end, target_start - target_end) > Threshold_angle)
		{
			is_collision_ = 0;
			return;
		}
	}

	if ((start == target_start))
	{
		if (Geometry::angle(end - start, target_end - target_start) > Threshold_angle)
		{
			is_collision_ = 0;
			return;
		}
	}

	if ((start == target_end))
	{
		if (Geometry::angle(end - start, target_start - target_end) > Threshold_angle)
		{
			is_collision_ = 0;
			return;
		}
	}


	/* If the distance between target edge and printing edge 
	*  is too big, we claim that their influence on each other 
	*  are neglectable*/
	if (!IsColCyl(target_end, target_start))
	{
		is_collision_ = 0;
		return;
	}

	/* Collision existed */
	is_collision_ = 1;

	/* Incident edge, cross vector forms the feasible orientation */
	if (target_start == start || target_start == end)
	{
		range_ = 2*F_PI - extruder_.Angle() * 2;

		/* eliminate the case that target_start = start, target_end = end */
		if (IfParalet(target_end))
		{
			is_collision_ = 0;
			return;
		}
		Rangle_.u = ColAngle(target_end);
		Rangle_.v = Rangle_.u;
		return;
	}

	if (target_end == start || target_end == end)
	{
		if (IfParalet(target_start))
		{
			is_collision_ = 0;
			return;
		}

		range_ = 2 * F_PI - extruder_.Angle() * 2;
		Rangle_.u = ColAngle(target_start);
		Rangle_.v = Rangle_.u;
		return;
	}

	if (IfParalet(target_start) && IfParalet(target_end))
	{
		is_collision_ = 0;
		return;
	}

	if (IfParalet(target_start))
	{
		Rangle_.u = ColAngle(target_end);
		Rangle_.v = ColAngle(target_end);
		return;
	}

	if (IfParalet(target_end))
	{
		Rangle_.u = ColAngle(target_start);
		Rangle_.v = ColAngle(target_start);
		return;
	}

	Rangle_.u = ColAngle(target_start);
	Rangle_.v = ColAngle(target_end);

	if (Rangle_.u.norm() < eps)
		Rangle_.u = Rangle_.v;

	if (Rangle_.v.norm() < eps)
		Rangle_.v = Rangle_.u;

	range_ = 2 * F_PI - extruder_.Angle() * 2 - Geometry::angle(ColAngle(target_end), ColAngle(target_start));
	return;
}


bool  CylinderBulk::IfParalet(point p)
{
	GeoV3 t = Geometry::Vector3d(end_ - start_);

	GeoV3 temp = Geometry::Vector3d(p- start_);
	temp = Geometry::cross(temp, t);
	//double angle = Geometry::angle(zt, temp);

	temp = Geometry::cross(t, temp);
 
	if (temp.norm() < eps)
		return true;
	else
		return false;
}


gte::Vector3<float> CylinderBulk::Point_(point p)
{
	gte::Vector<3, float> point_;

	std::array<float, 3> s;
	s[0] = p.x(); s[1] = p.y(); s[2] = p.z();
	point_ = s;
	return point_;
}


//x 
GeoV3  CylinderBulk::ColAngle(point target)
{
	GeoV3 t = Geometry::Vector3d( end_-start_);

	GeoV3 temp = Geometry::Vector3d(target - start_);
	temp = Geometry::cross(temp, t);
		//double angle = Geometry::angle(zt, temp);

	temp = Geometry::cross(t, temp);
	temp.normalize();
	return temp;
}
#include "QuadricCollision.h"


QuadricCollision::QuadricCollision()
{

}

QuadricCollision::~QuadricCollision()
{
}


QuadricCollision::QuadricCollision(WireFrame *ptr_frame)
{
	ptr_frame_ = ptr_frame;
	divide_ = 60;
}


void QuadricCollision::DetectCollision(WF_edge *target_e, DualGraph *ptr_subgraph)
{
	Init(target_e);

	/* collision with edge */
	int Nd = ptr_subgraph->SizeOfVertList();
	for (int i = 0; i < Nd; i++)
	{
		WF_edge *e = ptr_frame_->GetEdge(ptr_subgraph->e_orig_id(i));
		if (e != target_e_ && e != target_e_->ppair_)
		{
			DetectEdge(e);
		}
	}
}


void QuadricCollision::DetectCollision(WF_edge *target_e, WF_edge *order_e)
{
	Init(target_e);

	/* collision with edge */
	DetectEdge(order_e);
}


void QuadricCollision::Init()
{
	lld temp = 0;
	state_map_.clear();
	state_map_.push_back(temp);
	state_map_.push_back(temp);
	state_map_.push_back(temp);
}


void QuadricCollision::Init(WF_edge *target_e)
{
	target_e_ = target_e;
	Init();
}


void QuadricCollision::Init(vector<lld> &angle_state)
{
	lld temp = 0;
	angle_state.clear();
	angle_state.push_back(temp);
	angle_state.push_back(temp);
	angle_state.push_back(temp);
}


void QuadricCollision::DetectEdge(WF_edge *order_e)
{
	double ¦È;								// angle with Z axis (rad)
	double ¦Õ;								// angle with X axis (rad)

	for (int j = 0; j < 3; j++)
	{
		for (int i = 0; i < divide_; i++)
		{
			if (i < 20)
			{
				¦È = (j * 3 + 1)*18.0 / 180.0*F_PI;
				¦Õ = i*18.0 / 180.0*F_PI;
			}

			if (i>19 && i < 40)
			{
				¦È = (j * 3 + 2) * 18.0 / 180.0*F_PI;
				¦Õ = (i - 20)*18.0 / 180.0*F_PI;
			}

			if (i>39)
			{
				¦È = (j * 3 + 3)* 18.0 / 180.0*F_PI;
				¦Õ = (i - 40)*18.0 / 180.0*F_PI;
			}
			lld mask = ((lld)1 << i);
			if (DetectBulk(order_e, ¦È, ¦Õ))
			{
				state_map_[j] |= mask;			
			}		
		}
	}

	//North Point
	lld mask = ((lld)1 << 60);
	if (DetectBulk(order_e, 0, 0))
	{
		state_map_[2] |= mask;
	}

	//South Point
	mask = ((lld)1 << 61);
	if (DetectBulk(order_e, F_PI, 0))
	{
		state_map_[2] |= mask;
	}
}



bool QuadricCollision::DetectBulk(WF_edge *order_e, double ¦È, double ¦Õ)
{
	GeoV3 target_start = target_e_->pvert_->Position();
	GeoV3 target_end = target_e_->ppair_->pvert_->Position();
	GeoV3 order_start = order_e->pvert_->Position();
	GeoV3 order_end = order_e->ppair_->pvert_->Position();
	GeoV3 normal = Orientation(¦È, ¦Õ);

	//0
	if (Parallel(normal, target_start - target_end))
	{
		if (ParallelCase(target_start, target_end, order_start, order_end, normal))
		{
			return true;
		}
		return false;
	}

	//1
	if ((target_start - order_end).norm() < eps)
	{
		if ( SpecialCase(target_start,target_end,order_start,normal ))
		{
			return true;
		}
		return false;
	}

	if ((target_start - order_start).norm() < eps)
	{
		if (SpecialCase(target_start, target_end, order_end, normal))
		{
			return true;
		}
		return false;
	}

	if ((target_end - order_end).norm() < eps)
	{
		if (SpecialCase(target_end, target_start, order_start, normal))
		{
			return true;
		}
		return false;
	}

	if ((target_end - order_start).norm() < eps)
	{
		if (SpecialCase(target_end, target_start, order_end, normal))
		{
			return true;
		}
		return false;
	}

	//2
	if (Case(target_start, target_end, order_start, order_end, normal))
	{
		return true;
	}

	return false;
}


bool QuadricCollision::DetectAngle(GeoV3 connect, GeoV3 end, GeoV3 target_end, GeoV3 normal)
{
	if (angle(normal, target_end - connect) < extruder_.Angle())
		return true;
	return false;
}


bool QuadricCollision::Case(GeoV3 target_start, GeoV3 target_end, 
	GeoV3 order_start, GeoV3 order_end, GeoV3 normal)
{
	GenerateVolume(target_start, target_end, order_start, order_end, normal);

	//Cone
	if (DetectCone(target_start, normal, order_start, order_end))
		return true;

	if (DetectCone(target_end, normal, order_start, order_end))
		return true;

	//Cylinder
	if (DetectCylinder(target_start, normal, order_start, order_end))
		return true;

	if (DetectCylinder(target_end, normal, order_start, order_end))
		return true;

	//Face

	for (int i = 0; i < bulk_.size(); i++)
	{
		if (DetectTriangle(bulk_[i], order_start, order_end))
			return true;

	}

	return false;
}

bool QuadricCollision::SpecialCase(GeoV3 connect, GeoV3 target_s, GeoV3 order_s, GeoV3 normal)
{
	if (angle(normal, order_s - connect) < extruder_.Angle())
		return true;

	if (DetectCone(target_s, normal, connect, order_s))
		return true;

	if (DetectCylinder(target_s, normal, connect, order_s))
		return true;

	//Face
	GenerateVolume(connect, target_s, order_s, normal);
	for (int i = 0; i < bulk_.size(); i++)
	{
		if (DetectTriangle(bulk_[i], connect, order_s))
			return true;
	}

	return false;

}





bool QuadricCollision::ParallelCase(GeoV3 target_start, GeoV3 target_end,
	GeoV3 order_start, GeoV3 order_end, GeoV3 normal)
{

	//Exception situation
	if ((target_start - order_end).norm() < eps)
	{
		if (DetectAngle(target_start, target_end, order_start, normal))
		{
			return true;
		}
	}
	if ((target_start - order_start).norm() < eps)
	{
		if (DetectAngle(target_start, target_end, order_end, normal))
		{

			return true;
		}
	}
	if ((target_end - order_end).norm() < eps)
	{
		if (DetectAngle(target_end, target_start, order_start, normal))
		{

			return true;
		}
	}
	if ((target_end - order_start).norm() < eps)
	{
		if (DetectAngle(target_end, target_start, order_end, normal))
		{

			return true;
		}
	}

	//Normal situation
	//Cone
	if (DetectCone(target_start, normal, order_start, order_end))
		return true;

	if (DetectCone(target_end, normal, order_start, order_end))
		return true;

	//Cylinder
	if (DetectCylinder(target_start, normal, order_start, order_end))
		return true;

	if (DetectCylinder(target_end, normal, order_start, order_end))
		return true;

	return false;
}


bool QuadricCollision::DetectCone(GeoV3 start, GeoV3 normal, GeoV3 target_start, GeoV3 target_end)
{
	gte::Cone3<float> start_cone;
	start_cone.angle = extruder_.Angle();
	start_cone.height = extruder_.Height();

	gte::Ray3<float> start_ray;

	std::array<float, 3>s;
	s[0] = start.getX(); s[1] = start.getY(); s[2] = start.getZ();
	start_ray.origin = s;
	s[0] = normal.getX(); s[1] = normal.getY(); s[2] = normal.getZ();
	start_ray.direction = s;
	gte::Segment<3, float> segment;
	segment = Seg(target_start, target_end);
	gte::FIQuery<float, gte::Segment<3, float>, gte::Cone3<float>> intersection;
	auto result = intersection(segment, start_cone);


	return result.intersect;

}

bool QuadricCollision::DetectCylinder(GeoV3 start, GeoV3 normal, GeoV3 target_start, GeoV3 target_end)
{


	gte::Cylinder3<float> cylinder;
	cylinder.axis;
	gte::Line3<float> cylinder_line;

	std::array<float, 3>s;

	GeoV3 cylin_center;
	cylin_center = start + normal*(extruder_.Height() + extruder_.CyclinderLenth()/2);

	s[0] = cylin_center.getX(); s[1] = cylin_center.getY(); s[2] = cylin_center.getZ();
	cylinder_line.origin = s;
	s[0] = normal.getX(); s[1] = normal.getY(); s[2] = normal.getZ();
	cylinder_line.direction = s;

	cylinder.axis = cylinder_line;

	cylinder.height = extruder_.CyclinderLenth();
	cylinder.radius = extruder_.Radii();


	gte::Segment<3, float> segment;
	segment = Seg(target_start, target_end);
	gte::FIQuery<float, gte::Segment<3, float>, gte::Cylinder3<float>> intersection;
	auto result = intersection(segment, cylinder);

	return result.intersect;
}

bool QuadricCollision::DetectTriangle(Triangle triangle, GeoV3 target_start, GeoV3 target_end)
{
	gte::Triangle<3, float> triangle_ = Tri(triangle.v0(), triangle.v1(), triangle.v2());

	gte::FIQuery<float, gte::Segment<3, float>, gte::Triangle3<float>> intersection;
	gte::Segment<3, float> segment = Seg(target_start, target_end);

	auto result = intersection(segment, triangle_);
	return result.intersect;
}


void QuadricCollision::GenerateVolume(GeoV3 start, GeoV3  end, 
	GeoV3 target_start, GeoV3  target_end, GeoV3 normal)
{
	//face: front (up, down) back(up,down)
	GeoV3 t = end - start;
	double edge_length = t.norm();
	t.normalize();

	GeoV3 p = cross(t, normal);
	p.normalize();

	GeoV3 start_cone_center = normal*extruder_.Height() + start;
	GeoV3 end_cone_center = normal*extruder_.Height() + end;

	//face front
	GeoV3 start_front_cone = start_cone_center + p*extruder_.Radii();
	GeoV3 start_front_cylinder = start_front_cone + normal*extruder_.CyclinderLenth();
	GeoV3 end_front_cone = end_cone_center + p*extruder_.Radii();
	GeoV3 end_front_cylinder = end_front_cone + normal*extruder_.CyclinderLenth();

	//face back
	GeoV3 start_back_cone = start_cone_center - p*extruder_.Radii();
	GeoV3 start_back_cylinder = start_back_cone + normal*extruder_.CyclinderLenth();
	GeoV3 end_back_cone = end_cone_center - p*extruder_.Radii();
	GeoV3 end_back_cylinder = end_back_cone + normal*extruder_.CyclinderLenth();

	bulk_.clear();

	//front
	bulk_.push_back(Triangle(start, end, start_front_cone));
	bulk_.push_back(Triangle(end, end_front_cone, start_front_cone));
	bulk_.push_back(Triangle(start_front_cone, end_front_cone, start_front_cylinder));
	bulk_.push_back(Triangle(end_front_cone, end_front_cylinder, start_front_cylinder));

	//back
	bulk_.push_back(Triangle(start, end_back_cone, end));
	bulk_.push_back(Triangle(start, start_back_cone, end_back_cone));
	bulk_.push_back(Triangle(start_back_cone, end_back_cylinder, end_back_cone));
	bulk_.push_back(Triangle(start_back_cone, start_back_cylinder, end_back_cylinder));
}


void QuadricCollision::GenerateVolume(GeoV3 connect, GeoV3 end, GeoV3 target_end, GeoV3 normal)
{
	//face: front (up, down) back(up,down)

	GeoV3 t = end - connect;
	double edge_length = t.norm();
	t.normalize();

	GeoV3 p = cross(t, normal);
	p.normalize();

	double ¦Å = 10;

	GeoV3 start_cone_center = normal*extruder_.Height() + connect;

	//face front
	GeoV3 start_front_cone = start_cone_center + p*extruder_.Radii() + t*¦Å;
	GeoV3 start_front_cylinder = start_front_cone + normal*extruder_.CyclinderLenth() + t*¦Å;

	//face back
	GeoV3 start_back_cone = start_cone_center - p*extruder_.Radii() + t*¦Å;
	GeoV3 start_back_cylinder = start_back_cone + normal*extruder_.CyclinderLenth() + t*¦Å;

	GeoV3 start = connect + t*¦Å;
	//front
	bulk_.clear();
	bulk_.push_back(Triangle(start, start_back_cone, start_front_cone));
	bulk_.push_back(Triangle(start_front_cylinder, start_back_cone, start_front_cone));
	bulk_.push_back(Triangle(start_front_cylinder, start_back_cylinder, start_back_cone));
}


bool QuadricCollision::Parallel(GeoV3 a, GeoV3 b)
{
	if (abs(angle(a, b)) < eps || abs(angle(a, b) - F_PI) < eps)
		return true;
	return false;
}


gte::Segment<3, float> QuadricCollision::Seg(point target_start, point target_end)
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

gte::Segment<3, float> QuadricCollision::Seg(GeoV3 target_start, GeoV3 target_end)
{
	gte::Segment<3, float> segment;
	segment.p[0][0] = target_start.getX();
	segment.p[0][1] = target_start.getY();
	segment.p[0][2] = target_start.getZ();
	segment.p[1][0] = target_end.getX();
	segment.p[1][1] = target_end.getY();
	segment.p[1][2] = target_end.getZ();
	return segment;

}

gte::Triangle<3, float> QuadricCollision::Tri(GeoV3 a, GeoV3 b, GeoV3 c)
{
	gte::Triangle<3, float> triangle;
	triangle.v[0][0] = a.getX();
	triangle.v[0][1] = a.getY();
	triangle.v[0][2] = a.getZ();
	triangle.v[1][0] = b.getX();
	triangle.v[1][1] = b.getY();
	triangle.v[1][2] = b.getZ();
	triangle.v[2][0] = c.getX();
	triangle.v[2][1] = c.getY();
	triangle.v[2][2] = c.getZ();
	return triangle;

}


void QuadricCollision::Debug()
{
	//point b;
	//
	//WF_edge *a = ptr_frame_->GetEdge(33);
	//b = a->pvert_->Position();
	//cout << b.x() << ", " <<b.y() << ", " <<b.z()<< ", " << endl;
	//a = a->ppair_;
	//b = a->pvert_->Position();
	//cout << b.x() << ", " << b.y() << ", " << b.z() << ", " << endl;
	//a = ptr_frame_->GetEdge(55);
	//b = a->pvert_->Position();
	//cout << b.x() << ", " << b.y() << ", " << b.z() << ", " << endl;
	//a = a->ppair_;
	//b = a->pvert_->Position();
	//cout << b.x() << ", " << b.y() << ", " << b.z() << ", " << endl;
	//DetectCollision(ptr_frame_->GetEdge(33), ptr_frame_->GetEdge(55));

	//DetectBulk(ptr_frame_->GetEdge(55),0.628319,1.5708);

	//double ¦È, ¦Õ;

	//for (int j = 0; j < 3; j++)
	//{
	//	for (int i = 0; i < divide_; i++)
	//	{
	//		lld mask = (lld)1 << i;			
	//			if (i < 20)
	//			{
	//				¦È = (j * 3 + 1)*18.0 / 180.0*F_PI;
	//				¦Õ = i*18.0 / 180.0*F_PI;
	//			}
	//			if (i>19 && i < 40)
	//			{
	//				¦È = (j * 3 + 2) * 18.0 / 180.0*F_PI;
	//				¦Õ = (i - 20)*18.0 / 180.0*F_PI;
	//			}
	//			if (i>39)
	//			{
	//				¦È = (j * 3 + 3)* 18.0 / 180.0*F_PI;
	//				¦Õ = (i - 40)*18.0 / 180.0*F_PI;
	//			}
	//		if (state_map_[j] & mask)
	//		{
	//			cout << ¦È << ", " << ¦Õ << ",0" << endl;
	//			continue;
	//		}						
	//	}
	//}
}

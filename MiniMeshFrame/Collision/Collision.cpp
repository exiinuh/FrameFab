#include"Collision.h"

Collision::Collision()
{
}

Collision::Collision(WireFrame *ptr_frame, WF_edge *target_e)
{
	ptr_frame_ = ptr_frame;
	ptr_extruder_ = new ExtruderCone();
	target_e_ = target_e;

	divide_ = 72;
	CreatePrintTable();
	GenerateSampleNormal();
}

Collision::~Collision()
{
	delete ptr_extruder_;
	ptr_extruder_ = NULL;
}


void Collision::DetectCollision(DualGraph *ptr_subgraph)
{
	/* collision with edge */
	int Nd = ptr_subgraph->SizeOfVertList();
	for (int i = 0; i < Nd; i++)
	{
		WF_edge *e = ptr_frame_->GetEdge(ptr_subgraph->e_orig_id(i));
		if (e != target_e_ && e != target_e_->ppair_)
		{
			DetectCollision(e);
		}
	}
}


void Collision::DetectCollision(WF_edge *order_e)
{
	point order_start	= order_e->ppair_->pvert_->Position();
	point order_end		= order_e->pvert_->Position();
	point target_start	= target_e_->ppair_->pvert_->Position();
	point target_end	= target_e_->pvert_->Position();


	if (DisSegSeg(order_start, order_end, target_start, target_end) > threhold
		|| ((target_start == order_start) && (target_end == order_end))
		|| ((target_end == order_start) && (target_start == order_end)))
	{
		return;
	}

	// angle too big, it will not consider
	if (target_end == order_start)
	{
		if (Geometry::angle(target_start - target_end, order_end - order_start) 
			>(3.1415 - ptr_extruder_->Angle()))
		{

			return;
		}
	}

	if (target_end == order_end)
	{
		if (Geometry::angle(target_start - target_end, order_start - order_end) 
			> (3.1415 - ptr_extruder_->Angle()))
		{
			return;
		}
	}

	if (target_start == order_start)
	{
		if (Geometry::angle(target_end - target_start, order_end - order_start) 
			>(3.1415 - ptr_extruder_->Angle()))
		{
			return;
		}
	}

	if (target_start == order_end)
	{
		if (Geometry::angle(target_end - target_start, order_start - order_end) 
			> (3.1415 - ptr_extruder_->Angle()))
		{
			return;
		}
	}

	vector <point> consider;
	if (order_start != target_start && order_start != target_end)
	{
		consider.push_back(order_start);
	}
	if (order_end != target_start && order_end != target_end)
	{
		consider.push_back(order_end);
	}

	GeoV3 u, v;
	if (consider.size() == 1)
	{
		u = ColAngle(consider[0], target_start, target_end);
		v = u;
	}
	
	if (consider.size() == 2)
	{
		u = ColAngle(consider[0], target_start, target_end);
		v = ColAngle(consider[1], target_start, target_end);
	}
	
	//-----reduce--------------------------------------
	/* influence from id to i
	* i.e. the collision cost of existence of edge i
	* when printing edge id
	*/
	vector<GeoV3>::iterator it;
	for (it = normal_.begin(); it != normal_.end(); )
	{
		if (IsColVec(u, v, *it))
		{
			it = normal_.erase(it);
		}
		else
		{
			++it;
		}
	}
}


void Collision::CreatePrintTable()
{
	std::array<float, 3>s;
	s[0] = 0; s[1] = 0; s[2] = 1;
	table_.normal = s;
	table_.constant = ptr_frame_->Base();
}


void Collision::GenerateSampleNormal()
{
	/* sampling number of orientation vector in 2*pi angle range */
	point target_start = target_e_->ppair_->pvert_->Position();
	point target_end = target_e_->pvert_->Position();

	GeoV3 u;
	GeoV3 v;
	GeoV3 t = target_end - target_start;
	t.normalize();
	GeoV3 z(0, 0, 1);

	if (Geometry::cross(t, z).norm() < eps)
	{
		/* vertical case */
		u = Geometry::Vector3d(1, 0, 0);
		v = Geometry::Vector3d(0, 1, 0);
	}
	else
	{
		/* perpendicular to printing edge */
		u = Geometry::cross(t, z);
		u.normalize();
		v = Geometry::cross(u, t);
		v.normalize();
	}

	/* normal contains all sampling vectors */
	for (int i = 0; i < divide_; i++)
	{
		normal_.push_back(u*cos(2 * F_PI / divide_ * i) + v*sin(2 * F_PI / divide_ * i));
	}


	/* collision with table */
	vector<GeoV3>::iterator it;
	for (it = normal_.begin(); it != normal_.end();)
	{
		if (IsColTable(*it))
		{
			it = normal_.erase(it);
		}
		else
		{
			++it;
		}
	}
}


double Collision::DisSegPoint(point start, point end, point target)
{
	gte::DCPQuery<float, gte::Vector<3, float>, gte::Segment<3, float>> distance;

	std::array<float, 3> vec;
	vec[0] = target.x(); vec[1] = target.y(); vec[2] = target.z();

	auto result = distance(vec, Segement_(start, end));

	return result.distance;

}


double Collision::DisSegSeg(point start, point end, point target_start, point target_end)
{
	gte::DCPQuery<float, gte::Segment<3, float>, gte::Segment<3, float>> distance;
	auto result = distance(Segement_(target_start, target_end), Segement_(start, end));

	return result.distance;
}


bool Collision::IsColVec(GeoV3 start, GeoV3 end, GeoV3 target)
{
	ExtruderCone extruder_;
	if (Geometry::angle(target, start) <= extruder_.Angle())
		return true;

	if (Geometry::angle(target, end) <= extruder_.Angle())
		return true;

	if (abs(Geometry::angle(target, start) + Geometry::angle(target, end) - Geometry::angle(target, end)) < eps)
		return true;

	//if (Geometry::angle(target, Geometry::Vector3d(0, 0, -1)) < 2*extruder_.Angle())
	//	return true;

	return false;
}


bool Collision::IsColTable(GeoV3 target_angle)
{
	point target_start = target_e_->ppair_->pvert_->Position();
	point target_end = target_e_->pvert_->Position();

	// Collision with Table
	GeoV3 low = target_start;
	if (low.getZ() > target_end.z())
	{
		low = target_end;
	}

	GeoV3 t = target_end - target_start;
	t.normalize();
	GeoV3 u = target_angle;
	u.normalize();
	GeoV3 v = Geometry::cross(u, t);
	v.normalize();

	GeoV3 test;
	test = low + target_angle * ptr_extruder_->ToolLenth(); // In face, we use tool has 120 long cylinder
	
	std::array<float, 3> s; 
	s[0] = test.getX(); 
	s[1] = test.getY(); 
	s[2] = test.getZ();

	std::array<float, 3> n;
	n[0] = target_angle.getX(); 
	n[1] = target_angle.getY(); 
	n[2] = target_angle.getZ();

	gte::Circle3<float>circle;
	circle.center = s;
	circle.normal = n;
	circle.radius = ptr_extruder_->ToolLenth() * tan(ptr_extruder_->Angle());

	gte::FIQuery<float, gte::Plane3<float>, gte::Circle3<float>> intersection;

	auto result = intersection(table_, circle);
	if (result.intersect)
	{
		return true;
	}
	if (s[2] < table_.constant)
	{
		return true;
	}

	return false;
}


GeoV3 Collision::ColAngle(point target, point start, point end)
{
	GeoV3 t = Geometry::Vector3d(end - start);

	GeoV3 temp = Geometry::Vector3d(target - start);
	temp = Geometry::cross(temp, t);
	//double angle = Geometry::angle(zt, temp);

	temp = Geometry::cross(t, temp);
	temp.normalize();
	return temp;
}


gte::Segment<3, float> Collision::Segement_(point target_start, point target_end)
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

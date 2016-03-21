#include"BaseBulk.h"


point BaseBulk::TriIntersect(int i, point start, point end)
{
	Polyface *face = face_list_[i];

	for (int j = 0; j < face->vert_list_.size(); j++)
	{
		if (start == face->vert_list_[j])
			return start;
		if (end == face->vert_list_[j])
			return end;
	}
	gte::FIQuery<float, gte::Segment3<float>, gte::Triangle3<float>> intersection;
	auto result = intersection(Segement_(start, end), Triangle_(face));
	point temp(result.point[0], result.point[1], result.point[2]);
	//temp.x = result.point[0]; temp.y = result.point[1]; temp.z() = result.point[2];
	return temp;
}


point BaseBulk::ParaIntersect(int i, point start, point end)
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


bool BaseBulk::IfTriIntersect(int i, point start, point end)
{
	Polyface *face = face_list_[i];
	//vertify point
	for (int j = 0; j < face->vert_list_.size(); j++)
	{
		if (start == face->vert_list_[j] || end == face->vert_list_[j])
			return true;
	}

	gte::FIQuery<float, gte::Segment3<float>, gte::Triangle3<float>> intersection;
	auto result = intersection(Segement_(start, end), Triangle_(face));
	return result.intersect;
}


bool BaseBulk::IfParaIntersect(int i, point start, point end)
{
	Polyface *face = face_list_[i];
	gte::FIQuery<float, gte::Segment3<float>, gte::Triangle3<float>> intersection;
	auto result = intersection(Segement_(start, end), Triangle_(new Triangle(face->v2(), face->v0(), face->v1())));
	if (result.intersect == 1)
	{
		return true;
	}
	result = intersection(Segement_(start, end), Triangle_(new Triangle(face->v3(), face->v0(), face->v2())));
	if (result.intersect == 1)
	{
		return true;
	}
	return false;
}


gte::Segment<3, float> BaseBulk::Segement_(point target_start, point target_end)
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


gte::Triangle<3, float>  BaseBulk::Triangle_(Polyface *face)
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

gte::Line<3, float> BaseBulk::Line_(point p, GeoV3 vector)
{
	gte::Line<3, float> line;
	if (vector.norm() == 0)
	{
		cout << " Error: gte::Line vector =0" << endl;
	}
	std::array<float, 3> s;
	s[0] = p.x(); s[1] = p.y(); s[2] = p.z();
	line.origin = gte::Vector<3, float>(s);
	s[0] = vector.getX(); s[1] = vector.getY(); s[2] = vector.getZ();
	line.direction = gte::Vector<3, float>(s);
	return line;
}

bool     BaseBulk::IfLine2Tri(point o, GeoV3 v, Triangle* face)
{
	gte::FIQuery<float, gte::Line3<float>, gte::Triangle3<float>> intersection;

	gte::Line<3, float> line = Line_(o, v);
	auto result = intersection(line, Triangle_(face));
	return result.intersect;

}

point   BaseBulk::Line2Tri(point o, GeoV3 v, Triangle* face)
{
	gte::FIQuery<float, gte::Line3<float>, gte::Triangle3<float>> intersection;

	gte::Line<3, float> line = Line_(o, v);
	auto result = intersection(line, Triangle_(face));

	point temp;
	temp = point(result.point[0], result.point[1], result.point[2]);
	//temp.x = result.point[0]; temp.y = result.point[1]; temp.z() = result.point[2];
	return temp;
}


bool     BaseBulk::IfLine2Para(point o, GeoV3 v, Parallelogram* face)
{
	gte::FIQuery<float, gte::Line3<float>, gte::Triangle3<float>> intersection;
	gte::Line<3, float> line = Line_(o, v);

	auto result = intersection(line, Triangle_(new Triangle(face->v2(), face->v0(), face->v1())));
	if (result.intersect == 1)
	{
		return true;
	}

	result = intersection(line, Triangle_(new Triangle(face->v3(), face->v0(), face->v2())));
	if (result.intersect == 1)
	{
		return true;
	}
	return false;


}





point   BaseBulk::Line2Para(point o, GeoV3 v, Parallelogram* face)
{
	gte::FIQuery<float, gte::Line3<float>, gte::Triangle3<float>> intersection;
	gte::Line<3, float> line = Line_(o, v);
	point temp;
	auto result = intersection(line, Triangle_(new Triangle(face->v2(), face->v0(), face->v1())));
	if (result.intersect == 1)
	{
		temp = point(result.point[0], result.point[1], result.point[2]);
		return temp;
	}

	result = intersection(line, Triangle_(new Triangle(face->v3(), face->v0(), face->v2())));

	if (result.intersect == 1)
	{
		temp = point(result.point[0], result.point[1], result.point[2]);
		return temp;
	}

}

void BaseBulk::Print()
{

	for (int i = 0; i < face_list_.size(); i++)
	{
		face_list_[i]->Print();


	}


}


Polyface *BaseBulk::Face(int i)
{
	return face_list_[i];
}


point BaseBulk::StartPoint()
{
	return start_;
}


point BaseBulk::EndPoint()
{
	return end_;
}


point BaseBulk::ParallelCol(point p)
{

	if (IfLine2Para(p, vector_t_, Check_top_))
		return Line2Para(p, vector_t_, Check_top_);
	if (IfLine2Tri(p, vector_t_, Check_top_left_))
		return Line2Tri(p, vector_t_, Check_top_left_);
	if (IfLine2Tri(p, vector_t_, Check_top_right_))
		return Line2Tri(p, vector_t_, Check_top_right_);

	cout << "Error: Paralle wrong Collision." << endl;

}
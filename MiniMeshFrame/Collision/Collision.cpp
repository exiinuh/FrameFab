#include"Collision.h"


Collision::Collision()
{
}


Collision::Collision(ExtruderCone extruder, point start, point end)
{
	extruder_ = extruder;
	start_ = start;
	end_ = end;
}


Collision::~Collision()
{
}


void Collision::DetectFrame()
{

	range_list_.clear();
	range_state_.clear();
	extruder_.Default();

	ptr_dualgraph_ = new DualGraph(ptr_frame_);
	ptr_dualgraph_->Dualization();

	vector<DualVertex*> *dual_vertlist_ = ptr_dualgraph_->get_vert_list();
	WF_edge* temp_edge_;
	vector<Range*> temp_range_;
	vector<int> temp_state_;
	int M = ptr_dualgraph_->get_vert_list()->size();
	int Nd = M / 2;

	temp_range_.resize(Nd);
	temp_state_.resize(Nd);

	for (int i = 0; i < Nd; i++)
	{
		range_list_.push_back(temp_range_);
		range_state_.push_back(temp_state_);
	}

	for (int i = 0; i < Nd; i++)
	{
		cout << i << endl;
		//Fix Ployhedron
		temp_edge_ = ptr_frame_->GetEdge((*dual_vertlist_)[i]->original_id());
		start_ = temp_edge_->pvert_->Position();
		end_ = temp_edge_->ppair_->pvert_->Position();
		//Exception
		if (Geometry::angle((start_ - end_), GeoV3(0, 0, 1)) == 0)
		{
			for (int j = 0; j < Nd; j++)
			{
				range_state_[i][j] = -1;
				Range temp_range;
				temp_range = { pi / 2, pi / 2, pi / 2, pi / 2 };
				range_list_[i][j] = &temp_range;
			}
			continue;
		}

		GeneralFace();

		for (int j = 0; j< Nd; j++)
		{
			//For Specific Edge
			Range temp_range;
			WF_edge* ttemp_edge_ = ptr_frame_->GetEdge((*dual_vertlist_)[j]->original_id());
			target_start_ = ttemp_edge_->pvert_->Position();
			target_end_ = ttemp_edge_->ppair_->pvert_->Position();
			switch (DetectCollision())
			{
			case 0:
				range_state_[i][j] = 0;
				temp_range = { pi - extruder_.wave_angle_, pi / 2, pi / 2, pi / 2 + extruder_.wave_angle_ };
				range_list_[i][j] = &temp_range;
				break;

			case 1:
				range_state_[i][j] = 1;
				temp_range = allowed_angle_;
				range_list_[i][j] = &temp_range;
				break;

			case 2:
				range_state_[i][j] = 2;
				temp_range = { -1, -1, -1, -1 };
				break;

			default:
				cout << DetectCollision() << endl;
				cout << collision_point_.size() << endl;
				//top_right_t0.Print(); top_right_t1.Print();
				Print();
				cout << "-w- " << 2333333333333 << endl;
				system("pause");
				break;
			}
		}
	}
}


int Collision::DetectCollision()
{
	
	JudgeIntersection();
	CheckConllisionlist();

	//Exception Situation  

	if ((target_end_ == start_&&target_start_ == end_) || (target_end_ ==end_&&target_start_ == start_))
		return 0;

	//Normal Situation
	if (collision_point_.size() == 0)
	{
		if (Inside(target_start_) && Inside(target_end_))
		{
			double angle_1 = Angle(target_start_);
			double angle_2 = Angle(target_end_);
			if (JointAngle(angle_1, angle_2))
				return 1;
			else
				return 2;
		}
        return 0;
	}

	if (collision_point_.size() == 1)
	{
		
		point inside;
		if (Inside(target_start_) && !Equal(collision_point_[0], target_start_))
		{
			inside = target_start_;
		}
		else if (Inside(target_end_) && !Equal(collision_point_[0], target_end_))
		{
			inside = target_end_;
		}
		else
		{
			return 0;
		}

		double angle_1, angle_2;

		if (Equal(collision_point_[0], start_) || Equal(collision_point_[0], end_))
		{
			angle_1 = Angle(inside); angle_2 = angle_1;
		}
		else
		{
			angle_1 = Angle(inside); angle_2 = Angle(collision_point_[0]);
		}
		if (JointAngle(angle_1, angle_2))
			return 1;
		else
			return 2;
	}

	if (collision_point_.size() == 2)
	{
		
		double angle_1, angle_2;
		if ((Equal(collision_point_[0], start_) && Equal(collision_point_[1], end_)) ||
			(Equal(collision_point_[0], end_) && Equal(collision_point_[1], start_)))
		{
			return 0;
		}

		if (Equal(collision_point_[0], start_) || Equal(collision_point_[0], end_))
		{
			angle_1 = Angle(collision_point_[1]); angle_2 = angle_1;
			if (JointAngle(angle_1, angle_2))
				return 1;
			else
				return 2;
		}
		else if (Equal(collision_point_[1], start_) || Equal(collision_point_[1], end_))
		{
			angle_1 = Angle(collision_point_[0]); angle_2 = angle_1;
			if (JointAngle(angle_1, angle_2))
				return 1;
			else
				return 2;
		}
		else
		{
			angle_1 = Angle(collision_point_[0]); angle_2 = Angle(collision_point_[1]);
			if (JointAngle(angle_1, angle_2))
				return 1;
			else
				return 2;
		}
	}



	return -1;
}


void Collision::GeneralFace()
{
	if (end_.z() < start_.z())
	{
		point temp = start_;
		start_ = end_;
		end_ = temp;
	}

	double generatrix = extruder_.height_ / cos(extruder_.angle_);
	double radii = extruder_.height_*tan(extruder_.angle_);
	vector_t_ = end_ - start_;
	vector_t_.normalize();
	vector_z_ = point{ 0.0, 0.0, 1.0 };
	vector_tz_ = cross(vector_t_, vector_z_);
	vector_tzz_ = cross(vector_tz_, vector_z_);
   //triangle front; 
	GeoV3 front_face_right = GeoV3(start_) + vector_tzz_*(radii) 
		+ vector_z_*(extruder_.height_*cos(extruder_.wave_angle_))
		+ vector_tz_*(extruder_.height_*sin(extruder_.wave_angle_));
	GeoV3 front_face_left = GeoV3(start_) + vector_tzz_*(radii) 
		+ vector_z_*(extruder_.height_*cos(extruder_.wave_angle_))
		+ vector_tz_*(-extruder_.height_*sin(extruder_.wave_angle_));
	front_ = Triangle(GeoV3(start_), front_face_right, front_face_left);

	//triangle back;
	GeoV3 back_face_right = GeoV3(end_) + vector_tzz_*(-radii)
		+vector_z_*(extruder_.height_*cos(extruder_.wave_angle_))
		+ vector_tz_*(extruder_.height_*sin(extruder_.wave_angle_));
	GeoV3 back_face_left = GeoV3(end_) + vector_tzz_*(-radii)
		+vector_z_*(extruder_.height_*cos(extruder_.wave_angle_))
		+ vector_tz_*(-extruder_.height_*sin(extruder_.wave_angle_));
	back_ = Triangle(GeoV3(end_), back_face_left, back_face_right);

	//parallelogram left;
	GeoV3 start_left = GeoV3(start_) + vector_z_*(generatrix*cos(extruder_.angle_ + extruder_.wave_angle_))
		+ vector_tz_*(-generatrix*sin(extruder_.angle_ + extruder_.wave_angle_));
	GeoV3 end_left = GeoV3(end_) + vector_z_*(generatrix*cos(extruder_.angle_ + extruder_.wave_angle_))
		+ vector_tz_*(-generatrix*sin(extruder_.angle_ + extruder_.wave_angle_));
	left_ = Parallelogram(start_left, end_left, end_, start_); 

	//parallelogram right;
	GeoV3 start_right = GeoV3(start_) + vector_z_*(generatrix*cos(extruder_.angle_ + extruder_.wave_angle_))
		+ vector_tz_*(generatrix*sin(extruder_.angle_ + extruder_.wave_angle_));
	GeoV3 end_right = GeoV3(end_) + vector_z_*(generatrix*cos(extruder_.angle_ + extruder_.wave_angle_))
		+ vector_tz_*(generatrix*sin(extruder_.angle_ + extruder_.wave_angle_));
	right_ = Parallelogram(start_right, end_right, end_, start_);

	// triangle corner_start_right, corner_start_left
	corner_start_right_ = Triangle(GeoV3(start_), front_face_right, start_right);
	corner_start_left_ = Triangle(GeoV3(start_), start_left, front_face_left);

	// triangle corner_end_right, corner_end_left;
	corner_end_right_ = Triangle(GeoV3(end_), end_right, back_face_right);
	corner_end_left_ = Triangle(GeoV3(end_), back_face_left, end_left);

	//parallelogram top, top_left, top_right
	top_ = Parallelogram(front_face_left, back_face_left, back_face_right, front_face_right);
	top_left_ = Parallelogram(start_left, end_left, back_face_left, front_face_left);
	top_right_ = Parallelogram(front_face_right, back_face_right, end_right, start_right);

	//triangle  top
	top_left_t0 = Triangle(back_face_left, front_face_left, end_left);
	top_left_t1 = Triangle(front_face_left, start_left, end_left);

	top_right_t0 = Triangle(back_face_right,end_right,front_face_right);
	top_right_t1 = Triangle(end_right,start_right,front_face_right);
}


void Collision::JudgeIntersection()
{
	point temp;
	//front
	collision_point_.clear();
	collision_state_.clear();

	if (IfIntersect(front_, target_start_, target_end_))
	{
		temp = Intersect(front_, target_start_, target_end_);
		if (!CheckPoint(temp, collision_point_))
		{
			collision_point_.push_back(temp);
			collision_state_.push_back(0);
		}
	}
	//back
	if (IfIntersect(back_, target_start_, target_end_))
	{
		temp = Intersect(back_, target_start_, target_end_);
		if (!CheckPoint(temp, collision_point_))
		{
			collision_point_.push_back(temp);
			collision_state_.push_back(1);
		}
	}
	//corner_start_right, corner_start_left, corner_end_right, corner_end_left;//2 3 4 5
	if (IfIntersect(corner_start_right_, target_start_, target_end_))
	{
		temp = Intersect(corner_start_right_, target_start_, target_end_);
		if (!CheckPoint(temp, collision_point_))
		{
			collision_point_.push_back(temp);
			collision_state_.push_back(2);
		}
	}

	if (IfIntersect(corner_start_left_, target_start_, target_end_))
	{
		temp = Intersect(corner_start_left_, target_start_, target_end_);
		if (!CheckPoint(temp, collision_point_))
		{
			collision_point_.push_back(temp);
			collision_state_.push_back(3);
		}
	}
	if (IfIntersect(corner_end_right_, target_start_, target_end_))
	{
		temp = Intersect(corner_end_right_, target_start_, target_end_);
		if (!CheckPoint(temp, collision_point_))
		{
			collision_point_.push_back(temp);
			collision_state_.push_back(4);
		}
	}
	if (IfIntersect(corner_end_left_, target_start_, target_end_))
	{
		temp = Intersect(corner_end_left_, target_start_, target_end_);
		if (!CheckPoint(temp, collision_point_))
		{
			collision_point_.push_back(temp);
			collision_state_.push_back(5);
		}
	}
	//parallelogram left, right; //6 7
	if (IfIntersect(left_, target_start_, target_end_))
	{
		temp = Intersect(left_, target_start_, target_end_);
		if (!CheckPoint(temp, collision_point_))
		{
			collision_point_.push_back(temp);
			collision_state_.push_back(6);
		}
	}
	if (IfIntersect(right_, target_start_, target_end_))
	{
		temp = Intersect(right_, target_start_, target_end_);
		if (!CheckPoint(temp, collision_point_))
		{
			collision_point_.push_back(temp);
			collision_state_.push_back(7);
		}
	}

	//parallelogram top, top_left, top_right;//8 9 10
	if (IfIntersect(top_, target_start_, target_end_))
	{
		temp = Intersect(top_, target_start_, target_end_);
		if (!CheckPoint(temp, collision_point_))
		{
			collision_point_.push_back(temp);
			collision_state_.push_back(8);
		}
	}

	if (IfIntersect(top_left_t0, target_start_, target_end_))
	{
		temp = Intersect(top_left_t0, target_start_, target_end_);
		if (!CheckPoint(temp, collision_point_))
		{
			collision_point_.push_back(temp);
			collision_state_.push_back(11);
		}
	}

	if (IfIntersect(top_left_t1, target_start_, target_end_))
	{
		temp = Intersect(top_left_t1, target_start_, target_end_);
		if (!CheckPoint(temp, collision_point_))
		{
			collision_point_.push_back(temp);
			collision_state_.push_back(12);
		}


	}

	if (IfIntersect(top_right_t0, target_start_, target_end_))
	{
		temp = Intersect(top_right_t0, target_start_, target_end_);
		if (!CheckPoint(temp, collision_point_))
		{
			collision_point_.push_back(temp);
			collision_state_.push_back(13);
		}
	}
	
	if (IfIntersect(top_right_t1, target_start_, target_end_))
	{
		temp = Intersect(top_right_t1, target_start_, target_end_);
		if (!CheckPoint(temp, collision_point_))
		{
			collision_point_.push_back(temp);
			collision_state_.push_back(14);
		}
	}



}


bool Collision::IfIntersect(Triangle face, point start, point end)
{
	gte::FIQuery<float, gte::Segment3<float>, gte::Triangle3<float>> intersection;
	auto result = intersection(Segement_(start, end), Triangle_(face));
	//if (result.intersect)
	//{
	//	if (result.point == Triangle_(face).v[0])
	//		return false;
	//}
	return result.intersect;
}


bool Collision::IfIntersect(Parallelogram face, point start, point end)
{
	gte::FIQuery<float, gte::Segment3<float>, gte::Triangle3<float>> intersection;
	auto result = intersection(Segement_(start, end), Triangle_(face.t0_));
	if (result.intersect == 1)
	{
		return result.intersect;
	}
	result = intersection(Segement_(start, end), Triangle_(face.t1_));
	return result.intersect;
}


point Collision::Intersect(Triangle face, point start, point end)
{
	gte::FIQuery<float, gte::Segment3<float>, gte::Triangle3<float>> intersection;
	auto result = intersection(Segement_(start, end), Triangle_(face));
	point temp(result.point[0], result.point[1], result.point[2]);
	//temp.x = result.point[0]; temp.y = result.point[1]; temp.z() = result.point[2];
	return temp;
}


point Collision::Intersect(Parallelogram face, point start, point end)
{
	gte::FIQuery<float, gte::Segment3<float>, gte::Triangle3<float>> intersection;
	auto result = intersection(Segement_(start, end), Triangle_(face.t0_));
	point temp;
	if (result.intersect)
	{
		point temp(result.point[0], result.point[1], result.point[2]);
		//temp.x = result.point[0]; temp.y = result.point[1]; temp.z() = result.point[2];
		return temp;
	}
	else
	{
		auto result = intersection(Segement_(start, end), Triangle_(face.t1_));
		point temp(result.point[0], result.point[1], result.point[2]);
		//temp.x = result.point[0]; temp.y = result.point[1]; temp.z() = result.point[2];
		return temp;
	}
}


bool Collision::Inside(point p)
{
	vector<point> inside_collision;
	vector<int> state;
	point temp;
	point p_end(p.x(), p.y(), MAX);
	//front
	if (IfIntersect(front_, p, p_end))
	{
		temp = Intersect(front_, p, p_end);
		if (!CheckPoint(temp, inside_collision))
		{
			inside_collision.push_back(temp);
			state.push_back(0);
		}
	}
	//back
	if (IfIntersect(back_, p, p_end))
	{
		temp = Intersect(back_, p, p_end);
		if (!CheckPoint(temp, inside_collision))
		{
			inside_collision.push_back(temp);
			state.push_back(1);
		}
	}
	//corner_start_right, corner_start_left, corner_end_right, corner_end_left;//2 3 4 5
	if (IfIntersect(corner_start_right_, p, p_end))
	{
		temp = Intersect(corner_start_right_, p, p_end);
		if (!CheckPoint(temp, inside_collision))
		{
			inside_collision.push_back(temp);
			state.push_back(2);
		}
	}

	if (IfIntersect(corner_start_left_, p, p_end))
	{
		temp = Intersect(corner_start_left_, p, p_end);
		if (!CheckPoint(temp, inside_collision))
		{
			inside_collision.push_back(temp);
			state.push_back(3);
		}
	}
	if (IfIntersect(corner_end_right_, p, p_end))
	{
		temp = Intersect(corner_end_right_, p, p_end);
		if (!CheckPoint(temp, inside_collision))
		{
			inside_collision.push_back(temp);
			state.push_back(4);
		}
	}
	if (IfIntersect(corner_end_left_, p, p_end))
	{
		temp = Intersect(corner_end_left_, p, p_end);
		if (!CheckPoint(temp, inside_collision))
		{
			inside_collision.push_back(temp);
			state.push_back(5);
		}
	}
	//parallelogram left, right; //6 7
	if (IfIntersect(left_, p, p_end))
	{
		temp = Intersect(left_, p, p_end);
		if (!CheckPoint(temp, inside_collision))
		{
			inside_collision.push_back(temp);
			state.push_back(6);
		}
	}
	if (IfIntersect(right_, p, p_end))
	{
		temp = Intersect(right_, p, p_end);
		if (!CheckPoint(temp, inside_collision))
		{
			inside_collision.push_back(temp);
			state.push_back(7);
		}
	}

	//parallelogram top, top_left, top_right;//8 9 10
	if (IfIntersect(top_, p, p_end))
	{
		temp = Intersect(top_, p, p_end);
		if (!CheckPoint(temp, inside_collision))
		{
			inside_collision.push_back(temp);
			state.push_back(8);
		}
	}



	if (IfIntersect(top_left_t0, p, p_end))
	{
		temp = Intersect(top_left_t0, p, p_end);
		if (!CheckPoint(temp, inside_collision))
		{
			inside_collision.push_back(temp);
			state.push_back(11);
		}
	}


	if (IfIntersect(top_left_t1, p, p_end))
	{
		temp = Intersect(top_left_t1, p, p_end);
		if (!CheckPoint(temp, inside_collision))
		{
			inside_collision.push_back(temp);
			state.push_back(12);
		}
	}


	if (IfIntersect(top_right_t0, p, p_end))
	{
		temp = Intersect(top_right_t0, p, p_end);
		if (!CheckPoint(temp, inside_collision))
		{
			inside_collision.push_back(temp);
			state.push_back(13);
		}
	}

	if (IfIntersect(top_right_t1, p, p_end))
	{
		temp = Intersect(top_right_t1, p, p_end);
		if (!CheckPoint(temp, inside_collision))
		{
			inside_collision.push_back(temp);
			state.push_back(14);
		}
	}
	if (state.size() == 1)
		return true;
	return false;
}


bool Collision::JointAngle(double angle_1, double angle_2)
{
	double min_ = min(angle_1, angle_2);

	double max_ = max(angle_1, angle_2);
	allowed_angle_.right_end = min_ - extruder_.angle_;
	allowed_angle_.right_begin = pi / 2 - extruder_.wave_angle_;

	allowed_angle_.left_begin = max_ + extruder_.angle_;
	allowed_angle_.left_end = pi / 2 + extruder_.wave_angle_;

	if (allowed_angle_.right_begin >= allowed_angle_.right_end && 
			allowed_angle_.left_begin >= allowed_angle_.left_end)
		return false;

	if (allowed_angle_.right_begin >= allowed_angle_.right_end)
	{
		allowed_angle_.right_begin = -1.0;
		allowed_angle_.right_end = -1.0;
	}

	if (allowed_angle_.left_begin >= allowed_angle_.left_end)
	{
		allowed_angle_.left_begin = -1.0;
		allowed_angle_.left_end = -1.0;
	}

	return true;

}


double Collision::Angle(point intersection)
{
	GeoV3 temp = GeoV3(intersection - start_);
	temp = temp - vector_t_*(Geometry::dot(temp, vector_t_));
	return Geometry::angle(temp, vector_tz_);
}


double Collision::Distance(Triangle face, point start, point end)
{
	gte::DCPQuery<double, gte::Segment3<double>, gte::Triangle3<double>> query;
	
	gte::Segment <3, double> line;
	line.p[0][0] = start.x(); line.p[0][1] = start.y(); line.p[0][2] = start.z();
	line.p[1][0] = end.x(); line.p[1][1] = end.y(); line.p[1][2] = end.z();

	gte::Triangle<3, double> triangle;
	triangle.v[0][0] = face.v0_.x(); triangle.v[0][1] = face.v0_.y(); triangle.v[0][2] = face.v0_.z();
	triangle.v[1][0] = face.v1_.x(); triangle.v[1][1] = face.v1_.y(); triangle.v[1][2] = face.v1_.z();
	triangle.v[2][0] = face.v2_.x(); triangle.v[2][1] = face.v2_.y(); triangle.v[2][2] = face.v2_.z();

	auto result = query(line, triangle);
	
	std::cout << result.distance << std::endl;
	return result.distance;
}


double Collision::Distance(Parallelogram face, point start, point end)
{
	double t0 = Distance(face.t0_, start, end);
	double t1 = Distance(face.t1_, start, end);
	if (t1 > t0)
		return t0;
	else
		return t1;
}

//Check the point is in the vector
bool Collision::CheckPoint(point temp, vector<point> collision_point)
{
	point temp_;
	for (int i = 0; i < collision_point.size(); i++)
	{
		temp_ = collision_point[i];
		if (Equal(temp_, temp))
			return true;
	}
	return false;
}


void Collision::CheckConllisionlist()
{
	//do rank;
	vector<point> temp_;
	if (collision_point_.size() > 2)
	{
		point min = collision_point_[0];
		point max = collision_point_[0];
		point temp;
		for (int i = 0; i < collision_point_.size(); i++)
		{
			temp = collision_point_[i];
			if ((min - target_start_).length() >(temp - target_start_).length())
				min = temp;
			if ((max - target_start_).length() < (temp - target_start_).length())
				max = temp;
		}

		temp_.push_back(min);
		temp_.push_back(max);
		collision_point_ = temp_;
	}
}


void Collision::ConeSegementTest()
{
	gte::Cone<3, float> extruder;
	extruder.ray.origin = { 0.0f, 0.0f, 0.0f };
	extruder.ray.direction = { 0.0f, 0.0f, 1.0f};
    extruder.SetAngle(pi/6);
	extruder.height =2.0;

	gte::Segment<3, float> line;
	line.p[0][0] =0.0; line.p[0][1] = 0.0; line.p[0][2] = 5.0;
	line.p[1][0] =0.0; line.p[1][1] = 0.0; line.p[1][2] =-5.0;

	gte::FIQuery<float, gte::Segment3<float>, gte::Cone3<float>> intersection;
	intersection(line, extruder);
	auto result = intersection(line, extruder);
	cout << result.point[0][0] << result.point[0][1] << result.point[0][2] << endl;
	cout << result.point[1][0] << result.point[1][1] << result.point[1][2] << endl;
}


void Collision::SegementTriangleTest()
{
	gte::Segment<3, float> line;
	line.p[0][0] = 0.0; line.p[0][1] = 0.0; line.p[0][2] = 0;
	line.p[1][0] = 1.0; line.p[1][1] = 0.0; line.p[1][2] = 0;

	gte::Triangle<3, float> triangle;
	triangle.v[0][0] = 0; triangle.v[0][1] = 0; triangle.v[0][2] = 0;
	triangle.v[1][0] = 1; triangle.v[1][1] = 0; triangle.v[1][2] = 0;
	triangle.v[2][0] = 0; triangle.v[2][1] = 1; triangle.v[2][2] = 0;

	gte::FIQuery<float, gte::Segment3<float>, gte::Triangle3<float>> intersection;
	auto result = intersection(line, triangle);
	cout << result.intersect << endl;
	cout << result.point[0] << result.point[1] << result.point[2] << endl;
	//cout << result.point[1][0] << result.point[1][1] << result.point[1][2] << endl;
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


gte::Triangle<3, float>  Collision::Triangle_(Triangle face)
{
	gte::Triangle<3, float> triangle;
	triangle.v[0][0] = face.v0_.x(); 
	triangle.v[0][1] = face.v0_.y(); 
	triangle.v[0][2] = face.v0_.z();
	triangle.v[1][0] = face.v1_.x(); 
	triangle.v[1][1] = face.v1_.y(); 
	triangle.v[1][2] = face.v1_.z();
	triangle.v[2][0] = face.v2_.x(); 
	triangle.v[2][1] = face.v2_.y(); 
	triangle.v[2][2] = face.v2_.z();
	return triangle;
}


void Collision::Test()
{
	range_list_.clear();
	range_state_.clear();
	extruder_.Default();
	//DetectFrame();

	cout << 23333333333 << endl;
}


void Collision::Print()
{
	front_.Print();
	back_.Print();
	corner_start_right_.Print();
	corner_start_left_.Print();
	corner_end_right_.Print();
	corner_end_left_.Print();
	left_.Print();
	right_.Print();
	top_.Print();
	top_left_.Print();
	top_right_.Print();
}












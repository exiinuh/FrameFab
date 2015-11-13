#include"Collision.h"


Collision::Collision()
{
}

Collision::Collision(WireFrame  *ptr_frame, DualGraph *ptr_dualgraph)
{
	extruder_ = new ExtruderCone();
	ptr_frame_ = ptr_frame;
	ptr_dualgraph_ = ptr_dualgraph; 

	range_list_ = new vector<vector<Range*>>;
	range_state_ = new vector<vector<int>>;
	bulk_list_ = new vector<Bulk*>;
}

/*
Collision::Collision(ExtruderCone extruder, point start, point end)
{
	start_ = start;
	end_ = end;
}
*/

Collision::~Collision()
{
}


int   Collision::AboveCollisionAnalysis(Bulk *bulk, point target_start, point target_end)
{
	collision_point_.clear();

	if (UpCollisionBulk(bulk, target_end))
		collision_point_.push_back(UpIntersectBulk(bulk, target_end));
	if (UpCollisionBulk(bulk, target_start))
		collision_point_.push_back(UpIntersectBulk(bulk, target_start));

	if (collision_point_.size() == 1)
	{
		//add point 
		if (UpCollisionBulk(bulk, target_end))
		{
			collision_point_.push_back(point(target_start.x(), target_start.y(), collision_point_[0].z()));
		}
		else
		{
			collision_point_.push_back(point(target_end.x(), target_end.y(), collision_point_[0].z()));
        }
	}

		double angle_1, angle_2;
     	angle_1 = bulk->Angle(collision_point_[0]);
	    angle_2 = bulk->Angle(collision_point_[1]);
			if (JointAngle(angle_1, angle_2))
				return 1;
			else
				return 2;
		}

int Collision::AboveDetection(Bulk *bulk, point target_start, point target_end)
{
	if (!UpCollisionBulk(bulk, target_end) && !UpCollisionBulk(bulk, target_start))
		return 0;


}

void Collision::DetectFrame()
{
	cout << " Detect Collision begins..." << endl;

	double height = extruder_->Height();
	double angle = extruder_->Angle();
	double wave_angle = extruder_->WaveAngle();
	double generatrix = height / angle;
	double radii = height*tan(angle);

	range_list_->clear();
	range_state_->clear();
	bulk_list_->clear();

	vector<Range*> temp_range_, org_temp_range_;
	vector<int> temp_state_, org_temp_state_;

	int M = ptr_frame_->SizeOfEdgeList();
	int Nd = ptr_dualgraph_->SizeOfVertList();

	temp_range_.resize(Nd);
	temp_state_.resize(Nd);
	org_temp_range_.resize(2 * Nd);
	org_temp_state_.resize(2 * Nd);

	for (int i = 0; i < Nd; i++)
	{
		range_list_->push_back(temp_range_);
		range_state_->push_back(temp_state_);
	}

	for (int i = 0; i < Nd; i++)
	{

		//if (i%int(Nd / 10) == 0)
		//	cout << double(i) / double(Nd) * 100 << "%" << endl;

		//Fix Polyhedron
		WF_edge *e = ptr_frame_->GetEdge(ptr_dualgraph_->e_orig_id(i));
		point start = e->pvert_->Position();
		point end = e->ppair_->pvert_->Position();
		Bulk *bulk;

		//Exception

		if (Equal(Geometry::angle((start - end), GeoV3(0, 0, 1)), 0.0) || Equal(Geometry::angle((start - end), GeoV3(0, 0, 1)), pi))
		{
			bulk = NULL;
			bulk_list_->push_back(bulk);
			//for dual_list
			for (int j = 0; j < Nd; j++)
			{
				(*range_state_)[i][j] = -1;
				Range *temp_range;
				temp_range = new Range{ pi / 2, pi / 2, pi / 2, pi / 2 };
				(*range_list_)[i][j] = temp_range;
			}
			continue;
		}

		bulk = new Bulk(extruder_, start, end);
		bulk_list_->push_back(bulk);

		for (int j = 0; j< Nd; j++)
		{

			//For Specific Edge
			Range *temp_range = new Range();
			WF_edge *e_ij = ptr_frame_->GetEdge(ptr_dualgraph_->e_orig_id(j));
			point target_start = e_ij->pvert_->Position();
			point target_end = e_ij->ppair_->pvert_->Position();

			switch (DetectCollision(bulk, target_start, target_end))
			{
				//for dual_list
			case 0:
				if ((!AboveCollisionBulk(bulk, target_end) && !AboveCollisionBulk(bulk, target_start))&&(UpCollisionBulk(bulk, target_end) || UpCollisionBulk(bulk, target_start))  )
				{
					if (AboveCollisionAnalysis(bulk,target_start, target_end) == 2)
					{ 
						(*range_state_)[i][j] = 2;
						temp_range = new Range{ -1, -1, -1, -1 };
						(*range_list_)[i][j] = temp_range;
					}
					else
                    {
						(*range_state_)[i][j] = 1;
						*temp_range = allowed_angle_;
						(*range_list_)[i][j] = temp_range;
					}	
					break;
				}
				else
				{
					(*range_state_)[i][j] = 0;
				}
				   
				temp_range = new Range{ pi/2 - wave_angle, pi / 2, pi / 2, pi / 2 + wave_angle };
				(*range_list_)[i][j] = temp_range;
				break;

			case 1:
				(*range_state_)[i][j] = 1;
				*temp_range = allowed_angle_;
				(*range_list_)[i][j] = temp_range;
				break;

			case 2:
				(*range_state_)[i][j] = 2;
				temp_range = new Range{ -1, -1, -1, -1 };
				(*range_list_)[i][j] = temp_range;
				break;

			default:
				/*
				cout << DetectCollision() << endl;
				cout << collision_point_.size() << endl;
				//top_right_t0_.Print(); top_right_t1_.Print();
				Print();
				cout << "-w- " << 2333333333333 << endl;
				*/
				std::system("pause");
				break;
			}
		}
	}


	Range temp = Range{ pi/2 - wave_angle, pi / 2, pi / 2, pi / 2 + wave_angle };
	Range temp_ = Range{ -1, -1, -1, -1 };
	/*for (int i = 0; i < (*bulk_list_)[33]->face_list_.size(); i++)
	{
		(*bulk_list_)[33]->face_list_[i]->Print();
	}

	DetectCollision((*bulk_list_)[33], ptr_frame_->GetEdge(ptr_dualgraph_->e_orig_id(80))->pvert_->Position(), ptr_frame_->GetEdge(ptr_dualgraph_->e_orig_id(80))->ppair_->pvert_->Position());

*/
	for (int i = 0; i < Nd; i++)
	{
		cout << i << endl;
		cout << "--------------------------" << endl;
		for (int j = 0; j < Nd; j++)
		{
			if (!Equal(*(*range_list_)[i][j], temp) && !Equal(*(*range_list_)[i][j], temp_))
			Print(*(*range_list_)[i][j]);
		}

	}


	cout << endl;
	cout << " Detect Collision done." << endl;
}

bool Collision::AboveCollisionBulk(Bulk *bulk, point p)
{
	point p_end(p.x(), p.y(), MAX);

	for (int i = 6; i<9; i++)
	{
		if (bulk->IfTriIntersect(i, p, p_end))
		{
			return true;
		}
	}
		if (bulk->IfParaIntersect(12, p, p_end))
		{
			return true;
		}
		return false;
}

point Collision::UpIntersectBulk(Bulk *bulk, point p)
{
	point p_end(p.x(), p.y(), -MAX);

	for (int i = 6; i<9; i++)
	{
		if (bulk->IfTriIntersect(i, p, p_end))
		{
			point temp = bulk->TriIntersect(i, p, p_end);
			return temp;
		}
	}

	if (bulk->IfParaIntersect(12, p, p_end))
	{
		point temp = bulk->TriIntersect(12, p, p_end);
		return temp;
	}
}

bool  Collision::UpCollisionBulk(Bulk *bulk, point p)
{
	point p_end(p.x(), p.y(), -MAX);

	for (int i = 6; i<9; i++)
	{
		if (bulk->IfTriIntersect(i, p, p_end))
		{
			
			return true;
		}
	}

	if (bulk->IfParaIntersect(12, p, p_end))
	{
		return true;
	}

	return false;
}


//for bulk
int Collision::DetectCollision(Bulk *bulk, point target_start, point target_end)
{

	point start = bulk->StartPoint();
	point end = bulk->EndPoint();

	collision_point_.clear();
	collision_state_.clear();

	for (int i = 0; i < 10; i++)
	{
		if (bulk->IfTriIntersect(i, target_start, target_end))
		{
			point temp = bulk->TriIntersect(i, target_start, target_end);
			if (!CheckPoint(temp, collision_point_))
			{
				collision_point_.push_back(temp);
				collision_state_.push_back(i);
			}
		}
	}

	for (int i = 10; i < 13; i++)
	{
		if (bulk->IfParaIntersect(i, target_start, target_end))
		{
			point temp = bulk->ParaIntersect(i, target_start, target_end);
			if (!CheckPoint(temp, collision_point_))
			{
				collision_point_.push_back(temp);
				collision_state_.push_back(i);
			}
		}
	}

	vector<point> temp_;
	if (collision_point_.size() > 2)
	{
		point min = collision_point_[0];
		point max = collision_point_[0];
		point temp;
		for (int i = 0; i < collision_point_.size(); i++)
		{
			temp = collision_point_[i];
			if ((min - target_start).length() >(temp - target_start).length())
				min = temp;
			if ((max - target_start).length() < (temp - target_start).length())
				max = temp;
		}

		temp_.push_back(min);
		temp_.push_back(max);
		collision_point_ = temp_;
	}




	//Exception Situation  
	if ((target_end == start&&target_start == end) || (target_end ==end&&target_start == start))
		return 0;

	//Normal Situation
	if (collision_point_.size() == 0)
	{
		if (Inside(bulk, target_start) && Inside(bulk, target_end))
		{
			double angle_1 = bulk->Angle(target_start);
			double angle_2 = bulk->Angle(target_end);
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
		if (Inside(bulk, target_start) && !Equal(collision_point_[0], target_start))
		{
			inside = target_start;
		}
		else if (Inside(bulk, target_end) && !Equal(collision_point_[0], target_end))
		{
			inside = target_end;
		}
		else
		{
			return 0;
		}

		double angle_1, angle_2;

		if (Equal(collision_point_[0], start) || Equal(collision_point_[0], end))
		{
			angle_1 = bulk->Angle(inside); 
			angle_2 = angle_1;
		}
		else
		{
			angle_1 = bulk->Angle(inside);
			angle_2 = bulk->Angle(collision_point_[0]);
		}
		if (JointAngle(angle_1, angle_2))
			return 1;
		else
			return 2;
	}

	if (collision_point_.size() == 2)
	{
		
		double angle_1, angle_2;
		if ((Equal(collision_point_[0], start) && Equal(collision_point_[1], end)) ||
			(Equal(collision_point_[0], end) && Equal(collision_point_[1], start)))
		{
			return 0;
		}

		if (Equal(collision_point_[0], start) || Equal(collision_point_[0], end))
		{
			angle_1 = bulk->Angle(collision_point_[1]); 
			angle_2 = angle_1;
			if (JointAngle(angle_1, angle_2))
				return 1;
			else
				return 2;
		}
		else if (Equal(collision_point_[1], start) || Equal(collision_point_[1], end))
		{
			angle_1 = bulk->Angle(collision_point_[0]); 
			angle_2 = angle_1;
			if (JointAngle(angle_1, angle_2))
				return 1;
			else
				return 2;
		}
		else
		{
			angle_1 = bulk->Angle(collision_point_[0]); 
			angle_2 = bulk->Angle(collision_point_[1]);
			if (JointAngle(angle_1, angle_2))
				return 1;
			else
				return 2;
		}
	}
	return -1;
}


bool Collision::Inside(Bulk *bulk, point p)
{
	vector<point> inside_collision;
	vector<int> state;
	point temp;
	point p_end(p.x(), p.y(), MAX);

	for (int i = 0; i < 10; i++)
	{
		if (bulk->IfTriIntersect(i, p, p_end))
		{
			point temp = bulk->TriIntersect(i, p, p_end);
			if (!CheckPoint(temp, inside_collision))
			{
				inside_collision.push_back(temp);
				state.push_back(i);
			}
		}
	}

	for (int i = 10; i < 13; i++)
	{
		if (bulk->IfParaIntersect(i, p, p_end))
		{
			point temp = bulk->ParaIntersect(i, p, p_end);

			if (!CheckPoint(temp, inside_collision))
			{
				inside_collision.push_back(temp);
				state.push_back(i);
			}
		}
	}

	if (state.size() == 1)
		return true;
	return false;
}


bool Collision::JointAngle(double angle_1, double angle_2)
{
	double height = extruder_->Height();
	double angle = extruder_->Angle();
	double wave_angle = extruder_->WaveAngle();
	double generatrix = height / cos(angle);
	double radii = height*tan(angle);

	double min_ = min(angle_1, angle_2);
	double max_ = max(angle_1, angle_2);
	allowed_angle_.right_end = min_ - angle;
	allowed_angle_.right_begin = pi / 2 - wave_angle;

	allowed_angle_.left_begin = max_ + angle;
	allowed_angle_.left_end = pi / 2 + wave_angle;

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

/*
double Collision::Distance(Triangle face, point start, point end)
{
	gte::DCPQuery<double, gte::Segment3<double>, gte::Triangle3<double>> query;
	
	gte::Segment <3, double> line;
	line.p[0][0] = start.x(); line.p[0][1] = start.y(); line.p[0][2] = start.z();
	line.p[1][0] = end.x(); line.p[1][1] = end.y(); line.p[1][2] = end.z();

	gte::Triangle<3, double> triangle;
	triangle.v[0][0] = face.v0().x(); triangle.v[0][1] = face.v0_.y(); triangle.v[0][2] = face.v0_.z();
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
*/

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


void Collision::Test()
{
	range_list_->clear();
	range_state_->clear();
	//extruder_.Default();
	//DetectFrame();

	cout << 23333333333 << endl;
}


void Collision::Print()
{/*
	current_bulk_.front_.Print();
	current_bulk_.back_.Print();
	current_bulk_.corner_start_right_.Print();
	current_bulk_.corner_start_left_.Print();
	current_bulk_.corner_end_right_.Print();
	current_bulk_.corner_end_left_.Print();
	current_bulk_.left_.Print();
	current_bulk_.right_.Print();
	current_bulk_.top_.Print();
	current_bulk_.top_left_.Print();
	current_bulk_.top_right_.Print();
	*/
}












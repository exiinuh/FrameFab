#include"Collision.h"

Collision::Collision()
{
}

Collision::Collision(DualGraph *ptr_dualgraph)
{
	extruder_ = new ExtruderCone();
	ptr_dualgraph_ = ptr_dualgraph; 

	range_list_ = new vector<vector<Range*>>;
	range_state_ = new vector<vector<int>>;
	bulk_list_ = new vector<BaseBulk*>;
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

//For bulk
int Collision::AboveCollisionAnalysis(CommonBulk *bulk, point target_start, point target_end)
{
	collision_point_.clear();

	if (bulk->IfAboveDownCol(target_end))
		collision_point_.push_back(bulk->AboveDownCol(target_end));
	if (bulk->IfAboveDownCol(target_start))
		collision_point_.push_back(bulk->AboveDownCol(target_start));

	if (collision_point_.size() == 1)
	{
		//add point 
		if (bulk->IfAboveDownCol(target_end))
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

//For SpecialBulk
int		Collision ::AboveCollisionAnalysis(SpecialBulk *SpecialBulk, point target_start, point target_end)
{
	collision_point_.clear();

	if (SpecialBulk->IfAboveDownCol(target_end))
		collision_point_.push_back(SpecialBulk->AboveDownCol(target_end));
	if (SpecialBulk->IfAboveDownCol(target_start))
		collision_point_.push_back(SpecialBulk->AboveDownCol(target_start));

	if (collision_point_.size() == 1)
	{
		//add point 
		if (SpecialBulk->IfAboveDownCol(target_end))
		{
			collision_point_.push_back(point(target_start.x(), target_start.y(), collision_point_[0].z()));
		}
		else
		{
			collision_point_.push_back(point(target_end.x(), target_end.y(), collision_point_[0].z()));
		}
	}

	double angle_1, angle_2;
	angle_1 = SpecialBulk->Angle(collision_point_[0]);
	angle_2 = SpecialBulk->Angle(collision_point_[1]);
	if (JointAngle(angle_1, angle_2))
		return 1;
	else
		return 2;
}

void Collision::DetectFrame()
{
	cout << "---------------------------" << endl;
	cout << " Detect Collision begins..." << endl;

	WireFrame *ptr_frame = ptr_dualgraph_->ptr_frame_;

	double height = extruder_->Height();
	double angle = extruder_->Angle();
	double wave_angle = extruder_->WaveAngle();
	double generatrix = height / angle;
	double radii = height*tan(angle);

	range_list_->clear();
	range_state_->clear();
	bulk_list_->clear();

	int Nd = ptr_dualgraph_->SizeOfVertList();

	for (int i = 0; i < Nd; i++)
	{
		range_list_->push_back(vector<Range*>(Nd));
		range_state_->push_back(vector<int>(Nd));
	}

	for (int i = 0; i < Nd; i++)
	{
		if (i%int(Nd / 10) == 0)
			cout << double(i) / double(Nd) * 100 << "%" << endl;

		//Fix Polyhedron
		WF_edge *e = ptr_frame->GetEdge(ptr_dualgraph_->e_orig_id(i));
		point start = e->pvert_->Position();
		point end = e->ppair_->pvert_->Position();

		CommonBulk   *bulk;
		SpecialBulk	 *special_bulk;

	     if (end.z() < start.z())
			{
				point temp = start;
				start = end;
				end = temp;
			}

		//Exception
		if (Equal(Geometry::angle((start - end), GeoV3(0, 0, 1)), 0.0) || Equal(Geometry::angle((start - end), GeoV3(0, 0, 1)), pi))
		{
			bulk = NULL;
			bulk_list_->push_back(bulk);

		
			//for dual_list
			for (int j = 0; j < Nd; j++)
			{
				Range *temp_range = new Range();
				WF_edge *e_ij = ptr_frame->GetEdge(ptr_dualgraph_->e_orig_id(j));
				point target_start = e_ij->pvert_->Position();
				point target_end = e_ij->ppair_->pvert_->Position();
				
				(*range_state_)[i][j] = 0;
				temp_range = new Range{ pi / 2 - wave_angle, pi / 2, pi / 2, pi / 2 + wave_angle }; // But in fact it is a 2 dime Optimization£¬ and we use pi/2 as its angle, fixed
				(*range_list_)[i][j] = temp_range;

			
				if (Distance(target_start, target_end, start, end) < extruder_->Height()*tan(extruder_->Angle()))
				{
					if (target_end.z()>start.z() || target_start.z() > start.z())
					{
	                          (*range_state_)[i][j] = 2;
					         temp_range = new Range{ -1,-1,-1,-1 };
					          (*range_list_)[i][j] = temp_range;
					}
				}
			}
			continue;
		}

		//Sharp Case
		if (abs(Geometry::angle(Geometry::Vector3d(0, 0, 1), Geometry::Vector3d(end - start))) < extruder_->Angle())
		{
			special_bulk = new SpecialBulk(extruder_, start, end);
			bulk_list_->push_back(special_bulk);

			for (int j = 0; j< Nd; j++)
			{
				//For Specific Edge
				Range *temp_range = new Range();
				WF_edge *e_ij = ptr_frame->GetEdge(ptr_dualgraph_->e_orig_id(j));
				point target_start = e_ij->pvert_->Position();
				point target_end = e_ij->ppair_->pvert_->Position();

				switch (DetectCollision(special_bulk, target_start, target_end))
				{
					//for dual_list
				case 0:
					//Exception Onface
					if ((special_bulk->IfAboveUpCol(target_end) 
						&& special_bulk->IfAboveDownCol(target_end)) 
						||     (special_bulk->IfAboveUpCol(target_start) && special_bulk->IfAboveDownCol(target_start)))
					{
						if (AboveCollisionAnalysis(special_bulk, target_start, target_end) == 2)
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
					if ((!special_bulk->IfAboveUpCol(target_end) 
						&& !special_bulk->IfAboveUpCol(target_start)) 
						&& (special_bulk->IfAboveDownCol(target_end) || special_bulk->IfAboveDownCol(target_start)))
					{
						if (AboveCollisionAnalysis(special_bulk, target_start, target_end) == 2)
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

					temp_range = new Range{ pi / 2 - wave_angle, pi / 2, pi / 2, pi / 2 + wave_angle };
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
					std::system("pause");
					break;
				}
			}
			continue;
		}




		//Normal
		bulk = new CommonBulk(extruder_, start, end);
		bulk_list_->push_back(bulk);
		for (int j = 0; j< Nd; j++)
		{
			//For Specific Edge
			Range *temp_range = new Range();
			WF_edge *e_ij = ptr_frame->GetEdge(ptr_dualgraph_->e_orig_id(j));
			point target_start = e_ij->pvert_->Position();
			point target_end = e_ij->ppair_->pvert_->Position();

			switch (DetectCollision(bulk, target_start, target_end))
			{
				//for dual_list
			case 0:
				//Exception Onface
				if ((bulk->IfAboveUpCol(target_end) && bulk->IfAboveDownCol(target_end)) || (bulk->IfAboveUpCol(target_start) && bulk->IfAboveDownCol(target_start)))
				{
					if (AboveCollisionAnalysis(bulk, target_start, target_end) == 2)
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
				if ((!bulk->IfAboveUpCol(target_end) && !bulk->IfAboveUpCol(target_start)) && (bulk->IfAboveDownCol(target_end) || bulk->IfAboveDownCol( target_start)))
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





	/*for (int i = 0; i < (*bulk_list_)[33]->face_list_.size(); i++)
	{
		(*bulk_list_)[33]->face_list_[i]->Print();
	}

	DetectCollision((*bulk_list_)[33], ptr_frame_->GetEdge(ptr_dualgraph_->e_orig_id(80))->pvert_->Position(), ptr_frame_->GetEdge(ptr_dualgraph_->e_orig_id(80))->ppair_->pvert_->Position());

*/


	//
	//for (int i = 0; i < Nd; i++)
	//{
	//	cout << i << endl;
	//	cout << "--------------------------" << endl;
	//	for (int j = 0; j < Nd; j++)
	//	{
	//		if (!Equal(*(*range_list_)[i][j], temp) && !Equal(*(*range_list_)[i][j], temp_))
	//		Print(*(*range_list_)[i][j]);
	//	}

	//}


	//cout << endl;
	cout << " Detect Collision done." << endl;
}

//For bulk
int Collision::DetectCollision(CommonBulk *bulk, point target_start, point target_end)
{

	point start = bulk->StartPoint();
	point end = bulk->EndPoint();

	collision_point_.clear();
	collision_state_.clear();

	for (int i = 0; i < 8; i++)
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

	for (int i = 8; i < 14; i++)
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
		if (bulk->Inside(target_start) && bulk->Inside( target_end))
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
		if (bulk->Inside(target_start) && !Equal(collision_point_[0], target_start))
		{
			inside = target_start;
		}
		else if (bulk->Inside(target_end) && !Equal(collision_point_[0], target_end))
		{
			inside = target_end;
		}
		else if (Equal(collision_point_[0], target_end) && (bulk->Inside(target_start)))
		{
			inside = target_start;
			
		}
		else if (Equal(collision_point_[0], target_start) && (bulk->Inside( target_end)))
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
			angle_2 =pi/2;
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
			angle_2 = pi/2;
			if (JointAngle(angle_1, angle_2))
				return 1;
			else
				return 2;
		}
		else if (Equal(collision_point_[1], start) || Equal(collision_point_[1], end))
		{
			angle_1 = bulk->Angle(collision_point_[0]); 
			angle_2 = pi / 2;
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


//For SpecialBulk
int     Collision::DetectCollision(SpecialBulk *SpecialBulk, point target_start, point target_end)
{

	point start = SpecialBulk->StartPoint();
	point end = SpecialBulk->EndPoint();

	collision_point_.clear();
	collision_state_.clear();

	for (int i = 0; i < 8; i++)
	{
		if (SpecialBulk->IfTriIntersect(i, target_start, target_end))
		{
			point temp = SpecialBulk->TriIntersect(i, target_start, target_end);
			if (!CheckPoint(temp, collision_point_))
			{
				collision_point_.push_back(temp);
				collision_state_.push_back(i);
			}
		}
	}

	// special point 15
	for (int i = 8; i < 15; i++)
	{
		if (SpecialBulk->IfParaIntersect(i, target_start, target_end))
		{
			point temp = SpecialBulk->ParaIntersect(i, target_start, target_end);
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
	if ((target_end == start&&target_start == end) || (target_end == end&&target_start == start))
		return 0;

	//Normal Situation
	if (collision_point_.size() == 0)
	{
		if (SpecialBulk->Inside(target_start) && SpecialBulk->Inside(target_end))
		{
			double angle_1 = SpecialBulk->Angle(target_start);
			double angle_2 = SpecialBulk->Angle(target_end);
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
		if (SpecialBulk->Inside(target_start) && !Equal(collision_point_[0], target_start))
		{
			inside = target_start;
		}
		else if (SpecialBulk->Inside(target_end) && !Equal(collision_point_[0], target_end))
		{
			inside = target_end;
		}
		else if (Equal(collision_point_[0], target_end) && (SpecialBulk->Inside(target_start)))
		{
			inside = target_start;

		}
		else if (Equal(collision_point_[0], target_start) && (SpecialBulk->Inside(target_end)))
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
			angle_1 = SpecialBulk->Angle(inside);
			angle_2 = pi / 2;
		}
		else
		{
			angle_1 = SpecialBulk->Angle(inside);
			angle_2 = SpecialBulk->Angle(collision_point_[0]);
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
			angle_1 = SpecialBulk->Angle(collision_point_[1]);
			angle_2 = pi / 2;
			if (JointAngle(angle_1, angle_2))
				return 1;
			else
				return 2;
		}
		else if (Equal(collision_point_[1], start) || Equal(collision_point_[1], end))
		{
			angle_1 = SpecialBulk->Angle(collision_point_[0]);
			angle_2 = pi / 2;
			if (JointAngle(angle_1, angle_2))
				return 1;
			else
				return 2;
		}
		else
		{
			angle_1 = SpecialBulk->Angle(collision_point_[0]);
			angle_2 = SpecialBulk->Angle(collision_point_[1]);
			if (JointAngle(angle_1, angle_2))
				return 1;
			else
				return 2;
		}
	}
	return -1;

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


double Collision::Distance(Triangle face, point start, point end)
{
	gte::DCPQuery<double, gte::Segment3<double>, gte::Triangle3<double>> query;
	
	gte::Segment <3, double> line;
	line.p[0][0] = start.x(); line.p[0][1] = start.y(); line.p[0][2] = start.z();
	line.p[1][0] = end.x(); line.p[1][1] = end.y(); line.p[1][2] = end.z();
	
	
	gte::Triangle<3, double> triangle;
	triangle.v[0][0] = face.v0().x(); triangle.v[0][1] = face.v0().y(); triangle.v[0][2] = face.v0().z();
	triangle.v[1][0] = face.v1().x(); triangle.v[1][1] = face.v1().y(); triangle.v[1][2] = face.v1().z();
	triangle.v[2][0] = face.v2().x(); triangle.v[2][1] = face.v2().y(); triangle.v[2][2] = face.v2().z();

	auto result = query(line, triangle);
	return result.distance;
}


double	 Collision::Distance(point target_start, point target_end, point start, point end)
{
	gte::DCPQuery<double, gte::Segment3<double>, gte::Segment3<double>> query;
	gte::Segment <3, double> line;
	line.p[0][0] = start.x(); line.p[0][1] = start.y(); line.p[0][2] = start.z();
	line.p[1][0] = end.x(); line.p[1][1] = end.y(); line.p[1][2] = end.z();

	gte::Segment <3, double> line_0;
	line_0.p[0][0] = target_start.x(); line_0.p[0][1] = target_start.y(); line_0.p[0][2] = target_start.z();
	line_0.p[1][0] = target_end.x(); line_0.p[1][1] = target_end.y(); line_0.p[1][2] = target_end.z();
	auto result = query(line, line_0);
	return result.distance;
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


void Collision::Debug()
{
//113
//+		position_	{v=0x06738b94 {11.0578365, 4.74550581, 9.83846569} }	trimesh::Vec<3,float>
//+		position_	{v=0x03fea654 {12.1713638, -0.765893996, 23.1887951} }	trimesh::Vec<3,float>

//105
//+		position_	{v=0x03ffefc4 {6.59659815, -2.64607501, 22.3574867} }	trimesh::Vec<3,float>
//+		position_	{v=0x0683cabc {7.71587706, -0.578864992, 30.1668682} }	trimesh::Vec<3,float>


	point a = point(11.0578365, 4.74550581, 9.83846569);
	point b = point(12.1713638, -0.765893996, 23.1887951);

	point c = point(6.59659815, -2.64607501, 22.3574867);
	point d = point(7.71587706, -0.578864992, 30.1668682);

	point c_max = point(-4.64264679, -3.46753812, MAX);

	SpecialBulk* temp = new SpecialBulk(extruder_, a,b);
	temp->Print();

    point target_start_ = c;
    point target_end_ =d;
    cout << DetectCollision(temp, target_start_, target_end_) << endl;
	
	cout << temp->Inside(d) << endl;

	cout << temp->IfAboveUpCol(d) << " " << temp->IfBelowDownCol(d) <<endl;

	//cout << IfAboveUpCol(temp, c) << endl;
	//cout << IfBelowDownCol(temp, c) << endl;

	cout << temp->IfParaIntersect(11, c, c_max) << endl;
	
	cout << temp->IfTriIntersect(1, b, c) << endl;

	//cout << ptr_dualgraph_->e_dual_id(45) << " " << ptr_dualgraph_->e_dual_id(125) << " " << ptr_dualgraph_->e_dual_id(123) << " " << endl;
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
#include"ProcessAnalyzer.h"


ProcessAnalyzer::ProcessAnalyzer(SeqAnalyzer *ptr_seqanalyzer, char *path)
{
	ptr_seqanalyzer_ = ptr_seqanalyzer;
	path_ = path;

    break_height_ = 2; //2cm
	//SetThick();

	debug_ = true;
}



void ProcessAnalyzer::ProcPrint()
{
	WireFrame *ptr_frame = ptr_seqanalyzer_->ptr_graphcut_->ptr_frame_;
	DualGraph *ptr_dualgraph = ptr_seqanalyzer_->ptr_graphcut_->ptr_dualgraph_;

	if (debug_)
	{
		ptr_dualgraph->Dualization();
		ReadLayerQueue();
	}
	else
	{
		ptr_seqanalyzer_->GetQueue(layer_queue_);
	}


	exist_point_.clear();
	double thick = 0;
	
	for (int i = 0; i <layer_queue_.size(); i++)
	{
		int orig_e = layer_queue_[i];
		WF_edge *e = ptr_frame->GetEdge(orig_e);
		
		if (i == 0)
		{
			Process* temp_process = new Process();
			if (e->pvert_->Position().z() >= e->ppair_->pvert_->Position().z())
			{
				temp_process->start_ = e->ppair_->pvert_->Position();
				temp_process->end_ = e->pvert_->Position();
			}
			else
			{
				temp_process->end_ = e->ppair_->pvert_->Position();
				temp_process->start_ = e->pvert_->Position();
			}
			temp_process->extruder_state_ = true;
			temp_process->fan_state_ = true;
			temp_process->move_state_ = 5;			
			temp_process->vector = point(0, 0, 1);
			print_queue_.push_back(temp_process);
			exist_point_.push_back(temp_process->start_);
			exist_point_.push_back(temp_process->end_);
			continue;
		}

		Process* temp_process = new Process();
		temp_process = SetPoint(e, i);
		temp_process = SetFan(temp_process);
		temp_process = SetExtruderSpeed(temp_process,i);
		temp_process = SetVector(temp_process,i);
		if (temp_process->start_ != print_queue_[print_queue_.size() - 1]->end_)
		{
		}
			//	SetBreak(temp_process);



		point m = temp_process->start_;
		point b = temp_process->end_;
		
		
		if (IfPointInVector(temp_process->start_))
			temp_process->start_ += point(0, 0, thick);

		if (IfPointInVector(temp_process->end_))
			temp_process->end_ += point(0, 0, thick);

		if (!IfPointInVector(m))
			exist_point_.push_back(m);

		if (!IfPointInVector(b))
			exist_point_.push_back(b);

		
		
		print_queue_.push_back(temp_process);

	}

	Write();
}


Process* ProcessAnalyzer::SetPoint(WF_edge *e, int id)
{
	WireFrame *ptr_frame = ptr_seqanalyzer_->ptr_graphcut_->ptr_frame_;
	DualGraph *ptr_dualgraph = ptr_seqanalyzer_->ptr_graphcut_->ptr_dualgraph_;

	Process* temp_process = new Process();
	point up, down;
	//Must
	Geometry::Vector3d vec;
	if (e->pvert_->Position().z() > e->ppair_->pvert_->Position().z())
	{
        vec=Geometry::Vector3d (e->pvert_->Position() - e->ppair_->pvert_->Position());
		down = e->ppair_->pvert_->Position();
		up = e->pvert_->Position();
	}
	else
	{
		 vec=Geometry::Vector3d(-e->pvert_->Position() + e->ppair_->pvert_->Position());
		 up = e->ppair_->pvert_->Position();
		 down = e->pvert_->Position();
	}
	

	if(e->isPillar())
	{
		temp_process->end_ = up;
		temp_process->start_ = down;
		return temp_process;

	}


	if (!IfPointInVector(e->ppair_->pvert_->Position()) && !IfPointInVector(e->pvert_->Position()))
	{
		temp_process->end_ =up;
		temp_process->start_ = down;
		return temp_process;
	}

	
	


	//if (e->pvert_->Position().z() == 0)
	//{
	//	temp_process->start_ = e->pvert_->Position();
	//	temp_process->end_ = e->ppair_->pvert_->Position();
	//	return temp_process;
	//}
	//
	//if (e->ppair_->pvert_->Position().z() == 0)
	//{
	//	temp_process->end_ = e->pvert_->Position();
	//	temp_process->start_ = e->ppair_->pvert_->Position();
	//	return temp_process;
	//}

	if (IfPointInVector(e->ppair_->pvert_->Position()) && !IfPointInVector(e->pvert_->Position()))
	{
		temp_process->end_ = e->pvert_->Position();
		temp_process->start_ = e->ppair_->pvert_->Position();
		return temp_process;
	}

	if (!IfPointInVector(e->ppair_->pvert_->Position()) && IfPointInVector(e->pvert_->Position()))
	{
		temp_process->start_ = e->pvert_->Position();
		temp_process->end_ = e->ppair_->pvert_->Position();
		return temp_process;
	}


	temp_process->start_ = up;
	temp_process->end_ = down;
	return temp_process;


	// Optimization

	Process * prior_process = print_queue_[print_queue_.size() - 1];
	if (prior_process->end_ == e->pvert_->Position())
	{
		temp_process->start_ = e->pvert_->Position();
		temp_process->end_ = e->ppair_->pvert_->Position();
		return temp_process;
	}
	
	if (prior_process->end_ == e->ppair_->pvert_->Position())
	{
		temp_process->end_ = e->pvert_->Position();
		temp_process->start_ = e->ppair_->pvert_->Position();
		return temp_process;
	}

	if (id == layer_queue_.size() - 1)
	{
		temp_process->end_ = up;
		temp_process->start_ = down;
		return temp_process;
	}
	
	
	int orig_e = layer_queue_[id + 1];
	WF_edge *later_e = ptr_frame->GetEdge(orig_e);

	if (later_e->pvert_->Position() == e->pvert_->Position())
	{
		temp_process->end_ = e->pvert_->Position();
		temp_process->start_ = e->ppair_->pvert_->Position();
		return temp_process;
	}

	if (later_e->ppair_->pvert_->Position() == e->pvert_->Position())
	{
		temp_process->end_ = e->pvert_->Position();
		temp_process->start_ = e->ppair_->pvert_->Position();
		return temp_process;
	}

	if (later_e->pvert_->Position() == e->ppair_-> pvert_->Position())
	{
		temp_process->end_ = e->ppair_->pvert_->Position();
		temp_process->start_ = e->pvert_->Position();
		return temp_process;
	}

	if (later_e->ppair_-> pvert_->Position() == e->ppair_->pvert_->Position())
	{
		temp_process->end_ = e->ppair_->pvert_->Position();
		temp_process->start_ = e->pvert_->Position();
		return temp_process;
	}


	temp_process->start_ = down;
	temp_process->end_ = up;
	return temp_process;

}

bool  ProcessAnalyzer::IfPointInVector(point p)
{
	for (int i = 0; i <exist_point_.size(); i++)
	{
		if ((exist_point_[i] - p).length()<eps)
			return true;
	}
	return false;
}

Process*   ProcessAnalyzer::SetFan(Process *temp)
{
	Process* temp_process = temp;

	if (IfPointInVector(temp_process->end_))
		temp_process->fan_state_ = false;
	else
		temp_process->fan_state_ = true;

	return temp_process;
}

Process*  ProcessAnalyzer::SetExtruderSpeed(Process* temp, int id)
{

	Process* temp_process = temp;

	temp_process->extruder_state_ = true;


	Geometry::Vector3d vec = Geometry::Vector3d(temp_process->end_ - temp_process->start_);
	double angle = Geometry::angle(vec, Geometry::Vector3d(0, 0, 1));

	if (abs(angle) < eps)
	{
		
	if((temp->end_ == point(0, 0, 0)) || temp->start_ == point(0, 0, 0))
		{
			temp_process->move_state_ = 6;
		}
	else
		temp_process->move_state_ = 5;

	}


		
	else if (abs(angle - F_PI / 2) < 0.1)
		temp_process->move_state_ = 4;
	else if (0 < angle&&angle < F_PI / 2)
	{
		if (temp_process->fan_state_ == 1)
			temp_process->move_state_ = 2;
		else
			temp_process->move_state_ = 7;

	}
		
	else if (angle>F_PI / 2)
		temp_process->move_state_ = 3;

//	temp_process->vector = ptr_seqanalyzer_->GetExtru(id).Normal();
	return temp_process;
}



void ProcessAnalyzer::SetBreak(Process* temp)
{
	//0
	Process* up_process = new Process();
	up_process->start_ = print_queue_[print_queue_.size() - 1]->end_;
	up_process->end_ = up_process->start_ + point(0,  break_height_,0);

	up_process->fan_state_ = true;
	up_process->extruder_state_ = false;
	up_process->move_state_ = 0;
	up_process->vector = print_queue_[print_queue_.size() - 1]->vector;

	print_queue_.push_back(up_process);

	//1
	Process* mid_process_0 = new Process();
	mid_process_0->start_ = print_queue_[print_queue_.size() - 1]->end_;
	mid_process_0->end_ = mid_process_0->start_ + point(0, 0,break_height_);

	mid_process_0->fan_state_ = false;
	mid_process_0->extruder_state_ = false;
	mid_process_0->move_state_ = -1;
	mid_process_0->vector = print_queue_[print_queue_.size() - 1]->vector;

	print_queue_.push_back(mid_process_0);

	//2
	Process* mid_process_1 = new Process();
	mid_process_1->start_ = print_queue_[print_queue_.size() - 1]->end_;
	mid_process_1->end_ = temp->start_;
	mid_process_1->end_.z() = mid_process_1->start_.z();

	mid_process_1->fan_state_ = false;
	mid_process_1->extruder_state_ = false;
	mid_process_1->move_state_ = -1;
	mid_process_1->vector = print_queue_[print_queue_.size() - 1]->vector;

	print_queue_.push_back(mid_process_1);

	//3
	Process* down_process = new Process();

	down_process->start_ = print_queue_[print_queue_.size() - 1]->end_;
	down_process->end_ =temp->start_;

	down_process->fan_state_ = false;
	down_process->extruder_state_ = false;
	down_process->move_state_ = -1;
	down_process->vector = print_queue_[print_queue_.size() - 1]->vector;

	print_queue_.push_back(down_process);

}

void ProcessAnalyzer::Write()
{
	string path = path_;

	string point_path	= path + "/Point.txt";
	string fan_path		= path + "/FanState.txt";
	string speed_path	= path + "/SpeedState.txt";
	string vector_path	= path + "/Vector.txt";
	string istart_path	= path + "/IStart.txt";
	string iend_path	= path + "/IEnd.txt";
	string ivector_path = path + "/IVector.txt";
	string iwave_path	= path + "/IWave.txt";
	string isupport_path= path + "/ISupport.txt";
	string ibreak_path	= path + "/IBreak.txt";
	string icut_path	= path + "/ICut.txt ";

	FILE *fp		= fopen(point_path.c_str(), "w+");
	FILE *fs		= fopen(fan_path.c_str(), "w+");
	FILE *ss		= fopen(speed_path.c_str(), "w+");
	FILE *rr		= fopen(vector_path.c_str(), "w+");
	FILE *start		= fopen(istart_path.c_str(), "w+");
	FILE *end		= fopen(iend_path.c_str(), "w+");
	FILE *vector	= fopen(ivector_path.c_str(), "w+");
	FILE *IWave		= fopen(iwave_path.c_str(), "w+");
	FILE *ISupport	= fopen(isupport_path.c_str(), "w+");
	FILE *IBreak	= fopen(ibreak_path.c_str(), "w+");
	FILE *ICut		= fopen(icut_path.c_str(), "w+");

	 fprintf(  ISupport, "%d", ptr_seqanalyzer_->GetSupport());

	if (rr == NULL)
	{
		cout << "Error: Vector.txt miss" << endl;
	}
	double thick = 0.2;


	for (int i = 0; i <print_queue_.size(); i++)
	{
		point p = print_queue_[i]->start_;
		fprintf(fp, "%lf ,%lf ,%lf", p.x(), p.y(), p.z());
		fprintf(fp, "\n");


		fprintf(start, "%lf ,%lf ,%lf", p.x(), p.y(), p.z());
		fprintf(start, "\n");


		fprintf(ICut, " %lf, %lf, %lf", p.x(), p.y(), 0.0);
		fprintf(ICut, " \n");

		p = print_queue_[i]->end_;
		fprintf(fp, "%lf ,%lf ,%lf", p.x(), p.y(), p.z());
		fprintf(fp, "\n");

		fprintf(end, "%lf ,%lf ,%lf", p.x(), p.y(), p.z());
		fprintf(end, "\n");
		

		Vec3f ve = print_queue_[i]->vector;

		fprintf(vector, "%lf ,%lf ,%lf",ve.x(), ve.y(), ve.z());
		fprintf(vector, "\n");

		Geometry::Vector3d Break(ve.x(), ve.y(), 0);
		if (Break.norm() < eps)
			Break = Geometry::Vector3d(1, 0, 0);
		Break.normalize();

		fprintf(IBreak, "%lf ,%lf ,%lf", Break.getX(), Break.getY(), Break.getZ());
		fprintf(IBreak, "\n");

		bool e = print_queue_[i]->fan_state_;
		
		fprintf(fs, "%d", e);
		fprintf(fs, "\n");

		int m = print_queue_[i]->move_state_;
		fprintf(ss, "%d", m);
		fprintf(ss, "\n");


		point v = print_queue_[i]->vector;

		fprintf(rr, "%lf, %lf, %lf", v.x(), v.y(), v.z());
		fprintf(rr, "\n");

		fprintf(rr, "%lf, %lf, %lf", v.x(), v.y(), v.z());
		fprintf(rr, "\n");


		fprintf(IWave, "%lf", ptr_seqanalyzer_->GetWave(i));
		fprintf(IWave, "\n");
	}

	fclose(fp);
	fclose(fs);
	fclose(ss);
	fclose(rr);
	fclose(start);
	fclose(end);
	fclose(vector);
	fclose(IWave);
	fclose(IBreak);
	fclose(ISupport);

	fclose(ICut);

}

void ProcessAnalyzer::SetThick()
{
	
	double thick = 0.0;
	for (int i = 0; i < print_queue_.size(); i++)
	{
		Process* temp = print_queue_[i];


		if (temp->move_state_ == 4)
		{
			print_queue_[i]->start_ += point(0, 0, thick);
			print_queue_[i]->end_ += point(0, 0, thick);
			print_queue_[i]->fan_state_ = 1;
		}

		if (temp->move_state_ == 7)
		{
			temp->move_state_ == 4;
			print_queue_[i]->start_ += point(0, 0, thick);
			print_queue_[i]->end_ += point(0, 0, thick);
			print_queue_[i]->fan_state_ = 1;
		}

		//Special
		if (temp->move_state_ == 2)
		{
			print_queue_[i]->fan_state_ = 1;
			print_queue_[i]->start_ += point(0, 0, thick);
			print_queue_[i]->end_ += point(0, 0, thick);

		}
		
		if (temp->move_state_ == 3)
		{
			temp->move_state_ == 4;
			print_queue_[i]->fan_state_ = 1;
			print_queue_[i]->start_ += point(0, 0, thick);
			print_queue_[i]->end_ += point(0, 0, thick);
		}
	}	
}


Process* ProcessAnalyzer::SetVector(Process* temp, int id)
{
   temp->wave_ = ptr_seqanalyzer_->GetWave(id);
	temp->vector = ptr_seqanalyzer_->GetNormal(id);
	return temp;
}


void ProcessAnalyzer::ReadLayerQueue()
{
	string path = path_;
	string queue_path = path + "/Queue.txt";

	FILE *fp = fopen(queue_path.c_str(), "r");

	layer_queue_.clear();
	int Nd = ptr_seqanalyzer_->ptr_graphcut_->ptr_dualgraph_->SizeOfEdgeList();
	for (int i = 0; i < Nd; i++)
	{
		int e;
		fscanf(fp, "%d", &e);
		layer_queue_.push_back(e); 
	}
	fclose(fp);
}

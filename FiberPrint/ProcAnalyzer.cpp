#include "ProcAnalyzer.h"


ProcAnalyzer::ProcAnalyzer()
{


}


ProcAnalyzer::~ProcAnalyzer()
{
}


ProcAnalyzer::ProcAnalyzer(SeqAnalyzer *seqanalyzer, char *path)
{
	ptr_seqanalyzer_ = seqanalyzer;
	path_ = path;
	debug_ = false;
}


void ProcAnalyzer::ProcPrint()
{
	//

	WireFrame *ptr_frame = ptr_seqanalyzer_->ptr_frame_;
	DualGraph *ptr_dualgraph = ptr_seqanalyzer_->ptr_dualgraph_;
	ptr_collision_ = ptr_seqanalyzer_->GetCollision();


	if (debug_)
	{
		ptr_dualgraph->Dualization();
		ReadLayerQueue();
	}
	else
	{
		ptr_seqanalyzer_->GetQueue(layer_queue_);
	}

	string path = path_;
	string seq_path = path + "/ISeq.txt";
	FILE *seq = fopen(seq_path.c_str(), "w+");
	for (int i = 0; i < layer_queue_.size(); i++)
	{
		fprintf(seq, "%d", layer_queue_[i]);
		fprintf(seq, ",");

	}

	std::fclose(seq);

	exist_point_.clear();
	process_list_.clear();
	support_ = 0;

	//angle
	for (int i = 0; i < layer_queue_.size(); i++)
	{
		Process temp;
		int orig_e = layer_queue_[i];
		WF_edge *e = ptr_frame->GetEdge(orig_e);
		if (e->isPillar())
		{
			temp.normal_.push_back(GeoV3(0, 0, 1));
			support_++;
		}
		else
		{
			temp.normal_ = ptr_collision_->DetectStructure(e, exist_edge_);
		}
		exist_edge_.push_back(e);
		process_list_.push_back(temp);
	}

	//point
	for (int i = 0; i < layer_queue_.size(); i++)
	{
		Process temp = process_list_[i];
		int orig_e = layer_queue_[i];
		WF_edge *e = ptr_frame->GetEdge(orig_e);

		point up, down;
		if ((e->pvert_->Position()).z()>(e->ppair_->pvert_->Position()).z())
		{
			up = e->pvert_->Position();
			down = e->ppair_->pvert_->Position();
		}
		else
		{
			up = e->ppair_->pvert_->Position();
			down = e->pvert_->Position();
		}

		if (e->isPillar())
		{
			temp.start_ = down;
			temp.end_ = up;
			temp.fan_state_ = true;
			if (!IfPointInVector(down))
				exist_point_.push_back(down);
			if (!IfPointInVector(up))
				exist_point_.push_back(up);
		}
		else
		{
			if (IfPointInVector(down) && IfPointInVector(up))
			{
				temp.fan_state_ = false;
				temp.start_ = up;
				temp.end_ = down;
			}
			else if (IfPointInVector(down))
			{
				temp.fan_state_ = true;
				temp.start_ = down;
				temp.end_ = up;
			}
			else
			{
				temp.fan_state_ = true;
				temp.start_ = up;
				temp.end_ = down;
			}
		}
		process_list_[i] = temp;
	}
	Write();
}

void ProcAnalyzer::ReadLayerQueue()
{
	string path = path_;
	string queue_path = path + "/Queue.txt";

	FILE *fp = fopen(queue_path.c_str(), "r");

	layer_queue_.clear();
	int Nd = ptr_seqanalyzer_->ptr_dualgraph_->SizeOfEdgeList();
	for (int i = 0; i < Nd; i++)
	{
		int e;
		fscanf(fp, "%d", &e);
		layer_queue_.push_back(e);
	}
	std::fclose(fp);
}

bool  ProcAnalyzer::IfPointInVector(point p)
{
	for (int i = 0; i <exist_point_.size(); i++)
	{
		if ((exist_point_[i] - p).length()<eps)
			return true;
	}
	return false;
}

void ProcAnalyzer::Write()
{
	string path = path_;
	string fan_path = path + "/FanState.txt";
	string istart_path = path + "/IStart.txt";
	string iend_path = path + "/IEnd.txt";
	string isupport_path = path + "/ISupport.txt";


	FILE *fans = fopen(fan_path.c_str(), "w+");
	FILE *start = fopen(istart_path.c_str(), "w+");
	FILE *end = fopen(iend_path.c_str(), "w+");
	FILE *support = fopen(isupport_path.c_str(), "w+");

	fprintf(support, "%d", support_);
	std::fclose(support);


	for (int i = 0; i < process_list_.size(); i++)
	{
		Process temp = process_list_[i];
		point p = temp.start_;
		fprintf(start, "%lf ,%lf ,%lf", p.x(), p.y(), p.z());
		fprintf(start, "\n");

		p = temp.end_;
		fprintf(end, "%lf ,%lf ,%lf", p.x(), p.y(), p.z());
		fprintf(end, "\n");

		fprintf(fans, "%d", temp.fan_state_);
		fprintf(fans, "\n");

		stringstream ss;
		string str;
		ss << i;
		ss >> str;

		string ivector_path = path + "/INormal_" + str + ".txt";
		FILE *vector = fopen(ivector_path.c_str(), "w+");
		if (temp.normal_.size() == 0)
		{
			cout << "error:normal vector empty" << endl;
			getchar();
		}
		for (int j = 0; j < temp.normal_.size(); j++)
		{
			fprintf(vector, "%lf ,%lf ,%lf", temp.normal_[j].getX(), temp.normal_[j].getY(), temp.normal_[j].getZ());
			fprintf(vector, "\n");
		}

	

		std::fclose(vector);
	}
	std::fclose(start);
	std::fclose(end);
	std::fclose(fans);

}
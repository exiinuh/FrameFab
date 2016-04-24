#include "BFAnalyzer.h"


BFAnalyzer::BFAnalyzer()
{
}


BFAnalyzer::~BFAnalyzer()
{
}


bool BFAnalyzer::SeqPrint()
{
	Init();

	/* set pillars as starting edges */
	PrintPillars();

	return GenerateSeq(print_queue_.size(), Nd_);
}


bool BFAnalyzer::GenerateSeq(int h, int t)
{
	if (h == t)
	{
		return true;
	}

	if (debug_)
	{
		printf("---searching at edge %d, head %d, (tail %d).\n", 
			print_queue_[h - 1]->ID() / 2, h, t);
	}

	if (!TestifyStiffness())
	{
		printf("...edge %d: Stiffness examination failed.\n", print_queue_[h - 1]->ID() / 2);
		return false;
	}

	WF_edge *ei = print_queue_[h - 1];
	for (int dual_j = 0; dual_j < Nd_; dual_j++)
	{
		int orig_j = ptr_wholegraph_->e_orig_id(dual_j);
		WF_edge *ej = ptr_frame_->GetEdge(orig_j);
		if (!ptr_dualgraph_->isExistingEdge(orig_j))
		{
			if (ei->pvert_ == ej->pvert_ || ei->ppair_->pvert_ == ej->pvert_
				|| ei->pvert_ == ej->ppair_->pvert_ || ei->ppair_->pvert_ == ej->ppair_->pvert_)
			{
				int free_angle = ptr_collision_->ColFreeAngle(angle_state_[dual_j]);
				if (free_angle == 0)
				{
					printf("...edge %d: collision examination failed.\n", orig_j / 2);
					return false;
				}

				print_queue_.push_back(ej);
				UpdateStructure(ej);
				vector<vector<lld>> tmp_angle(3);
				UpdateStateMap(dual_j, tmp_angle);

				if (GenerateSeq(h + 1, t))
				{
					return true;
				}

				RecoverStateMap(dual_j, tmp_angle);
				RecoverStructure(ej);
				print_queue_.pop_back();
			}
		}
	}

	return false;
}


void BFAnalyzer::PrintOutQueue(int N)
{	
	string path = ptr_path_;
	string queue_path = path + "/BruteForceQueue.txt";

	FILE *fp = fopen(queue_path.c_str(), "w");

	for (int i = 0; i < N; i++)
	{
		fprintf(fp, "%d\n", print_queue_[i]->ID() / 2);
	}

	fclose(fp);
}
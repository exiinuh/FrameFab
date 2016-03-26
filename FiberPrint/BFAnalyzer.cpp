#include "BFAnalyzer.h"


BFAnalyzer::BFAnalyzer()
{
}


BFAnalyzer::~BFAnalyzer()
{
}


bool BFAnalyzer::SeqPrint()
{
	int Nd = ptr_dualgraph_->SizeOfVertList();

	inqueue_.resize(Nd);
	fill(inqueue_.begin(), inqueue_.end(), false);

	return GenerateSeq(1, Nd);
}


bool BFAnalyzer::GenerateSeq(int h, int t)
{
	if (h > t)
	{
		return TestifySeq();
	}
	for (int i = 0; i < t; i++)
	{
		if (!inqueue_[i])
		{
			print_queue_.push_back({ 0, 0, i });
			inqueue_[i] = true;
			if (GenerateSeq(h + 1, t))
			{
				return true;
			}
			print_queue_.pop_back();
			inqueue_[i] = false;
		}
	}

	return false;
}


bool BFAnalyzer::TestifySeq()
{
	printf("--------------------------------------\n");
	printf("Test on sequence starts.\n");

	int Nd = ptr_dualgraph_->SizeOfVertList();
	
	delete ptr_subgraph_;
	ptr_subgraph_ = new DualGraph(ptr_frame_);
	
	angle_state_.resize(Nd);
	for (int i = 0; i < Nd; i++)
	{
		angle_state_[i].push_back(0);
		angle_state_[i].push_back(0);
		angle_state_[i].push_back(0);
	}

	for (int i = 0; i < Nd; i++)
	{
		int dual_i = print_queue_[i].dual_id_;
		int orig_i = ptr_dualgraph_->e_orig_id(dual_i);
		WF_edge *e = ptr_frame_->GetEdge(orig_i);

		ptr_subgraph_->UpdateDualization(e);

		/* testify collision */
		if ((~(angle_state_[dual_i][0] & angle_state_[dual_i][1]
			& angle_state_[dual_i][2])) == 0)
		{
			printf("Test on collision falied.\n");
			return false;
		}

		/* testify stiffness */
		if (!TestifyStiffness())
		{
			printf("Test on stiffness falied.\n");
			return false;
		}

		/* update collision */
		UpdateStateMap(dual_i, angle_state_);
	}

	return true;
}
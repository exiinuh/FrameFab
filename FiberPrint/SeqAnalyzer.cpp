#include "SeqAnalyzer.h"


SeqAnalyzer::SeqAnalyzer()
{
}


SeqAnalyzer::~SeqAnalyzer()
{
	delete ptr_subgraph_;
	ptr_subgraph_ = NULL;

	delete ptr_collision_;
	ptr_collision_ = NULL;
}


bool SeqAnalyzer::LayerPrint()
{
	return true;
}


void SeqAnalyzer::GetQueue(vector<int> &layer_queue)
{
	layer_queue.clear();

	int Nq = layer_queue_.size();
	for (int i = 0; i < Nq; i++)
	{
		int dual_e = layer_queue_[i].dual_id_;
		layer_queue.push_back(ptr_dualgraph_->e_orig_id(dual_e));
	}
}
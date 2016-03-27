#include "SeqAnalyzer.h"


SeqAnalyzer::SeqAnalyzer()
	:gamma_(100), Dt_tol_(0.1), Dr_tol_(10 * F_PI / 180),
	Wl_(1.0), Wp_(1.0), Wi_(1.0), extru_(false), debug_(false), fileout_(false)
{
}


SeqAnalyzer::SeqAnalyzer(GraphCut *ptr_graphcut)
	:gamma_(100), Dt_tol_(0.1), Dr_tol_(10 * F_PI / 180),
	Wl_(1.0), Wp_(1.0), Wi_(1.0), extru_(false), debug_(true), fileout_(false)
{
	ptr_frame_ = ptr_graphcut->ptr_frame_;
	ptr_dualgraph_ = ptr_graphcut->ptr_dualgraph_;

	ptr_subgraph_ = new DualGraph(ptr_frame_);
	ptr_collision_ = new QuadricCollision(ptr_frame_);
}


SeqAnalyzer::SeqAnalyzer(GraphCut *ptr_graphcut, FiberPrintPARM *ptr_parm, char *ptr_path)
{
	ptr_frame_ = ptr_graphcut->ptr_frame_;
	ptr_dualgraph_ = ptr_graphcut->ptr_dualgraph_;
	ptr_parm_ = ptr_parm;
	ptr_path_ = ptr_path;

	ptr_subgraph_ = new DualGraph(ptr_frame_);
	ptr_collision_ = new QuadricCollision(ptr_frame_);

	extru_ = false;
	debug_ = true;
	fileout_ = false;

	ptr_dualgraph_->Dualization();

	gamma_ = ptr_parm->gamma_;
	Dt_tol_ = ptr_parm->Dt_tol_;
	Dr_tol_ = ptr_parm->Dr_tol_;
	Wl_ = ptr_parm->Wl_;
	Wp_ = ptr_parm->Wp_;
	Wi_ = 1.0;
}


SeqAnalyzer::~SeqAnalyzer()
{
	delete ptr_subgraph_;
	ptr_subgraph_ = NULL;

	delete ptr_collision_;
	ptr_collision_ = NULL;
}


bool SeqAnalyzer::SeqPrint()
{
	return true;
}


void SeqAnalyzer::UpdateStateMap(int dual_i, vector<vector<lld>> &state_map)
{
	WF_edge *order_e = ptr_frame_->GetEdge(ptr_dualgraph_->e_orig_id(dual_i));
	int Nd = ptr_dualgraph_->SizeOfVertList();
	for (int dual_j = 0; dual_j< Nd; dual_j++)
	{
		int orig_j = ptr_dualgraph_->e_orig_id(dual_j);
		if (dual_i != dual_j && !ptr_subgraph_->isExistingEdge(orig_j))
		{
			WF_edge *target_e = ptr_frame_->GetEdge(orig_j);
			ptr_collision_->DetectCollision(target_e, order_e);
			for (int k = 0; k < 3; k++)
			{
				state_map[k].push_back(angle_state_[dual_j][k]);
			}
			ptr_collision_->ModifyAngle(angle_state_[dual_j]);
		}
	}
}


void SeqAnalyzer::RecoverStateMap(int dual_i, vector<vector<lld>> &state_map)
{
	int Nd = ptr_dualgraph_->SizeOfVertList();
	int p = 0;
	for (int dual_j = 0; dual_j < Nd; dual_j++)
	{
		int orig_j = ptr_dualgraph_->e_orig_id(dual_j);
		if (dual_i != dual_j && !ptr_subgraph_->isExistingEdge(orig_j))
		{
			for (int k = 0; k < 3; k++)
			{
				angle_state_[dual_j][k] = state_map[k][p];
			}
			p++;
		}
	}
}


bool SeqAnalyzer::TestifyStiffness()
{		
	/* examinate stiffness on printing subgraph */
	Stiffness *ptr_stiffness = new Stiffness(ptr_subgraph_, ptr_parm_);
	int Ns = ptr_subgraph_->SizeOfFreeFace();
	VX D(Ns * 6);
	D.setZero();

	if (ptr_stiffness->CalculateD(D))
	{
		for (int k = 0; k < Ns; k++)
		{
			VX offset(3);
			VX distortion(3);
			for (int t = 0; t < 3; t++)
			{
				offset[t] = D[k * 6 + t];
				distortion[t] = D[k * 6 + t + 3];
			}

			if (offset.norm() >= Dt_tol_ || distortion.norm() >= Dr_tol_)
			{
				delete ptr_stiffness;
				return false;
			}
		}
	}
	else
	{
		delete ptr_stiffness;
		return false;
	}
	
	delete ptr_stiffness;
	return true;
}


void SeqAnalyzer::GetQueue(vector<int> &print_queue)
{
	print_queue.clear();

	int Nq = print_queue_.size();
	for (int i = 0; i < Nq; i++)
	{
		int dual_e = print_queue_[i].dual_id_;
		print_queue.push_back(ptr_dualgraph_->e_orig_id(dual_e));
	}
}
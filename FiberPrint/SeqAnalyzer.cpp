#include "SeqAnalyzer.h"


SeqAnalyzer::SeqAnalyzer()
	:gamma_(100), Dt_tol_(0.1), Dr_tol_(10 * F_PI / 180),
	Wl_(1.0), Wp_(1.0), Wa_(1.0), debug_(false), fileout_(false)
{
	ptr_frame_		= NULL;
	ptr_dualgraph_	= NULL;
	ptr_subgraph_	= NULL;
	ptr_collision_	= NULL;
	ptr_parm_		= NULL;
}


SeqAnalyzer::SeqAnalyzer(GraphCut *ptr_graphcut)
	:gamma_(100), Dt_tol_(0.1), Dr_tol_(10 * F_PI / 180),
	Wl_(1.0), Wp_(1.0), Wa_(1.0), debug_(true), fileout_(false)
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

	debug_ = true;
	fileout_ = false;

	Dt_tol_ = ptr_parm->Dt_tol_;
	Dr_tol_ = ptr_parm->Dr_tol_;
	Wl_ = ptr_parm->Wl_;
	Wp_ = ptr_parm->Wp_;
	Wa_ = ptr_parm->Wa_;
}


SeqAnalyzer::~SeqAnalyzer()
{
	delete ptr_subgraph_;
	ptr_subgraph_ = NULL;

	delete ptr_collision_;
	ptr_collision_ = NULL;

	if (ptr_dualgraph_ != NULL)
	{
		int Nd = ptr_dualgraph_->SizeOfVertList();
		for (int i = 0; i < Nd*Nd; i++)
		{
			delete colli_map_[i];
			colli_map_[i] = NULL;
		}
	}
}


bool SeqAnalyzer::SeqPrint()
{
	return true;
}


void SeqAnalyzer::UpdateStructure(WF_edge *e)
{
	int dual_upd = ptr_subgraph_->UpdateDualization(e);

	/* modify D0 */
	if (dual_upd != -1)
	{
		int Ns = ptr_subgraph_->SizeOfFreeFace();
		D0_.conservativeResize(6 * Ns);
	}
}


void SeqAnalyzer::RecoverStructure(WF_edge *e)
{
	int dual_del = ptr_subgraph_->RemoveUpdation(e);
	
	/* modify D0 */
	if (dual_del != -1)
	{
		int Ns = ptr_subgraph_->SizeOfFreeFace();
		if (dual_del != Ns)
		{
			D0_.block(6 * dual_del, 0, 6 * (Ns - dual_del), 1) =
				D0_.block(6 * (dual_del + 1), 0, 6 * (Ns - dual_del), 1);
		}
		D0_.conservativeResize(6 * Ns);
	}
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

			int id = dual_i*Nd + dual_j;
			if (colli_map_[id] == NULL)
			{
				colli_map_[id] = new vector < lld > ;
				ptr_collision_->DetectCollision(target_e, order_e, *colli_map_[id]);
			}

			for (int k = 0; k < 3; k++)
			{
				state_map[k].push_back(angle_state_[dual_j][k]);
			}
			ptr_collision_->ModifyAngle(angle_state_[dual_j], *colli_map_[id]);
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

	if (ptr_stiffness->CalculateD(D, D0_))
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


void SeqAnalyzer::Init()
{
	ptr_dualgraph_->Dualization();
	int Nd = ptr_dualgraph_->SizeOfVertList();

	D0_.resize(0);
	D0_.setZero();

	print_queue_.clear();

	angle_state_.clear();
	angle_state_.resize(Nd);

	colli_map_.resize(Nd*Nd);
	for (int i = 0; i < Nd*Nd; i++)
	{
		colli_map_[i] = NULL;
	}

	delete ptr_subgraph_;
	ptr_subgraph_ = new DualGraph(ptr_frame_);
}


void SeqAnalyzer::GetPrintOrder()
{
	print_order_.clear();

	int Nq = print_queue_.size();
	for (int i = 0; i < Nq; i++)
	{
		int dual_e = print_queue_[i].dual_id_;
		print_order_.push_back(ptr_dualgraph_->e_orig_id(dual_e));
	}
}


void SeqAnalyzer::InputPrintOrder(vector<int> &print_queue)
{
	print_order_.clear();

	int Nq = print_queue.size();
	for (int i = 0; i < Nq; i++)
	{
		print_order_.push_back(print_queue[i]);
	}
}


void SeqAnalyzer::OutputPrintOrder(vector<int> &print_queue)
{
	print_queue.clear();

	int Nq = print_order_.size();
	for (int i = 0; i < Nq; i++)
	{
		print_queue.push_back(print_order_[i]);
	}
}



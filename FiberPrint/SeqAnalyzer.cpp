#include "SeqAnalyzer.h"


SeqAnalyzer::SeqAnalyzer()
	:gamma_(100), Dt_tol_(0.1), Dr_tol_(10 * F_PI / 180),
	Wp_(1.0), Wa_(1.0), Wi_(5.0), debug_(false), fileout_(false)
{
	ptr_frame_		= NULL;
	ptr_wholegraph_	= NULL;
	ptr_dualgraph_	= NULL;
	ptr_collision_	= NULL;
	ptr_path_		= NULL;
}


SeqAnalyzer::SeqAnalyzer(
	DualGraph			*ptr_dualgraph,
	QuadricCollision	*ptr_collision,
	Stiffness			*ptr_stiffness,
	FiberPrintPARM		*ptr_parm,
	char				*ptr_path
	)
{
	ptr_frame_ = ptr_dualgraph->ptr_frame_;
	ptr_dualgraph_ = ptr_dualgraph;
	ptr_collision_ = ptr_collision;
	ptr_stiffness_ = ptr_stiffness;
	ptr_path_ = ptr_path;

	ptr_wholegraph_ = new DualGraph(ptr_frame_);

	debug_ = true;
	fileout_ = false;

	Dt_tol_ = ptr_parm->Dt_tol_;
	Dr_tol_ = ptr_parm->Dr_tol_;
	Wp_ = ptr_parm->Wp_;
	Wa_ = ptr_parm->Wa_;
	Wi_ = ptr_parm->Wi_;
}


SeqAnalyzer::~SeqAnalyzer()
{
	delete ptr_wholegraph_;
	ptr_wholegraph_ = NULL;
}


bool SeqAnalyzer::SeqPrint()
{
	return true;
}


void SeqAnalyzer::PrintOutTimer()
{
}


void SeqAnalyzer::WriteRenderPath(int min_layer, int max_layer, char *ptr_path)
{
}


void SeqAnalyzer::UpdateStructure(WF_edge *e)
{
	upd_struct_.Start();

	int dual_upd = ptr_dualgraph_->UpdateDualization(e);

	/* modify D0 */
	if (dual_upd != -1)
	{
		int Ns = ptr_dualgraph_->SizeOfFreeFace();
		D0_.conservativeResize(6 * Ns);

		/* set initiate value by neighbors */
		int orig_u = ptr_dualgraph_->v_orig_id(dual_upd);
		WF_edge *eu = ptr_frame_->GetNeighborEdge(orig_u);

		VX sum_D(6);
		sum_D.setZero();
		int sum = 0;

		while (eu != NULL)
		{
			WF_vert *v = eu->pvert_;
			int dual_v = ptr_dualgraph_->v_dual_id(v->ID());
			if (dual_v != -1 && !v->isFixed())
			{
				VX tmp_D(6);
				for (int i = 0; i < 6; i++)
				{
					tmp_D[i] = D0_[6 * dual_v + i];
				}
				sum_D += tmp_D;
				sum++;
			}
			eu = eu->pnext_;
		}

		if (sum != 0)
		{
			sum_D /= sum;
		}
		for (int i = 0; i < 6; i++)
		{
			D0_[6 * (Ns - 1) + i] = sum_D[i];
		}
	}

	upd_struct_.Stop();
}


void SeqAnalyzer::RecoverStructure(WF_edge *e)
{
	rec_struct_.Start();

	int dual_del = ptr_dualgraph_->RemoveUpdation(e);
	
	/* modify D0 */
	if (dual_del != -1)
	{
		int Ns = ptr_dualgraph_->SizeOfFreeFace();
		if (dual_del != Ns)
		{
			D0_.block(6 * dual_del, 0, 6 * (Ns - dual_del), 1) =
				D0_.block(6 * (dual_del + 1), 0, 6 * (Ns - dual_del), 1);
		}
		D0_.conservativeResize(6 * Ns);
	}

	rec_struct_.Stop();
}


void SeqAnalyzer::UpdateStateMap(int dual_i, vector<vector<lld>> &state_map)
{
	upd_map_.Start();

	WF_edge *order_e = ptr_frame_->GetEdge(ptr_wholegraph_->e_orig_id(dual_i));
	int Nd = ptr_wholegraph_->SizeOfVertList();
	for (int dual_j = 0; dual_j< Nd; dual_j++)
	{
		int orig_j = ptr_wholegraph_->e_orig_id(dual_j);
		if (dual_i != dual_j && !ptr_dualgraph_->isExistingEdge(orig_j))
		{
			WF_edge *target_e = ptr_frame_->GetEdge(orig_j);

			upd_map_collision_.Start();
			vector<lld> tmp(3);
			ptr_collision_->DetectCollision(target_e, order_e, tmp);
			upd_map_collision_.Stop();

			for (int k = 0; k < 3; k++)
			{
				state_map[k].push_back(angle_state_[dual_j][k]);
			}
			ptr_collision_->ModifyAngle(angle_state_[dual_j], tmp);
		}
	}

	upd_map_.Stop();
}


void SeqAnalyzer::RecoverStateMap(int dual_i, vector<vector<lld>> &state_map)
{
	rec_map_.Start();

	int Nd = ptr_wholegraph_->SizeOfVertList();
	int p = 0;
	for (int dual_j = 0; dual_j < Nd; dual_j++)
	{
		int orig_j = ptr_wholegraph_->e_orig_id(dual_j);
		if (dual_i != dual_j && !ptr_dualgraph_->isExistingEdge(orig_j))
		{
			for (int k = 0; k < 3; k++)
			{
				angle_state_[dual_j][k] = state_map[k][p];
			}
			p++;
		}
	}

	rec_map_.Stop();
}


bool SeqAnalyzer::TestifyStiffness()
{
	test_stiff_.Start();

	/* examinate stiffness on printing subgraph */
	ptr_stiffness_->Init();

	int Ns = ptr_dualgraph_->SizeOfFreeFace();
	VX D(Ns * 6);
	D.setZero();

	test_stiff_cal_.Start();
	bool bSuccess = ptr_stiffness_->CalculateD(D, D0_);
	test_stiff_cal_.Stop();

	if (bSuccess)
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
				bSuccess = false;
				break;
			}
		}
	}

	D0_ = D;

	test_stiff_.Stop();
	return bSuccess;
}


void SeqAnalyzer::Init()
{
	ptr_wholegraph_->Dualization();
	int Nd = ptr_wholegraph_->SizeOfVertList();

	D0_.resize(0);
	D0_.setZero();

	print_queue_.clear();

	angle_state_.clear();
	angle_state_.resize(Nd);

	ptr_dualgraph_->Init();
}


void SeqAnalyzer::GetPrintOrder()
{
	print_order_.clear();

	int Nq = print_queue_.size();
	for (int i = 0; i < Nq; i++)
	{
		int dual_e = print_queue_[i].dual_id_;
		print_order_.push_back(ptr_wholegraph_->e_orig_id(dual_e));
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



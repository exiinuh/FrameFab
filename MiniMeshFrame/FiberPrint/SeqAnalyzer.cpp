#include "SeqAnalyzer.h"


SeqAnalyzer::SeqAnalyzer()
			:alpha_(1.0), beta_(10000), gamma_(100), start_edge_(0), orientation_(SEQUENCE)
{
	queue_ = new vector<int>;
}


SeqAnalyzer::SeqAnalyzer(GraphCut *ptr_graphcut)
			:alpha_(1.0), beta_(10000), gamma_(100), start_edge_(0), orientation_(SEQUENCE)
{
	ptr_graphcut_ = ptr_graphcut;
	queue_ = NULL;
}


SeqAnalyzer::~SeqAnalyzer()
{
	//delete ptr_collision_;
	//ptr_collision_ = NULL;

	//delete queue_;
	//queue_ = NULL;
}


void SeqAnalyzer::LayerPrint()
{
	WireFrame *ptr_frame = ptr_graphcut_->ptr_frame_;
	DualGraph *ptr_dualgraph = ptr_graphcut_->ptr_dualgraph_;

	ptr_collision_ = new Collision(ptr_frame, ptr_dualgraph);
	ptr_collision_->DetectFrame();

	vector<WF_vert*> verts = *(ptr_frame->GetVertList());
	vector<WF_edge*> edges = *(ptr_frame->GetEdgeList());
	int N = ptr_frame->SizeOfVertList();
	int M = ptr_frame->SizeOfEdgeList();
	int Nd = ptr_dualgraph->SizeOfVertList();

	tsp_x_.resize(Nd * Nd);
	tsp_x_.setZero();

	MX L(Nd, Nd);
	MX D(Nd, Nd);
	L.setZero();
	D.setConstant(gamma_);

	vector<vector<Range*>> *range_list = ptr_collision_->GetRangeList();
	vector<vector<int>>	   *range_state = ptr_collision_->GetRangeState();         
	// -1- vertical pi/2; 0 all angle; 1 some angle; 2  no angle 
	
	for (int i = 0; i < Nd; i++)
	{
		int orig_i = ptr_dualgraph->e_orig_id(i);
		int u1 = edges[orig_i]->ppair_->pvert_->ID();
		int v1 = edges[orig_i]->pvert_->ID();

		for (int j = 0; j < Nd; j++)
		{
			int orig_j = ptr_dualgraph->e_orig_id(j);
			int u2 = edges[orig_j]->ppair_->pvert_->ID();
			int v2 = edges[orig_j]->pvert_->ID();

			if (u1 == u2 || u1 == v2 || v1 == u2 || v1 == v2)
			{
				D(i, j) = alpha_;
				tsp_x_(i*Nd + j) = 1;
			}

			double angle = 0;
			Range *range = (*range_list)[i][j];
			switch ((*range_state)[i][j])
			{
			case 0:
				L(i, j) = alpha_;
				break;

			case 1:
				if (range->right_begin != -1 && range->right_end != -1)
				{
					angle += range->right_end - range->right_begin;
				}
				if (range->left_begin != -1 && range->left_end != -1)
				{
					angle += range->left_end - range->left_begin;
				}
				
				L(i, j) = beta_ * angle / 20.0 + alpha_;
				break;

			case 2:
				L(i, j) = beta_ + alpha_;
				break;

			default:
				break;
			}
		}
	}

	MX cost(Nd, Nd);
	for (int i = 0; i < Nd; i++)
	{
		for (int j = 0; j < Nd; j++)
		{
			cost(i, j) = L(i, j) * D(i, j);
		}
	}

	//Statistics s_cost("TSP_Cost", cost);
	//s_cost.GenerateMatrixFile();

	TSPSolver TSP_solver = TSPSolver(cost);
	TSP_solver.Solve(tsp_x_, 1);

	//Statistics s_x("TSP_Res", x);
	//s_x.GenerateVectorFile();
	
	//queue_->clear();
	GenerateQueue();
}


void SeqAnalyzer::SetStartEdge(int id)
{
	start_edge_ = id;
	GenerateQueue();
}


void SeqAnalyzer::GenerateQueue()
{
	DualGraph *ptr_dualgraph = ptr_graphcut_->ptr_dualgraph_;
	int Nd = ptr_dualgraph->SizeOfVertList();
	int h = 0;
	delete queue_;
	queue_ = new vector<int>;
	queue_->push_back(ptr_dualgraph->e_dual_id(start_edge_));
	while (h < Nd)
	{
		int i = (*queue_)[h];
		for (int j = 0; j < Nd; j++)
		{
			if (tsp_x_(i*Nd + j))
			{
				queue_->push_back(j);
				break;
			}
		}
		h++;
	}
}


void SeqAnalyzer::ChangeOrientation()
{
	if (orientation_ == SEQUENCE)
	{
		orientation_ = REVERSE;
	}
	else
	{
		orientation_ = SEQUENCE;
	}
}


Orientation SeqAnalyzer::GetOrientation()
{
	return orientation_;
}
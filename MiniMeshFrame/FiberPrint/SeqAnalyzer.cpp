#include "SeqAnalyzer.h"


SeqAnalyzer::SeqAnalyzer()
{
}


SeqAnalyzer::SeqAnalyzer(GraphCut *ptr_graphcut)
			:alpha_(1.0), beta_(10000), gamma_(100)
{
	ptr_graphcut_ = ptr_graphcut;
}


SeqAnalyzer::~SeqAnalyzer()
{
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
	int Nd = M / 2;
	MX L(Nd, Nd);
	MX D(Nd, Nd);
	VX x(Nd*Nd);
	L.setZero();
	D.setConstant(gamma_);
	x.setZero();

	vector<vector<Range*>> range_list = ptr_collision_->GetRangeList(); //           
	vector<vector<int>>	range_state = ptr_collision_->GetRangeState();         // -1- vertical pi/2; 0 all angle; 1 some angle; 2  no angle 
	for (int i = 0; i < Nd; i++)
	{
		int u1 = ptr_dualgraph->u(i);
		int v1 = ptr_dualgraph->v(i);
		for (int j = 0; j < Nd; j++)
		{
			int u2 = ptr_dualgraph->u(j);
			int v2 = ptr_dualgraph->v(j);
			if (u1 == u2 || u1 == v2 || v1 == u2 || v1 == v2)
			{
				D(i, j) = alpha_;
				x(i*Nd + j) = 1;
			}

			double angle = 0;
			Range *range = range_list[i][j];
			switch (range_state[i][j])
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
				L(i, j) = beta_ * angle / 20 + alpha_;
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
			cost(i, j) = L(i, j)*D(i, j);
		}
	}

	TSPSolver *TSP_solver = new TSPSolver(&cost);
	TSP_solver->Solve(x, 0);
}
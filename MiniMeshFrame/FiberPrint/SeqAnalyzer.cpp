#include "SeqAnalyzer.h"


SeqAnalyzer::SeqAnalyzer()
:alpha_(1.0), beta_(10000), gamma_(100)
{
	layer_queue_ = new vector<QueueInfo>;
}


SeqAnalyzer::SeqAnalyzer(GraphCut *ptr_graphcut)
:alpha_(1.0), beta_(10000), gamma_(100)
{
	ptr_graphcut_ = ptr_graphcut;
	layer_queue_ = new vector<QueueInfo>;
}


SeqAnalyzer::SeqAnalyzer(GraphCut *ptr_graphcut, FiberPrintPARM *ptr_parm)
{

	ptr_graphcut_ = ptr_graphcut;

	layer_queue_ = new vector<QueueInfo>;

	alpha_ = ptr_parm->alpha_;
	beta_ = ptr_parm->beta_;
	gamma_ = ptr_parm->gamma_;
}


SeqAnalyzer::~SeqAnalyzer()
{
	delete ptr_subgraph_; 
	ptr_subgraph_ = NULL;
}


bool SeqAnalyzer::LayerPrint()
{
	DualGraph *ptr_dualgraph = ptr_graphcut_->ptr_dualgraph_;
	int Nd = ptr_dualgraph->SizeOfVertList();

	WireFrame *ptr_frame = ptr_graphcut_->ptr_frame_;
	int N = ptr_frame->SizeOfVertList();

	/* the highest vertex and the lowest vertex */
	double maxz = -1e20;
	double minz = 1e20;
	for (int i = 0; i < Nd; i++)
	{
		double h = ptr_dualgraph->Height(i);
		if (h > maxz)
		{
			maxz = h;
		}
		if (h < minz)
		{
			minz = h;
		}
	}

	/* maxz - minz */
	height_differ_ = maxz - minz;

	/* detect collision */
	ptr_collision_ = new Collision(ptr_dualgraph);
	ptr_collision_->DetectFrame();

	/* split layers */
	vector<int>	label = *(ptr_graphcut_->GetLabel());
	int max_layer = 0;
	for (int i = 0; i < Nd; i++)
	{
		int orig_i = ptr_dualgraph->e_orig_id(i);
		if (label[orig_i] > max_layer)
		{
			max_layer = label[orig_i];
		}
	}

	max_layer++;
	layers_.resize(max_layer);
	for (int i = 0; i < Nd; i++)
	{
		int orig_i = ptr_dualgraph->e_orig_id(i);
		layers_[label[orig_i]].push_back(i);
	}

	/* printing */
	ptr_subgraph_ = new DualGraph(ptr_frame);
	for (int l = 0; l < max_layer; l++)
	{
		/* number of dual verts in current layer */
		int Nl = layers_[l].size();
		int h = layer_queue_->size();
		int t = h + Nl;

		/* set start edge */
		bool success = false;
		for (int st_e = 0; st_e < Nl; st_e++)
		{
			int dual_e = layers_[l][st_e];
			int orig_e = ptr_dualgraph->e_orig_id(dual_e);
			WF_edge *e = ptr_frame->GetEdge(orig_e);
			int u = e->ppair_->pvert_->ID();
			int v = e->pvert_->ID();
			
			/* Trial Strategy, only for edges connected to already printed structure for pillar */
			if (e->isPillar() || ptr_subgraph_->isExistingVert(u) || ptr_subgraph_->isExistingVert(v))
			{
				QueueInfo start_edge = QueueInfo{ l, st_e, layers_[l][st_e] };
				layer_queue_->push_back(start_edge);

				if (GenerateSeq(l, h, t))
				{
					success = true;
					break;
				}
			}
		}

		if (!success)
		{
			return false;
		}
	}
	

	/* generate bulks according to printing sequence */
	//base_.clear();

	//anglelist_= AngleList(layer_queue_);

	//WF_edge* temp_edge;
	//for (int i = 0; i < layer_queue_->size();i++)
	//{
	//	ExtruderCone temp_extru;
	//	WF_edge*  temp_edge = ptr_graphcut_->ptr_frame_->GetEdge(ptr_dualgraph->e_orig_id( (*layer_queue_)[i].dual_id_));
	//	temp_extru.Rotation(anglelist_[i], temp_edge->pvert_->Position(), temp_edge->ppair_->pvert_->Position());
	//	extrulist_.push_back(temp_extru);
	//}

	//return true;
}


bool SeqAnalyzer::GenerateSeq(int l, int h, int t)
{
	DualGraph *ptr_dualgraph = ptr_graphcut_->ptr_dualgraph_;
	WireFrame *ptr_frame = ptr_subgraph_->ptr_frame_;
	int Nl = layers_[l].size();

	int i = (*layer_queue_)[h].layer_id_;
	int dual_i = (*layer_queue_)[h].dual_id_;
	int orig_i = ptr_dualgraph->e_orig_id(dual_i);
	double height_i = ptr_dualgraph->Height(dual_i);

	double	min_cost = 1e20;
	int		cost_id = -1;
	double	min_D = 1e20;
	int		D_id = -1;

	double	P;							// adjacency weight
	double	L;							// collision weight
	double	S;							// stiffness weight

	map<double, int> choice;

	/* update printed subgraph */
	ptr_subgraph_->UpdateDualization(ptr_frame->GetEdge(orig_i));

	/* exit */
	if (h == t - 1)
	{
		return true;
	}

	/* next edge in current layer */
	for (int j = 0; j < Nl; j++)
	{
		int dual_j = layers_[l][j];
		int orig_j = ptr_dualgraph->e_orig_id(dual_j);
		double height_j = ptr_dualgraph->Height(dual_j);
		if (!ptr_subgraph_->isExistingEdge(orig_j))
		{
			/* adjacency weight */
			if (ptr_frame->isPillar(orig_j))
			{
				/* fixed points should be in higher priority */
				P = 1.2;
			}
			else
			if (ptr_dualgraph->isAdjacent(dual_i, dual_j))
			{
				/* they are adjancent */
				P = 1.5 - log(1 + (height_i - height_j) / height_differ_ / 2);
			}
			else
			{
				/* they are not adjancent */
				P = gamma_ * (1.5 - log(1 + (height_i - height_j) / height_differ_ / 2));
			}

			/* collision weight */
			double angle = 0;
			Range *range = ptr_collision_->GetRange(dual_i, dual_j);
			int	state = ptr_collision_->GetRangeState(dual_i, dual_j);
			switch (state)
			{
			case 0:
				/* they will not collide anyway */
				L = 0;
				break;

			case 1:
				/* they will collide sometimes */
				/* sum up limited range */
				//if (range->right_begin != -1 && range->right_end != -1)
				//{
				//	angle += range->right_end - range->right_begin;
				//}
				//if (range->left_begin != -1 && range->left_end != -1)
				//{
				//	angle += range->left_end - range->left_begin;
				//}
				//angle = 0;
				//L = beta_ * angle / 20.0 + alpha_;
				L = angle / 2 * F_PI;
				break;

			case 2:
				/* they will collide anyway */
				// L = beta_ + alpha_;
				L = 1;
				break;

			default:
				break;
			}

			/* stiffness weight */
			/* insert a trail edge */
			ptr_subgraph_->UpdateDualization(ptr_frame->GetEdge(orig_j));

			/* examinate stiffness on printing subgraph */
			Stiffness *ptr_stiffness = new Stiffness(ptr_subgraph_);
			int Ns = ptr_subgraph_->SizeOfFreeFace();
			VX D(Ns);
			D.setZero();
			if (ptr_stiffness->CalculateD(D))
			{
				S = -1e20;
				for (int k = 0; k < Ns; k++)
				{
					if (D[k] > S)
					{
						S = D[k];
					}
				}
				if (S < min_D)
				{
					min_D = S;
					D_id = j;
				}
				S *= 1e5;
			}
			else
			{
				S = 1e20;
			}

			/* remove the trail edge */
			ptr_subgraph_->RemoveUpdation(ptr_frame->GetEdge(orig_j));

			delete ptr_stiffness;
			ptr_stiffness = NULL;


			/* cost weight */
			//double cost = L * P * S;
			double cost = wl_ * L + wp_ * P;
			if (S > tol)
			{
				continue;
			}
			choice.insert(make_pair(cost, j));
		}
	}

	map<double, int>::iterator it;
	for (it = choice.begin(); it != choice.end(); it++)
	{
		QueueInfo next_edge = QueueInfo{ l, it->second, layers_[l][it->second] };
		layer_queue_->push_back(next_edge);

		if (GenerateSeq(l, h + 1, t))
		{
			return true;
		}
	}

	ptr_subgraph_->RemoveUpdation(ptr_frame->GetEdge(orig_i));
	layer_queue_->pop_back();
	return false;
}


void SeqAnalyzer::Print(Set *a)
{
	cout <<a->min<<" "<< a->max<<" " << endl;
}

void  SeqAnalyzer::Print(vector<Set*> *a)
{
	for (int i = 0; i<a->size(); i++)
	{

		Print((*a)[i]);
	}
}

void SeqAnalyzer::Debug()
{
}
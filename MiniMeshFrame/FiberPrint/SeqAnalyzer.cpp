#include "SeqAnalyzer.h"


SeqAnalyzer::SeqAnalyzer()
:alpha_(1.0), beta_(10000), gamma_(100), start_edge_(0)//, orientation_(SEQUENCE)
{
	layer_queue_ = new vector<QueueInfo>;
}


SeqAnalyzer::SeqAnalyzer(GraphCut *ptr_graphcut)
:alpha_(1.0), beta_(10000), gamma_(100), start_edge_(0)//, orientation_(SEQUENCE)
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

	start_edge_ = 0;
	//orientation_ = SEQUENCE;
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
	DualGraph *ptr_dualgraph = ptr_graphcut_->ptr_dualgraph_;
	int Nd = ptr_dualgraph->SizeOfVertList();

	WireFrame *ptr_frame = ptr_graphcut_->ptr_frame_;

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
	double delta_max = maxz - minz;

	/* detect collision */
	ptr_collision_ = new Collision(ptr_dualgraph);
	ptr_collision_->DetectFrame();
	vector<vector<Range*>> *range_list = ptr_collision_->GetRangeList();
	vector<vector<int>>	   *range_state = ptr_collision_->GetRangeState();

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

	/* stiffness on printing subgraph */
	DualGraph *ptr_subgraph = new DualGraph(ptr_frame);
	Stiffness *ptr_stiffness = new Stiffness(ptr_subgraph);

	for (int l = 0; l < max_layer; l++)
	{	
		/* number of dual verts in current layer */
		int Nl = layers_[l].size();					

		/* the lowest edge as starting edge */
		double min_height = 1e20;
		int start_id = 0;
		for (int i = 0; i < Nl; i++)
		{
			int dual_i = layers_[l][i];
			double height_i = ptr_dualgraph->Height(dual_i);

			if (min_height > height_i)
			{
				min_height = height_i;
				start_id = i;
			}
		}


		vector<bool> visited(Nl);
		fill(visited.begin(), visited.end(), false);

		/* h record the overall already printed dual verts */
		int h = layer_queue_->size();
		int t = h + Nl;

		QueueInfo start_edge = QueueInfo{ l, start_id, layers_[l][start_id] };
		layer_queue_->push_back(start_edge);

		while (h < t)
		{
			int i = (*layer_queue_)[h].layer_id_;
			int dual_i = (*layer_queue_)[h].dual_id_;
			int orig_i = ptr_dualgraph->e_orig_id(dual_i);
			double height_i = ptr_dualgraph->Height(dual_i);	// central point height of this original edge

			double	min_cost = 1e20;
			int		cost_id = -1;
			double	min_D = 1e20;
			int		D_id = -1;

			double	P;							// adjacency weight
			double	L;							// collision weight
			double	S;							// stiffness weight

			/* visited flag */
			visited[i] = true;

			/* update printed subgraph */
			ptr_subgraph->UpdateDualization(ptr_frame->GetEdge(orig_i));

			/* Trial Strategy			  */
			/* next edge in current layer */
			for (int j = 0; j < Nl; j++)
			{
				if (!visited[j])
				{
					/* adjacency weight */
					int dual_j = layers_[l][j];
					int orig_j = ptr_dualgraph->e_orig_id(dual_j);
					double height_j = ptr_dualgraph->Height(dual_j);

					if (ptr_frame->isPillar(orig_j))
					{
						/* fixed points should be in higher priority */
						P = 1.2;
					}
					else
					if (ptr_dualgraph->isAdjacent(dual_i, dual_j))
					{
						/* they are adjancent */
						P = 1.5 - log(1 + (height_i - height_j) / delta_max / 2);
					}
					else
					{	
						/* they are not adjancent */
						P = gamma_ * (1.5 - log(1 + (height_i - height_j) / delta_max / 2));
					}

					/* collision weight */
					double angle = 0;
					Range *range = (*range_list)[dual_i][dual_j];
					switch ((*range_state)[dual_i][dual_j])
					{
					case 0:
						/* they will not collide anyway */
						L = alpha_;
						break;

					case 1:
						/* they will collide sometimes */
						/* sum up limited range */
						if (range->right_begin != -1 && range->right_end != -1)
						{
							angle += range->right_end - range->right_begin;
						}
						if (range->left_begin != -1 && range->left_end != -1)
						{
							angle += range->left_end - range->left_begin;
						}

						L = beta_ * angle / 20.0 + alpha_;
						break;

					case 2:
						/* they will collide anyway */
						L = beta_ + alpha_;
						break;

					default:
						break;
					}
					

					/* stiffness weight */
					/* insert a trail edge */
					ptr_subgraph->UpdateDualization(ptr_frame->GetEdge(orig_j));
					ptr_stiffness->Init();

					/* examinate stiffness */
					int Ns = ptr_subgraph->SizeOfFreeFace();
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

					/* cost weight */
					double cost = L * P * S;
					if (cost < min_cost)
					{
						min_cost = cost;
						cost_id = j;
					}

					/* remove the trail edge */
					ptr_subgraph->RemoveUpdation(ptr_frame->GetEdge(orig_j));
				}
			}

			if (cost_id != -1)
			{
				QueueInfo next_edge = QueueInfo{ l, cost_id, layers_[l][cost_id] };
				layer_queue_->push_back(next_edge);
			}
			h++;
		}
	}
	

	/* generate bulks according to printing sequence */
	base_.clear();

	anglelist_= AngleList(layer_queue_);

	WF_edge* temp_edge;
	for (int i = 0; i < layer_queue_->size();i++)
	{
		ExtruderCone temp_extru;
		WF_edge*  temp_edge = ptr_graphcut_->ptr_frame_->GetEdge(ptr_dualgraph->e_orig_id( (*layer_queue_)[i].dual_id_));
		temp_extru.Rotation(anglelist_[i], temp_edge->pvert_->Position(), temp_edge->ppair_->pvert_->Position());
		extrulist_.push_back(temp_extru);
	}
}


vector<double>  SeqAnalyzer::AngleList(vector<QueueInfo>*layer_queue)
{
	vector<double> temp;
	for (int i = 0; i < layer_queue->size(); i++)
	{
		temp.push_back(AngleValue(i, layer_queue));
		base_.push_back(ptr_graphcut_->ptr_frame_->GetEdge(ptr_graphcut_->ptr_dualgraph_->e_orig_id((*layer_queue)[i].dual_id_))->pvert_->Position());
	}
	return temp;
}

double SeqAnalyzer::AngleValue(int id, vector<QueueInfo>*layer_queue)
{
	
	int dual_id = ((*layer_queue)[id]).dual_id_;
	vector<Range*> range_list = (*(ptr_collision_->GetRangeList()))[dual_id];

	double value = 0;

	for (int i = 0; i < id-1; i++)
	{
		if (abs(value) < Divergence(range_list[(*layer_queue)[i].dual_id_]))
			 value = Divergence(range_list[(*layer_queue)[i].dual_id_]);
	}

	return pi / 2 + value;
}

double SeqAnalyzer::Divergence(Range *r)
{
	double value, value_1;
	Set* s_0 = new Set{ r->left_begin, r->left_end };
	Set* s_1 = new Set{ r->right_begin, r->right_end };
	value = MAX;
	value_1 = value;

	if (s_0->max != -1)
	{
		if (pi/2 >= s_0->min && pi/2<= s_0->max)
			return 0;

		if (abs(s_0->min - pi / 2) < abs(s_0->max - pi / 2))
			value = s_0->min - pi / 2;
		else
			value = s_0->max - pi / 2;
	}

	if (s_1->max != -1)
	{
		if (pi / 2 >= s_1->min && pi / 2 <= s_1->max)
			return 0;

		if (abs(s_1->min - pi / 2) < abs(s_1->max - pi / 2))
			value_1 = s_1->min - pi / 2;
		else
			value_1 = s_1->max - pi / 2;
	}

	if (abs(value) < abs(value_1))
		return value;
	else
		return value_1;
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
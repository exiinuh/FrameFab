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


	base_.clear();
	

	DualGraph *ptr_dualgraph = ptr_graphcut_->ptr_dualgraph_;
	int Nd = ptr_dualgraph->SizeOfVertList();

	WireFrame *ptr_frame = ptr_graphcut_->ptr_frame_;
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
	double delta_max = maxz - minz;

	ptr_collision_ = new Collision(ptr_dualgraph);
	ptr_collision_->DetectFrame();
	vector<vector<Range*>> *range_list = ptr_collision_->GetRangeList();
	vector<vector<int>>	   *range_state = ptr_collision_->GetRangeState();

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

	for (int l = 0; l < max_layer; l++)
	{
		int Nl = layers_[l].size();						// number of dual verts in current layer

		MX L(Nl, Nl);
		MX D(Nl, Nl);
		L.setZero();
		D.setZero();

		// -1- vertical pi/2; 0 all angle; 1 some angle; 2  no angle 
		double min_height = 1e20;
		int min_id = 0;
		for (int i = 0; i < Nl; i++)
		{
			int dual_i = layers_[l][i];
			double height_i = ptr_dualgraph->Height(dual_i);

			if (min_height > height_i)
			{
				min_height = height_i;
				min_id = i;
			}

			for (int j = 0; j < Nl; j++)
			{
				int dual_j = layers_[l][j];
				int orig_j = ptr_dualgraph->e_orig_id(dual_j);
				double height_j = ptr_dualgraph->Height(dual_j);

				if (ptr_frame->isPillar(orig_j))
				{
					D(i, j) = 1.2;
				}
				else
				if (ptr_dualgraph->isAdjacent(dual_i, dual_j))
				{
					D(i, j) = 1.5 - log(1 + (height_i - height_j) / delta_max / 2);
				}
				else
				{
					D(i, j) = gamma_ * (1.5 - log(1 + (height_i - height_j) / delta_max / 2));
				}

				double angle = 0;
				Range *range = (*range_list)[dual_i][dual_j];
				switch ((*range_state)[dual_i][dual_j])
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

		MX cost(Nl, Nl);
		for (int i = 0; i < Nl; i++)
		{
			for (int j = 0; j < Nl; j++)
			{
				cost(i, j) = L(i, j) * D(i, j);
			}
		}


		int h = layer_queue_->size();
		int t = h + Nl;

		vector<bool> visited(Nl);
		fill(visited.begin(), visited.end(), false);

		QueueInfo start_edge = QueueInfo{ l, min_id, layers_[l][min_id] };
		layer_queue_->push_back(start_edge);

		while (h < t)
		{
			double min_cost = 1e20;
			int min_id = -1;
			int i = (*layer_queue_)[h].layer_id_;
			visited[i] = true;

			for (int j = 0; j < Nl; j++)
			{
				if (!visited[j] && cost(i, j) < min_cost)
				{
					min_cost = cost(i, j);
					min_id = j;
				}
			}

			if (min_id != -1)
			{
				QueueInfo next_edge = QueueInfo{ l, min_id, layers_[l][min_id] };
				layer_queue_->push_back(next_edge);
			}
			h++;
		}
	}
	

	anglelist_= AngleList(layer_queue_);

    for (int i = 0; i < anglelist_.size(); i++)
		cout << anglelist_[i] << endl;

	WF_edge* temp_edge;
	for (int i = 0; i < layer_queue_->size();i++)
	{
		ExtruderCone temp_extru;
		WF_edge*  temp_edge = ptr_graphcut_->ptr_frame_->GetEdge(ptr_dualgraph->e_orig_id( (*layer_queue_)[i].dual_id_));
		temp_extru.Rotation(anglelist_[i], temp_edge->pvert_->Position(), temp_edge->ppair_->pvert_->Position());
		extrulist_.push_back(temp_extru);
	}
}




void SeqAnalyzer::GenerateQueue(int l, int Nl)
{
}


void SeqAnalyzer::SetStartEdge(int id)
{
	//start_edge_ = id;
	//GenerateQueue();
}


/*
SeqAnalyzer::SeqAnalyzer()
			:alpha_(1.0), beta_(10000), gamma_(100), start_edge_(0), orientation_(SEQUENCE)
{
	layer_queue_	= new vector<int>;
	dual_queue_		= new vector<int>;
}


SeqAnalyzer::SeqAnalyzer(GraphCut *ptr_graphcut)
			:alpha_(1.0), beta_(10000), gamma_(100), start_edge_(0), orientation_(SEQUENCE)
{
	ptr_graphcut_	= ptr_graphcut;
	layer_queue_	= new vector<int>;
	dual_queue_		= new vector<int>;
}


SeqAnalyzer::SeqAnalyzer(GraphCut *ptr_graphcut, FiberPrintPARM *ptr_parm)
{
	ptr_graphcut_	= ptr_graphcut;

	layer_queue_	= new vector<int>;
	dual_queue_		= new vector<int>;

	alpha_	= ptr_parm->alpha_;
	beta_	= ptr_parm->beta_;
	gamma_	= ptr_parm->gamma_;

	orientation_ = SEQUENCE;
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
	double delta_max = maxz - minz;

	ptr_collision_ = new Collision(ptr_frame, ptr_dualgraph);
	ptr_collision_->DetectFrame();
	vector<vector<Range*>> *range_list = ptr_collision_->GetRangeList();
	vector<vector<int>>	   *range_state = ptr_collision_->GetRangeState();

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
	tsp_x_.resize(max_layer);
	for (int i = 0; i < Nd; i++)
	{
		int orig_i = ptr_dualgraph->e_orig_id(i);
		layers_[label[orig_i]].push_back(i);
	}

	for (int l = 0; l < max_layer; l++)
	{
		int Nl = layers_[l].size();						// number of dual verts in current layer

		tsp_x_[l].resize(Nl * Nl);
		tsp_x_[l].setZero();

		MX L(Nl, Nl);
		MX D(Nl, Nl);
		L.setZero();
		D.setZero();

		// -1- vertical pi/2; 0 all angle; 1 some angle; 2  no angle 
		double min_height = 1e20;
		int min_id = 0;
		for (int i = 0; i < Nl; i++)
		{
			int dual_i = layers_[l][i];
			double height_i = ptr_dualgraph->Height(dual_i);

			if (min_height > height_i)
			{
				min_height = height_i;
				min_id = i;
			}

			for (int j = 0; j < Nl; j++)
			{
				int dual_j = layers_[l][j];
				int orig_j = ptr_dualgraph->e_orig_id(dual_j);
				double height_j = ptr_dualgraph->Height(dual_j);

				if (ptr_dualgraph->isAdjacent(dual_i, dual_j))
				{
					//D(i, j) = alpha_;
					//double tmp = 1 + (height_i - height_j) * 1.0 / delta_max / 2;
					//double tmp1 = log(tmp);
					if (ptr_frame->isPillar(orig_j))
					{
						D(i, j) = 1.2;
					}
					else
					{
						D(i, j) = 1.5 - log(1 + (height_i - height_j) / delta_max / 2);
					}
					tsp_x_[l](i*Nl + j) = 1;
				}
				else
				{
					//D(i, j) = gamma_;
					//double tmp = 1 + (height_i - height_j) * 1.0 / delta_max / 2;
					//double tmp1 = log(tmp);
					if (ptr_frame->isPillar(orig_j))
					{
						D(i, j) = 1.2;
					}
					else
					{
						D(i, j) = gamma_ * (1.5 - log(1 + (height_i - height_j) / delta_max / 2));
					}
				}

				double angle = 0;
				Range *range = (*range_list)[dual_i][dual_j];
				switch ((*range_state)[dual_i][dual_j])
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

		MX cost(Nl, Nl);
		for (int i = 0; i < Nl; i++)
		{
			for (int j = 0; j < Nl; j++)
			{
				cost(i, j) = L(i, j) * D(i, j);
			}
		}

		//Statistics s_cost("TSP_Cost", cost);
		//s_cost.GenerateMatrixFile();

		TSPSolver TSP_solver = TSPSolver(cost);
		TSP_solver.Solve(tsp_x_[l], 0);

		//Statistics s_x("TSP_Res", x);
		//s_x.GenerateVectorFile();

		//queue_->clear();
		printf("%d %d", l, min_id);
		start_edge_ = min_id;
		GenerateQueue(l, Nl);
	}
}


void SeqAnalyzer::GenerateQueue(int l, int Nl)
{
	int h = layer_queue_->size();
	int t = layer_queue_->size() + Nl;

	//delete queue_;
	//queue_ = new vector<int>;
	layer_queue_->push_back(start_edge_);

	while (h < t)
	{
		int i = (*layer_queue_)[h];
		int dual_i = layers_[l][i];
		dual_queue_->push_back(dual_i); 

		if (h < t - 1)
		{
			for (int j = 0; j < Nl; j++)
			{
				int dual_j = layers_[l][j];
				if (tsp_x_[l](i*Nl + j))
				{
					layer_queue_->push_back(j);
					break;
				}
			}
		}
		h++;
	}
}


void SeqAnalyzer::SetStartEdge(int id)
{
	//start_edge_ = id;
	//GenerateQueue();
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
*/

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
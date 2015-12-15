#include "SeqAnalyzer.h"


SeqAnalyzer::SeqAnalyzer()
:gamma_(100), D_tol_(0.1), Wl_(10.0), Wp_(1.0)
{
	layer_queue_ = new vector<QueueInfo>;
}


SeqAnalyzer::SeqAnalyzer(GraphCut *ptr_graphcut)
:gamma_(100), D_tol_(0.1), Wl_(10.0), Wp_(1.0)
{
	ptr_graphcut_ = ptr_graphcut;
	layer_queue_ = new vector<QueueInfo>;
}


SeqAnalyzer::SeqAnalyzer(GraphCut *ptr_graphcut, FiberPrintPARM *ptr_parm)
{
	ptr_graphcut_ = ptr_graphcut;

	layer_queue_ = new vector<QueueInfo>;

	gamma_	= ptr_parm->gamma_;
	D_tol_	= ptr_parm->D_tol_;
	Wl_		= ptr_parm->Wl_;
	Wp_		= ptr_parm->Wp_;
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


	Wave.clear();
	for (int i = 0; i < Nd; i++)
	{
		Wave.push_back(0);
	}

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

	vector<vector<double>> range_list = ptr_collision_->R2_range_;
	vector<vector<int>> range_state = ptr_collision_->R2_state_;

	/* split layers */
	/* label stores layer index of each dual node */
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
		/* 
		* Nl: number of dual verts in current layer
		* h : head for printing queue of the layer
		* t : tail for printing queue of the layer
		*/
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
			
			/* Trial Strategy, only compute for edges connected to 
			*  already printed structure or pillar edges
			*/
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
	
	/* Feasible printing orientation */
	WF_edge* temp_edge;
	angle_list_ = AngleList(layer_queue_);

	for (int i = 0; i < layer_queue_->size();i++)
	{
		ExtruderCone temp_extruder;

		/* original edge id */
		WF_edge*  temp_edge = ptr_graphcut_->ptr_frame_->GetEdge(ptr_dualgraph->e_orig_id( (*layer_queue_)[i].dual_id_));
		
		temp_extruder.Rotation(angle_list_[i], temp_edge->pvert_->Position(), temp_edge->ppair_->pvert_->Position());

		extruder_list_.push_back(temp_extruder);
	}
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

	double	P;							// adjacency weight
	double	L;							// collision weight

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
			/* collision weight */
			double angle = ptr_collision_->R2_range_[dual_i][dual_j];

			Range *range = ptr_collision_->GetRange(dual_i, dual_j);
			int	state = ptr_collision_->GetRangeState(dual_i, dual_j);
			switch (state)
			{
			case 0:
				/* they will not collide anyway */
				L = 0;
				break;

			case 1:
				L = 1 - (angle / 2 * F_PI);
				break;

			case 2:
				/*
				* they will collide anyway
				* prohibited printing edge
				*/
				continue;
				break;

			default:
				break;
			}


			/* stiffness */
			/* insert a trail edge */
			ptr_subgraph_->UpdateDualization(ptr_frame->GetEdge(orig_j));

			/* examinate stiffness on printing subgraph */
			Stiffness *ptr_stiffness = new Stiffness(ptr_subgraph_);
			int Ns = ptr_subgraph_->SizeOfFreeFace();
			VX D(Ns);
			D.setZero();

			double max_D = -1e20;
			
			printf("------------\n");
			printf("Layers %d, head index %d\n", l, h);
			printf("Trial Deformation calculation edge %d\n", dual_j);
			if (ptr_stiffness->CalculateD(D))
			{
				for (int k = 0; k < Ns; k++)
				{
					if (D[k] > max_D)
					{
						max_D = D[k];
					}
				}
			}
			else
			{
				max_D = 1e20;
			}

			/* remove the trail edge */
			ptr_subgraph_->RemoveUpdation(ptr_frame->GetEdge(orig_j));

			delete ptr_stiffness;
			ptr_stiffness = NULL;

			/* examination failed */
			//if (max_D >= D_tol_)
			//{
			//	continue;
			//}


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


			/* cost weight */
			double cost = Wl_ * L + Wp_ * P;
			choice.insert(make_pair(cost, j));
		}
	}

	/* ranked by weight */
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

vector<GeoV3> SeqAnalyzer::AngleList(vector<QueueInfo> *layer_queue)
{
	support_ = 0;
	vector<GeoV3> temp;

	cout << "------------------------ " << endl;
	cout << "Feasible Angle Computing:" << endl;

	for (int i = 0; i < layer_queue->size(); i++)
	{
		cout << "edge " << i << " angle in processing" << endl;
		temp.push_back(AngleDec(i));
	}
	return temp;
}

GeoV3 SeqAnalyzer::AngleDec(int id)
{
	/* sampling number of orientation vector in 2*pi angle range */
	int divide = 72;
	vector<GeoV3> normal;
	
	WF_edge*  temp_edge = ptr_graphcut_->ptr_frame_->GetEdge(ptr_graphcut_->ptr_dualgraph_->e_orig_id((*layer_queue_)[id].dual_id_));
	if (temp_edge->isPillar())
	{
		support_ += 1;
		return GeoV3(0, 0, 1);
	}

	point start, end;
	start = temp_edge->pvert_->Position();
	end   = temp_edge->ppair_->pvert_->Position();

	GeoV3 u, v;

	GeoV3 t = end - start;
	t.normalize();
	GeoV3 z(0, 0, 1);

	if (Geometry::cross(t, z).norm() < eps)
	{
		/* vertical case */
		u = Geometry::Vector3d(1, 0, 0);
		v = Geometry::Vector3d(0, 1, 0);
	}
	else
	{
		/* perpendicular to printing edge */
		u = Geometry::cross(t, z);
		u.normalize();
		v = Geometry::cross(u, t);
		v.normalize();
	}

	/* normal contains all sampling vectors */
	for (int i = 0; i < divide; i++)
	{
		normal.push_back(u*cos(2 * F_PI / divide*i) + v*sin(2 * F_PI / divide*i));
	}

	/* i traverse all the printed edges until current state id */
	for (int i = 0; i < id; i++)
	{
		vector<GeoV3>::iterator it;
		/* influence from id to i 
		* i.e. the collision cost of existence of edge i
		* when printing edge id
		*/
		RAngle TestR = ptr_collision_->R2_Angle[(*layer_queue_)[id].dual_id_][(*layer_queue_)[i].dual_id_];

		if (ptr_collision_->R2_state_[(*layer_queue_)[id].dual_id_][(*layer_queue_)[i].dual_id_] == 0)
			continue;

		for (it = normal.begin(); it != normal.end();)
		{
			if (IsColVec(TestR.u, TestR.v, *it ))
			{
				it = normal.erase(it);
			}
			else
			{
				++it;
			}
		}
	}

	if (normal.size() == 0)
	{
		cout << " Infeasible orientation!" << endl;
		Wave[id] = 0;
		return GeoV3(0, 0, 1);
		return GeoV3(0, 0, 1);
	}

	if (normal.size() == 72)
	{
		Wave[id] = 2 * F_PI;
		return normal[0];
	}

	ResolveAngle resolve;
	resolve = ResolveAngle(normal);

	Wave[id] = resolve.wave;
	return resolve.dec;
}

bool  SeqAnalyzer::IsColVec(GeoV3 start, GeoV3 end, GeoV3 target)
{
	if (Geometry::angle(target, start) <= extruder_.Angle())
		return true;

	if (Geometry::angle(target, end) <= extruder_.Angle())
		return true;

	if (abs(Geometry::angle(target, start) + Geometry::angle(target, end) - Geometry::angle(target, end)) < eps)
		return true;

	return false;
}

void SeqAnalyzer::Print(Set *a)
{
	cout << a->min << " " << a->max << " " << endl;
}

void SeqAnalyzer::Print(vector<Set*> *a)
{
	for (int i = 0; i < a->size(); i++)
	{

		Print((*a)[i]);
	}
}

void SeqAnalyzer::Debug()
{
}
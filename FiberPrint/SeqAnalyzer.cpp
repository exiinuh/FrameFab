#include "SeqAnalyzer.h"


SeqAnalyzer::SeqAnalyzer()
<<<<<<< HEAD
	:gamma_(100), Dt_tol_(0.1), Dr_tol_(10 * F_PI / 180),
	Wl_(1.0), Wp_(1.0), Wi_(1.0), extru_(false), debug_(false), fileout_(false)
=======
	:gamma_(100), Dt_tol_(0.1), Dr_tol_(10 * F_PI / 180), 
	Wl_(10.0), Wp_(100.0), Wi_(1.0)
>>>>>>> 2c719846f6006b0658c93a4bf28bf6ac0236a416
{
}


SeqAnalyzer::SeqAnalyzer(GraphCut *ptr_graphcut)
<<<<<<< HEAD
	:gamma_(100), Dt_tol_(0.1), Dr_tol_(10 * F_PI / 180),
	Wl_(1.0), Wp_(1.0), Wi_(1.0), extru_(false), debug_(true), fileout_(false)
{
	ptr_frame_ = ptr_graphcut->ptr_frame_;
	ptr_dualgraph_ = ptr_graphcut->ptr_dualgraph_;

	ptr_subgraph_ = new DualGraph(ptr_frame_);
	ptr_collision_ = new QuadricCollision(ptr_frame_);
=======
	:gamma_(100), Dt_tol_(0.1), Dr_tol_(10 * F_PI / 180), 
	Wl_(10.0), Wp_(100.0), Wi_(1.0)
{
	ptr_graphcut_ = ptr_graphcut;
	extru_ = false;
>>>>>>> 2c719846f6006b0658c93a4bf28bf6ac0236a416
}


SeqAnalyzer::SeqAnalyzer(GraphCut *ptr_graphcut, FiberPrintPARM *ptr_parm, char *ptr_path)
{
<<<<<<< HEAD
	ptr_frame_ = ptr_graphcut->ptr_frame_;
	ptr_dualgraph_ = ptr_graphcut->ptr_dualgraph_;
	ptr_parm_ = ptr_parm;
	ptr_path_ = ptr_path;
=======
	ptr_graphcut_ = ptr_graphcut;
>>>>>>> 2c719846f6006b0658c93a4bf28bf6ac0236a416

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
}


bool SeqAnalyzer::SeqPrint()
{
<<<<<<< HEAD
	return true;
=======
	DualGraph *ptr_dualgraph = ptr_graphcut_->ptr_dualgraph_;
	ptr_dualgraph->Dualization();
	int Nd = ptr_dualgraph->SizeOfVertList();

	WireFrame *ptr_frame = ptr_graphcut_->ptr_frame_;
	int N = ptr_frame->SizeOfVertList();
	int M = ptr_frame->SizeOfEdgeList();

	/* split layers */
	/* label stores layer index of each dual node */
	int max_layer = 0;
	for (int i = 0; i < Nd; i++)
	{
		int orig_i = ptr_dualgraph->e_orig_id(i);
		int label = ptr_frame->GetEdge(orig_i)->Layer();
		if (label > max_layer)
		{
			max_layer = label;
		}
	}

	max_layer++;
	ptr_frame->SetMaxLayer(max_layer); 
	layers_.resize(max_layer);
	for (int i = 0; i < Nd; i++)
	{
		int orig_i = ptr_dualgraph->e_orig_id(i);
		int label = ptr_frame->GetEdge(orig_i)->Layer();
		layers_[label].push_back(i);
	}

	/* printing */
	ptr_subgraph_ = new DualGraph(ptr_frame);

	/* set pillars as starting edges */
	/* ranked by x */
	int base_size = layers_[0].size();
	multimap<double, int>base_queue;
	multimap<double, int>::iterator it;
	for (int i = 0; i < base_size; i++)
	{
		int dual_e = layers_[0][i];
		WF_edge *e = ptr_frame->GetEdge(ptr_dualgraph->e_orig_id(dual_e));
		if (e->isPillar())
		{
			point center = e->CenterPos();
			base_queue.insert(make_pair(center.x(), i));
			ptr_subgraph_->UpdateDualization(e);
		}
	}

	for (it = base_queue.begin(); it != base_queue.end(); it++)
	{
		QueueInfo base_edge = QueueInfo{ 0, it->second, layers_[0][it->second] };
		layer_queue_.push_back(base_edge);
	}

	printf("Size of base queue: %d\n", base_queue.size());
	for (int l = 0; l < max_layer; l++)
	{
		printf("Size of layer %d is %d\n", l, layers_[l].size());
	}

	getchar();

	/* init range */
	angle_state_.resize(Nd);
	fill(angle_state_.begin(), angle_state_.end(), 0);
	ptr_collision_ = new Collision(ptr_frame);
	for (int dual_i = 0; dual_i < Nd; dual_i++)
	{
		int orig_i = ptr_dualgraph->e_orig_id(dual_i);
		if (!ptr_subgraph_->isExistingEdge(orig_i))
		{
			WF_edge *e = ptr_frame->GetEdge(orig_i);
			ptr_collision_->DetectCollision(e, ptr_subgraph_);
			angle_state_[dual_i] = ptr_collision_->Angle();
		}
	}

	/* print starting from the first layer */
	for (int l = 0; l < max_layer; l++)
	{
		/* 
		* Nl: number of dual verts in current layer
		* h : head for printing queue of the layer
		* t : tail for printing queue of the layer
		*/
		int Nl = layers_[l].size();
		int h = layer_queue_.size();
		int t;
		if (l == 0)
		{
			t = Nl;
		}
		else
		{
			t = h + Nl;
		}

		if (debug_)
		{
			printf("layer %d is in processing, intial head index %d, tail index %d\n", l, h, t);
		}

		if (h == t)
		{
			continue;
		}


		/* max_z_ and min_z_ in current layer */
		min_z_ = 1e20;
		max_z_ = -min_z_;
		for (int i = 0; i < Nl; i++)
		{
			int orig_i = ptr_dualgraph->e_orig_id(layers_[l][i]);
			point u = ptr_frame->GetEdge(orig_i)->pvert_->Position();
			point v = ptr_frame->GetEdge(orig_i)->ppair_->pvert_->Position();
			min_z_ = min(min_z_, (double)min(u.z(), v.z()));
			max_z_ = max(max_z_, (double)max(u.z(), v.z()));
		}


		/* set start edge for searching of current layer */
		multimap<double, int> choice;

		bool success = false;
		for (int st_e = 0; st_e < Nl; st_e++)
		{
			double cost = GenerateCost(l, st_e);
			if (cost != -1.0)
			{
				choice.insert(make_pair(cost, st_e));
			}
		}

		for (it = choice.begin(); it != choice.end(); it++)
		{
			int dual_i = layers_[l][it->second];
			WF_edge *ei = ptr_frame->GetEdge(ptr_dualgraph->e_orig_id(dual_i));
			QueueInfo start_edge = QueueInfo{ l, it->second, dual_i };
			layer_queue_.push_back(start_edge);

			vector<lld> tmp_angle;
			for (int dual_j = 0; dual_j < Nd; dual_j++)
			{
				int orig_j = ptr_dualgraph->e_orig_id(dual_j);
				if (dual_i != dual_j && !ptr_subgraph_->isExistingEdge(orig_j))
				{
					WF_edge *ej = ptr_frame->GetEdge(orig_j);
					ptr_collision_->DetectCollision(ej, ei);
					tmp_angle.push_back(angle_state_[dual_j]);
					angle_state_[dual_j] |= ptr_collision_->Angle();
				}
			}

			if (GenerateSeq(l, h, t))
			{
				success = true;
				break;
			}

			int j = 0;
			for (int dual_j = 0; dual_j < Nd; dual_j++)
			{
				int orig_j = ptr_dualgraph->e_orig_id(dual_j);
				if (dual_i != dual_j && !ptr_subgraph_->isExistingEdge(orig_j))
				{
					angle_state_[dual_j] = tmp_angle[j];
					j++;
				}
			}
		}

		if (!success)
		{
			if (debug_)
			{
				printf("all possible start edge at layer %d has been tried but no feasible sequence is obtained.\n", l);
			}
			getchar();
			return false;
		}
	}

	if (debug_)
	{
		WriteLayerQueue();
	}
	
	/* detect extruder angles */
	//DetectAngle();

	/* for rendering */
>>>>>>> 2c719846f6006b0658c93a4bf28bf6ac0236a416
}


void SeqAnalyzer::UpdateStateMap(int dual_i, vector<vector<lld>> &state_map)
{
<<<<<<< HEAD
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
=======
	if (debug_)
	{
		printf("searching edge #%d in layer %d, head %d, (tail %d)\n", 
			layer_queue_[h].layer_id_, l, h, t);
	}

	DualGraph *ptr_dualgraph = ptr_graphcut_->ptr_dualgraph_;
	WireFrame *ptr_frame = ptr_subgraph_->ptr_frame_;
	int Nl = layers_[l].size();
	int M = ptr_frame->SizeOfEdgeList();
	int Nd = ptr_dualgraph->SizeOfVertList();

	int i = layer_queue_[h].layer_id_;
	int dual_i = layer_queue_[h].dual_id_;
	int orig_i = ptr_dualgraph->e_orig_id(dual_i);

	/* update printed subgraph */
	ptr_subgraph_->UpdateDualization(ptr_frame->GetEdge(orig_i));

	/* exit */
	if (h == t - 1)
	{
		if (debug_)
		{
			printf("searching at layer %d finishes.\n", l);
			printf("-------------------------------\n");
		}

		return true;
	}

	/* next choice */
	multimap<double, int> choice;
	multimap<double, int>::iterator it;

	/* next edge in current layer */
	for (int j = 0; j < Nl; j++)
	{
		/* cost weight */
		double cost = GenerateCost(l, j);
		if (cost != -1)
		{
			choice.insert(pair<double, int>(cost, j));
		}
	}

	///* for rendering */
	//char layer[10];
	//char head[10];
	//sprintf(layer, "%d", l);
	//sprintf(head, "%d", h);

	//string path = path_;
	//string file = path + "/PathRender_" + layer + "_" + head + ".txt";
	//FILE *fp = fopen(file.c_str(), "w+");

	//double min_cost = choice.begin()->first;
	//double max_cost;
	//int M = ptr_frame->SizeOfEdgeList();
	//vector<double> tmp_cost(M);
	//fill(tmp_cost.begin(), tmp_cost.end(), -1);
	//for (it = choice.begin(); it != choice.end(); it++)
	//{
	//	max_cost = it->first; 
	//	int dual_j = layers_[l][it->second]; 
	//	int orig_j = ptr_dualgraph->e_orig_id(dual_j);
	//	tmp_cost[orig_j] = tmp_cost[ptr_frame->GetEdge(orig_j)->ppair_->ID()] = it->first;
	//}

	//for (int j = 0; j < M; j++)
	//{
	//	double r;
	//	double g;
	//	double b;
	//	WF_edge *e = ptr_frame->GetEdge(j);
	//	if (j < e->ppair_->ID())
	//	{
	//		if (ptr_subgraph_->isExistingEdge(j))
	//		{
	//			r = 0.5;
	//			g = 0.5;
	//			b = 0.5;
	//		}
	//		else
	//		if (e->Layer() != l)
	//		{
	//			r = 1.0;
	//			g = 1.0;
	//			b = 1.0;
	//		}
	//		else
	//		if (tmp_cost[j] == -1)
	//		{
	//			r = 0;
	//			g = 0;
	//			b = 0;
	//		}
	//		else
	//		{
	//			double cost_j;
	//			if (max_cost == min_cost)
	//			{
	//				cost_j = 1.0;
	//			}
	//			else
	//			{
	//				cost_j = (tmp_cost[j] - min_cost) / (max_cost - min_cost);
	//			}
	//			
	//			r = 1.0;
	//			g = cost_j;
	//			b = 0.0;
	//		}

	//		point u = e->pvert_->RenderPos(); 
	//		point v = e->ppair_->pvert_->RenderPos();
	//		fprintf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
	//			u.x(), u.y(), u.z(), v.x(), v.y(), v.z(), r, g, b);
	//	}
	//}
	//fclose(fp);

	/* ranked by weight */
	for (it = choice.begin(); it != choice.end(); it++)
	{
		int dual_j = layers_[l][it->second];
		int orig_j = ptr_dualgraph->e_orig_id(dual_j);
		WF_edge *ej = ptr_frame->GetEdge(orig_j);
		QueueInfo next_edge = QueueInfo{ l, it->second, dual_j };
		layer_queue_.push_back(next_edge);

		vector<lld> tmp_angle;
		for (int dual_k = 0; dual_k < Nd; dual_k++)
		{
			int orig_k = ptr_dualgraph->e_orig_id(dual_k);
			if (dual_j != dual_k && !ptr_subgraph_->isExistingEdge(orig_k))
			{
				WF_edge *ek = ptr_frame->GetEdge(orig_k);
				ptr_collision_->DetectCollision(ek, ej);
				tmp_angle.push_back(angle_state_[dual_k]);
				angle_state_[dual_k] |= ptr_collision_->Angle();
			}
		}

		if (debug_)
		{
			printf("choose edge #%d in layer %d with cost %lf\n", it->second, l, it->first);
			printf("entering next searching state.\n");
		}

		if (GenerateSeq(l, h + 1, t))
		{
			return true;
		}

		int k = 0;
		for (int dual_k = 0; dual_k < Nd; dual_k++)
		{
			int orig_k = ptr_dualgraph->e_orig_id(dual_k);
			if (dual_j != dual_k && !ptr_subgraph_->isExistingEdge(orig_k))
			{
				angle_state_[dual_k] = tmp_angle[k];
				k++;
			}
		}
	}

	ptr_subgraph_->RemoveUpdation(ptr_frame->GetEdge(orig_i));
	layer_queue_.pop_back();

	if (debug_)
	{
		printf("searching at layer %d, head %d, (tail %d) ended.\n", l, h, t);
	}

	return false;
}


double SeqAnalyzer::GenerateCost(int l, int j)
{		
	DualGraph *ptr_dualgraph = ptr_graphcut_->ptr_dualgraph_;
	WireFrame *ptr_frame = ptr_subgraph_->ptr_frame_;
	int M = ptr_frame->SizeOfEdgeList();

	int dual_j = layers_[l][j];
	int orig_j = ptr_dualgraph->e_orig_id(dual_j);
	WF_edge *ej = ptr_frame->GetEdge(orig_j);

	if (!ptr_subgraph_->isExistingEdge(orig_j))
	{
		double	P;							// adjacency weight
		double	L;							// collision weight
		double  I;							// influence weight

		if (debug_)
		{
			printf("Attempting edge #%d, layer %d\n", j, l);
		}

		/* collision weight */
		L = (double)ptr_collision_->ColFreeAngle(angle_state_[dual_j]) /
			ptr_collision_->Divide();

		if (0 == L)
		{
			return -1;
		}


		/* adjacency weight */
		int u = ptr_frame->GetEndu(orig_j);
		int v = ptr_frame->GetEndv(orig_j);
		point pos_u = ptr_frame->GetPosition(u);
		point pos_v = ptr_frame->GetPosition(v);
		bool exist_u = ptr_subgraph_->isExistingVert(u);
		bool exist_v = ptr_subgraph_->isExistingVert(v);
		double z = (min(pos_u.z(), pos_v.z()) - min_z_) / (max_z_ - min_z_);

		if (exist_u && exist_v)
		{
			/* edge j share two ends with printed structure */
			if (debug_)
			{
				printf("it shares two ends with printed structure\n");
			}
			P = z;
		}
		else
		if (exist_u || exist_v)
		{
			/* edge j share one end with printed structure */
			if (debug_)
			{
				printf("it shares only one ends with printed structure\n");
			}

			double ang;
			if (exist_u)
			{
				ang = Geometry::angle(point(0, 0, 1), pos_v - pos_u);
			}
			else
			{
				ang = Geometry::angle(point(0, 0, 1), pos_u - pos_v);
			}
			P = z * exp(ang);
		}
		else
		{
			if (debug_)
			{
				printf("it floats, skip\n");
			}
			return -1;
		}


		/* stiffness */
		/* insert a trail edge */
		ptr_subgraph_->UpdateDualization(ptr_frame->GetEdge(orig_j));

		/* examinate stiffness on printing subgraph */
		Stiffness *ptr_stiffness = new Stiffness(ptr_subgraph_);
		int Ns = ptr_subgraph_->SizeOfFreeFace();
		VX D(Ns);
		D.setZero();

		bool stiff_success = true;
		if (ptr_stiffness->CalculateD(D))
		{
			for (int k = 0; k < Ns; k++)
			{
				VX offset(3);
				VX distortion(3);
				for (int h = 0; h < 3; h++)
				{
					offset[h] = D[j * 6 + h];
					distortion[h] = D[j * 6 + h + 3];
				}

				if (offset.norm() >= Dt_tol_ || distortion.norm() >= Dr_tol_)
				{
					stiff_success = false;
					break;
				}
			}
		}
		else
		{
			stiff_success = false;
		}

		/* remove the trail edge */
		ptr_subgraph_->RemoveUpdation(ptr_frame->GetEdge(orig_j));

		delete ptr_stiffness;
		ptr_stiffness = NULL;

		/* examination failed */
		if (!stiff_success)
		{
			if (debug_)
			{
				cout << "Stiffness Verification Fail." << endl;
			}
			return -1;
		}

		if (debug_)
		{
			cout << "Stiffness Verification Pass" << endl;
		}

		/* influence weight */
		int sum_angle = 0; 
		int Nd = ptr_dualgraph->SizeOfVertList();
		int remaining = Nd - ptr_subgraph_->SizeOfVertList();
		for (int dual_k = 0; dual_k < Nd; dual_k++)
		{
			int orig_k = ptr_dualgraph->e_orig_id(dual_k);
			if (dual_j != dual_k && !ptr_subgraph_->isExistingEdge(orig_k))
			{
				ptr_collision_->DetectCollision(ptr_frame->GetEdge(orig_k), ej);
				lld tmp_angle = (ptr_collision_->Angle() | angle_state_[dual_k]);
				sum_angle += ptr_collision_->ColFreeAngle(tmp_angle);
>>>>>>> 2c719846f6006b0658c93a4bf28bf6ac0236a416
			}
			ptr_collision_->ModifyAngle(angle_state_[dual_j]);
		}
<<<<<<< HEAD
=======
		I = sum_angle / remaining;


		double cost = Wl_ * L + Wp_ * P + Wi_ * I;
		if (debug_)
		{
			printf("cost : %f\n", cost);
		}
		return cost;
>>>>>>> 2c719846f6006b0658c93a4bf28bf6ac0236a416
	}
}


void SeqAnalyzer::RecoverStateMap(int dual_i, vector<vector<lld>> &state_map)
{
<<<<<<< HEAD
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
=======
	DualGraph *ptr_dualgraph = ptr_graphcut_->ptr_dualgraph_;
	layer_queue.clear();

	int Nq = layer_queue_.size();
	for (int i = 0; i < Nq; i++)
	{
		int dual_e = layer_queue_[i].dual_id_;
		layer_queue.push_back(ptr_dualgraph->e_orig_id(dual_e));
>>>>>>> 2c719846f6006b0658c93a4bf28bf6ac0236a416
	}
}


<<<<<<< HEAD
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
=======
void SeqAnalyzer::WriteLayerQueue()
{
	string path = path_;
	string queue_path = path + "/Queue.txt";

	FILE *fp = fopen(queue_path.c_str(), "w");

	vector<int> layer_queue;
	GetQueue(layer_queue);
	int Nq = layer_queue.size(); 
	for (int i = 0; i < Nq; i++)
	{
		fprintf(fp, "%d\n", layer_queue[i]);
	}
	fclose(fp);
}


void SeqAnalyzer::DetectAngle()
{
	//cout << "---------Angle Detection--------" << endl;
	//DualGraph *ptr_dualgraph = ptr_graphcut_->ptr_dualgraph_;
	//WireFrame *ptr_frame = ptr_graphcut_->ptr_frame_;

	//vector<GeoV3 > Normal;
	//vector<double> Wave;

	//support_ = 0;
	//for (int i = 0; i < layer_queue_.size(); i++)
	//{
	//	int orig_e = ptr_dualgraph->e_orig_id(layer_queue_[i].dual_id_);
	//	WF_edge *target = ptr_frame->GetEdge(orig_e);

	//	if (target->isPillar())
	//	{
	//		Normal.push_back(GeoV3(0, 0, 1));
	//		Wave.push_back(2 * F_PI);

	//		support_ += 1;
	//		continue;
	//	}

	//	Collision col(ptr_graphcut_->ptr_frame_, target);
	//	for (int j = 0; j < i; j++)
	//	{
	//		orig_e = ptr_dualgraph->e_orig_id(layer_queue_[j].dual_id_);
	//		col.DetectCollision(ptr_frame->GetEdge(orig_e));

	//		if (col.normal_.size() == 0)
	//		{
	//			cout << "Oops~~, What is wrong?!" << endl;
	//		}
	//	}
	//	ResolveAngle resolve(col.normal_);
	//	Normal.push_back(resolve.dec);
	//	Wave.push_back(resolve.wave);

	//}

	//wave_ = Wave;

	////Extruder
	//for (int i = 0; i < layer_queue_.size(); i++)
	//{
	//	extru_ = true;
	//	ExtruderCone temp_extruder;

	//	/* original edge id */
	//	int orig_e = ptr_dualgraph->e_orig_id(layer_queue_[i].dual_id_);
	//	WF_edge *temp_edge = ptr_frame->GetEdge(orig_e);

	//	temp_extruder.Rotation(Normal[i], temp_edge->pvert_->Position(), 
	//		temp_edge->ppair_->pvert_->Position());
	//	extruder_list_.push_back(temp_extruder);
	//}

	//cout << "---------Angle Detection done--------" << endl;
>>>>>>> 2c719846f6006b0658c93a4bf28bf6ac0236a416
}
#include "FFAnalyzer.h"


FFAnalyzer::FFAnalyzer()
	:gamma_(100), Dt_tol_(0.1), Dr_tol_(10 * F_PI / 180),
	Wl_(1.0), Wp_(1.0), Wi_(1.0)
{
	extru_ = false;
	debug_ = false;
	fileout_ = false;
}


FFAnalyzer::FFAnalyzer(GraphCut *ptr_graphcut)
	:gamma_(100), Dt_tol_(0.1), Dr_tol_(10 * F_PI / 180),
	Wl_(1.0), Wp_(1.0), Wi_(1.0)
{
	ptr_frame_ = ptr_graphcut->ptr_frame_;
	ptr_dualgraph_ = ptr_graphcut->ptr_dualgraph_;

	extru_ = false;
	debug_ = false;
	fileout_ = false;
}


FFAnalyzer::FFAnalyzer(GraphCut *ptr_graphcut, FiberPrintPARM *ptr_parm, char *path)
{
	ptr_frame_ = ptr_graphcut->ptr_frame_;
	ptr_dualgraph_ = ptr_graphcut->ptr_dualgraph_;
	ptr_subgraph_ = new DualGraph(ptr_frame_);
	ptr_collision_ = new QuadricCollision(ptr_frame_);

	gamma_ = ptr_parm->gamma_;
	Dt_tol_ = ptr_parm->Dt_tol_;
	Dr_tol_ = ptr_parm->Dr_tol_;
	Wl_ = ptr_parm->Wl_;
	Wp_ = ptr_parm->Wp_;
	Wi_ = 1.0;
	debug_ = 0;

	path_ = path;
	extru_ = false;
	debug_ = true;
	fileout_ = false;
}


FFAnalyzer::~FFAnalyzer()
{
}


bool FFAnalyzer::LayerPrint()
{
	ptr_dualgraph_->Dualization();
	int Nd = ptr_dualgraph_->SizeOfVertList();
	int N = ptr_frame_->SizeOfVertList();
	int M = ptr_frame_->SizeOfEdgeList();

	/* split layers */
	/* label stores layer index of each dual node */
	int max_layer = 0;
	for (int i = 0; i < Nd; i++)
	{
		int orig_i = ptr_dualgraph_->e_orig_id(i);
		int label = ptr_frame_->GetEdge(orig_i)->Layer();
		if (label > max_layer)
		{
			max_layer = label;
		}
	}

	max_layer++;
	ptr_frame_->SetMaxLayer(max_layer);
	layers_.resize(max_layer);
	for (int i = 0; i < Nd; i++)
	{
		int orig_i = ptr_dualgraph_->e_orig_id(i);
		int label = ptr_frame_->GetEdge(orig_i)->Layer();
		layers_[label].push_back(i);
	}

	/* printing */
	/* set pillars as starting edges */
	/* ranked by x */
	int base_size = layers_[0].size();
	multimap<double, int>base_queue;
	multimap<double, int>::iterator it;
	for (int i = 0; i < base_size; i++)
	{
		int dual_e = layers_[0][i];
		WF_edge *e = ptr_frame_->GetEdge(ptr_dualgraph_->e_orig_id(dual_e));
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

	angle_state_.resize(Nd);
	for (int dual_i = 0; dual_i < Nd; dual_i++)
	{
		int orig_i = ptr_dualgraph_->e_orig_id(dual_i);
		if (!ptr_subgraph_->isExistingEdge(orig_i))
		{
			WF_edge *e = ptr_frame_->GetEdge(orig_i);
			ptr_collision_->DetectCollision(e, ptr_subgraph_);
			ptr_collision_->AngleState(angle_state_[dual_i]);
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
			int orig_i = ptr_dualgraph_->e_orig_id(layers_[l][i]);
			point u = ptr_frame_->GetEdge(orig_i)->pvert_->Position();
			point v = ptr_frame_->GetEdge(orig_i)->ppair_->pvert_->Position();
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
			WF_edge *ei = ptr_frame_->GetEdge(ptr_dualgraph_->e_orig_id(dual_i));
			QueueInfo start_edge = QueueInfo{ l, it->second, dual_i };
			layer_queue_.push_back(start_edge);

			vector<vector<lld>> tmp_angle(3);
			for (int dual_j = 0; dual_j < Nd; dual_j++)
			{
				int orig_j = ptr_dualgraph_->e_orig_id(dual_j);
				if (dual_i != dual_j && !ptr_subgraph_->isExistingEdge(orig_j))
				{
					WF_edge *ej = ptr_frame_->GetEdge(orig_j);
					ptr_collision_->DetectCollision(ej, ei);
					for (int k = 0; k < 3; k++)
					{
						tmp_angle[k].push_back(angle_state_[dual_j][k]);
					}
					ptr_collision_->ModifyAngle(angle_state_[dual_j]);
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
				int orig_j = ptr_dualgraph_->e_orig_id(dual_j);
				if (dual_i != dual_j && !ptr_subgraph_->isExistingEdge(orig_j))
				{
					for (int k = 0; k < 3; k++)
					{
						angle_state_[dual_j][k] = tmp_angle[k][j];
					}
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
			return false;
		}
	}

	if (fileout_)
	{
		WriteLayerQueue();
		WritePathRender();
	}

	/* detect extruder angles */
	//DetectBulk();

	return true;
}


bool FFAnalyzer::GenerateSeq(int l, int h, int t)
{
	if (debug_)
	{
		printf("searching edge #%d in layer %d, head %d, (tail %d)\n",
			layer_queue_[h].layer_id_, l, h, t);
	}

	int Nl = layers_[l].size();
	int M = ptr_frame_->SizeOfEdgeList();
	int Nd = ptr_dualgraph_->SizeOfVertList();

	int i = layer_queue_[h].layer_id_;
	int dual_i = layer_queue_[h].dual_id_;
	int orig_i = ptr_dualgraph_->e_orig_id(dual_i);

	/* update printed subgraph */
	ptr_subgraph_->UpdateDualization(ptr_frame_->GetEdge(orig_i));

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

	/* ranked by weight */
	for (it = choice.begin(); it != choice.end(); it++)
	{
		int dual_j = layers_[l][it->second];
		int orig_j = ptr_dualgraph_->e_orig_id(dual_j);
		WF_edge *ej = ptr_frame_->GetEdge(orig_j);
		QueueInfo next_edge = QueueInfo{ l, it->second, dual_j };
		layer_queue_.push_back(next_edge);

		vector<vector<lld>> tmp_angle(3);
		for (int dual_k = 0; dual_k < Nd; dual_k++)
		{
			int orig_k = ptr_dualgraph_->e_orig_id(dual_k);
			if (dual_j != dual_k && !ptr_subgraph_->isExistingEdge(orig_k))
			{
				WF_edge *ek = ptr_frame_->GetEdge(orig_k);
				ptr_collision_->DetectCollision(ek, ej);
				for (int k = 0; k < 3; k++)
				{
					tmp_angle[k].push_back(angle_state_[dual_k][k]);
				}
				ptr_collision_->ModifyAngle(angle_state_[dual_k]);
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
			int orig_k = ptr_dualgraph_->e_orig_id(dual_k);
			if (dual_j != dual_k && !ptr_subgraph_->isExistingEdge(orig_k))
			{
				for (int j = 0; j < 3; j++)
				{
					angle_state_[dual_k][j] = tmp_angle[j][k];
				}
				k++;
			}
		}
	}

	ptr_subgraph_->RemoveUpdation(ptr_frame_->GetEdge(orig_i));
	layer_queue_.pop_back();

	if (debug_)
	{
		printf("searching at layer %d, head %d, (tail %d) ended.\n", l, h, t);
	}

	return false;
}


double FFAnalyzer::GenerateCost(int l, int j)
{
	int M = ptr_frame_->SizeOfEdgeList();
	int dual_j = layers_[l][j];
	int orig_j = ptr_dualgraph_->e_orig_id(dual_j);
	WF_edge *ej = ptr_frame_->GetEdge(orig_j);

	if (!ptr_subgraph_->isExistingEdge(orig_j))
	{
		double	P;							// adjacency weight
		double	L;							// collision weight
		double  I;							// influence weight

		if (debug_)
		{
			printf("Attempting edge #%d, layer %d, head %d\n",
				j, l, layer_queue_.size());
		}

		/* collision weight */
		L = (double)ptr_collision_->ColFreeAngle(angle_state_[dual_j]) /
			ptr_collision_->Divide();

		if (0 == L)
		{
			return -1;
		}


		/* adjacency weight */
		int u = ptr_frame_->GetEndu(orig_j);
		int v = ptr_frame_->GetEndv(orig_j);
		point pos_u = ptr_frame_->GetPosition(u);
		point pos_v = ptr_frame_->GetPosition(v);
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
		ptr_subgraph_->UpdateDualization(ej);

		/* examinate stiffness on printing subgraph */
		Stiffness *ptr_stiffness = new Stiffness(ptr_subgraph_);
		int Ns = ptr_subgraph_->SizeOfFreeFace();
		VX D(Ns * 6);
		D.setZero();

		bool stiff_success = true;
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
		ptr_subgraph_->RemoveUpdation(ej);

		delete ptr_stiffness;
		ptr_stiffness = NULL;

		/* examination failed */
		if (!stiff_success)
		{
			printf("Stiffness examination failed.\n");
			return -1;
		}


		///* influence weight */
		//int sum_angle = 0;
		//int Nd = ptr_dualgraph_->SizeOfVertList();
		//int remaining = Nd - ptr_subgraph_->SizeOfVertList();
		//for (int dual_k = 0; dual_k < Nd; dual_k++)
		//{
		//	int orig_k = ptr_dualgraph_->e_orig_id(dual_k);
		//	if (dual_j != dual_k && !ptr_subgraph_->isExistingEdge(orig_k))
		//	{
		//		ptr_collision_->DetectCollision(ptr_frame_->GetEdge(orig_k), ej);
		//		lld tmp_angle = (ptr_collision_->Angle() | angle_state_[dual_k]);
		//		sum_angle += ptr_collision_->ColFreeAngle(tmp_angle);
		//	}
		//}
		//I = (double)sum_angle / remaining / ptr_collision_->Divide();


		double cost = Wl_*L + Wp_*P + Wi_*I;
		//double cost = Wl_ * L + Wp_ * P + Wi_ * I;
		if (debug_)
		{
			printf("L: %lf, P: %lf, I:%lf\ncost: %f\n", L, P, I, cost);
		}
		return cost;
	}

	return -1;
}




void FFAnalyzer::DetectBulk()
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
}


//void FFAnalyzer::BruteForcePrint()
//{
//	ptr_dualgraph_->Dualization();
//
//	int M = ptr_frame_->SizeOfEdgeList();
//	int Nd = ptr_dualgraph_->SizeOfVertList();
//	int h = 0;
//
//	for (int i = 0; i < Nd; i++)
//	{
//		WF_edge *e = ptr_frame_->GetEdge(ptr_dualgraph_->e_orig_id(i));
//		if (e->isPillar())
//		{
//			//ptr_subgraph_->UpdateDualization(e); 
//			layer_queue_.push_back(QueueInfo{ 0, 0, i });
//			h++;
//		}
//	}
//
//	angle_state_.resize(Nd);
//	for (int i = 0; i < Nd; i++)
//	{
//		int orig_i = ptr_dualgraph_->e_orig_id(i);
//		if (!ptr_subgraph_->isExistingEdge(orig_i)
//			&& !ptr_frame_->GetEdge(orig_i)->isPillar())
//		{
//			WF_edge *e = ptr_frame_->GetEdge(orig_i);
//			ptr_collision_->DetectCollision(e, ptr_subgraph_);
//			angle_state_[i] = ptr_collision_->Angle();
//		}
//	}
//
//	GenerateSeq(h, Nd);
//}


bool FFAnalyzer::GenerateSeq(int h, int t)
{
	if (h == t)
	{
		return true;
	}

	printf("head :%d\n", h);

	/* stiffness */
	/* examinate stiffness on printing subgraph */
	Stiffness *ptr_stiffness = new Stiffness(ptr_subgraph_);
	int Ns = ptr_subgraph_->SizeOfFreeFace();
	VX D(Ns * 6);
	D.setZero();

	bool stiff_success = true;
	if (ptr_stiffness->CalculateD(D))
	{
		for (int k = 0; k < Ns; k++)
		{
			VX offset(3);
			VX distortion(3);
			for (int l = 0; l < 3; l++)
			{
				offset[l] = D[k * 6 + l];
				distortion[l] = D[k * 6 + l + 3];
			}

			if (offset.norm() >= Dt_tol_ || distortion.norm() >= Dr_tol_)
			{
				stiff_success = false;
				getchar();
				break;
			}
		}
	}
	else
	{
		stiff_success = false;
	}

	delete ptr_stiffness;
	ptr_stiffness = NULL;

	if (!stiff_success)
	{
		return false;
	}


	int Nd = ptr_dualgraph_->SizeOfVertList();
	for (int i = 0; i < Nd; i++)
	{
		int orig_i = ptr_dualgraph_->e_orig_id(i);
		WF_edge *ei = ptr_frame_->GetEdge(orig_i);
		if (!ei->isPillar() && !ptr_subgraph_->isExistingEdge(orig_i))
		{
			if (ptr_collision_->ColFreeAngle(angle_state_[i]) == 0)
			{
				return false;
			}

			if (h != ptr_frame_->SizeOfPillar())
			{
				int u = ei->pvert_->ID();
				int v = ei->ppair_->pvert_->ID();
				if (!ptr_subgraph_->isExistingVert(u)
					&& !ptr_subgraph_->isExistingVert(v))
				{
					continue;
				}
			}

			vector<vector<lld>> tmp_angle(3);
			for (int j = 0; j < Nd; j++)
			{
				int orig_j = ptr_dualgraph_->e_orig_id(j);
				if (i != j &&
					!ptr_subgraph_->isExistingEdge(orig_j) && !ei->isPillar())
				{
					WF_edge *ej = ptr_frame_->GetEdge(orig_j);
					ptr_collision_->DetectCollision(ej, ei);
					for (int k = 0; k < 3; k++)
					{
						tmp_angle[k].push_back(angle_state_[j][k]);
					}
					ptr_collision_->ModifyAngle(angle_state_[j]);
				}
			}

			layer_queue_.push_back(QueueInfo{ 0, 0, i });
			ptr_subgraph_->UpdateDualization(ei);
			if (GenerateSeq(h + 1, t))
			{
				return true;
			}
			ptr_subgraph_->RemoveUpdation(ei);
			layer_queue_.pop_back();

			int k = 0;
			for (int j = 0; j < Nd; j++)
			{
				int orig_j = ptr_dualgraph_->e_orig_id(j);
				if (i != j &&
					!ptr_subgraph_->isExistingEdge(orig_j) && !ei->isPillar())
				{
					for (int p = 0; p < 3; p++)
					{
						angle_state_[j][p] = tmp_angle[p][k];
					}
					k++;
				}
			}
		}
	}

	return false;
}


void FFAnalyzer::WriteLayerQueue()
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


void FFAnalyzer::WritePathRender()
{
	///* for rendering */
	//ptr_dualgraph_->Dualization();

	//int Nd = ptr_dualgraph_->SizeOfVertList();
	//angle_state_.resize(Nd);
	//fill(angle_state_.begin(), angle_state_.end(), 0);

	//FILE *ifp = fopen("C:/Users/DELL/Desktop/Queue.txt", "r+");
	//int max_layer = -1;
	//for (int i = 0; i < Nd; i++)
	//{
	//	int orig_i;
	//	fscanf(ifp, "%d", &orig_i);

	//	int layer = ptr_frame_->GetEdge(orig_i)->Layer();
	//	if (layer > max_layer)
	//	{
	//		max_layer = layer;
	//	}
	//	QueueInfo edge = QueueInfo{ layer, 0,
	//		ptr_dualgraph_->e_dual_id(orig_i) };
	//	layer_queue_.push_back(edge);
	//}

	//fclose(ifp);

	//max_layer++;
	//layers_.resize(max_layer);
	//for (int dual_i = 0; dual_i < Nd; dual_i++)
	//{
	//	int orig_i = ptr_dualgraph_->e_orig_id(dual_i);
	//	int layer = ptr_frame_->GetEdge(orig_i)->Layer();
	//	layers_[layer].push_back(dual_i);
	//}

	//for (int i = 0; i < Nd; i++)
	//{
	//	int l = layer_queue_[i].layer_;
	//	int Nl = layers_[l].size();
	//	int dual_i = layer_queue_[i].dual_id_;
	//	int orig_i = ptr_dualgraph_->e_orig_id(dual_i);
	//	WF_edge *ei = ptr_frame_->GetEdge(orig_i);

	//	ptr_subgraph_->UpdateDualization(ei);

	//	for (int dual_j = 0; dual_j < Nd; dual_j++)
	//	{
	//		int orig_j = ptr_dualgraph_->e_orig_id(dual_j);
	//		if (dual_i != dual_j && !ptr_subgraph_->isExistingEdge(orig_j))
	//		{
	//			WF_edge *ej = ptr_frame_->GetEdge(orig_j);
	//			ptr_collision_->DetectCollision(ej, ei);
	//			angle_state_[dual_j] |= ptr_collision_->Angle();
	//		}
	//	}

	//	if (ei->isPillar())
	//	{
	//		continue;
	//	}

	//	min_z_ = 1e20;
	//	max_z_ = -min_z_;
	//	for (int j = 0; j < Nl; j++)
	//	{
	//		int orig_j = ptr_dualgraph_->e_orig_id(layers_[l][j]);
	//		int u = ptr_frame_->GetEndu(orig_j);
	//		int v = ptr_frame_->GetEndv(orig_j);
	//		point pos_u = ptr_frame_->GetPosition(u);
	//		point pos_v = ptr_frame_->GetPosition(v);
	//		min_z_ = min(min_z_, (double)min(pos_u.z(), pos_v.z()));
	//		max_z_ = max(max_z_, (double)max(pos_u.z(), pos_v.z()));
	//	}

	//	vector<double> cost(Nd);
	//	double min_cost = 1e20;
	//	double max_cost = -min_cost;
	//	for (int j = 0; j < Nl; j++)
	//	{
	//		int dual_j = layers_[l][j];
	//		cost[dual_j] = GenerateCost(l, j);
	//		if (cost[dual_j] != -1)
	//		{
	//			if (cost[dual_j] < min_cost)
	//			{
	//				min_cost = cost[dual_j];
	//			}
	//			if (cost[dual_j] > max_cost)
	//			{
	//				max_cost = cost[dual_j];
	//			}
	//		}
	//	}

	//	char id[10];
	//	sprintf(id, "%d", i);

	//	string path = path_;
	//	string file = path + "/PathRender_" + id + ".txt";
	//	FILE *fp = fopen(file.c_str(), "w+");

	//	for (int dual_j = 0; dual_j < Nd; dual_j++)
	//	{
	//		double r;
	//		double g;
	//		double b;
	//		int orig_j = ptr_dualgraph_->e_orig_id(dual_j);
	//		WF_edge *e = ptr_frame_->GetEdge(orig_j);

	//		if (ptr_subgraph_->isExistingEdge(orig_j))
	//		{
	//			r = 0.5;
	//			g = 0.5;
	//			b = 0.5;
	//		}
	//		else
	//			if (e->Layer() != l)
	//			{
	//				r = 1.0;
	//				g = 1.0;
	//				b = 1.0;
	//			}
	//			else
	//				if (cost[dual_j] == -1)
	//				{
	//					r = 0;
	//					g = 0;
	//					b = 0;
	//				}
	//				else
	//				{
	//					double cost_j;
	//					if (max_cost == -1e20)
	//					{
	//						cost_j = 0.0;
	//					}
	//					else
	//					{
	//						cost_j = (cost[dual_j] - min_cost) / (max_cost - min_cost);
	//					}

	//					r = 1.0;
	//					g = cost_j;
	//					b = 0.0;
	//				}

	//		point u = e->pvert_->RenderPos();
	//		point v = e->ppair_->pvert_->RenderPos();
	//		fprintf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
	//			u.x(), u.y(), u.z(), v.x(), v.y(), v.z(), r, g, b);
	//		//printf("%lf %lf %lf\n", r, g, b);
	//	}

	//	fclose(fp);
	//}
}

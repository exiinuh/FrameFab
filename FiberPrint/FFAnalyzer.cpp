#include "FFAnalyzer.h"


FFAnalyzer::FFAnalyzer()
{
}


FFAnalyzer::~FFAnalyzer()
{
}


bool FFAnalyzer::SeqPrint()
{
	FF_analyzer_.Start();

	Init();

	/* split layers */
	/* label stores layer index of each dual node */
	int layer_size = ptr_frame_->SizeOfLayer();
	layers_.clear();
	layers_.resize(layer_size);
	for (int i = 0; i < Nd_; i++)
	{
		int orig_i = ptr_wholegraph_->e_orig_id(i);
		int label = ptr_frame_->GetEdge(orig_i)->Layer();
		layers_[label].push_back(i);
	}

	/* printing */
	/* set pillars as starting edges */
	PrintPillars();

	/* print starting from the first layer */
	bool bSuccess = true;
	for (int l = 0; l < layer_size; l++)
	{
		/*
		* Nl: number of dual verts in current layer
		* h : head for printing queue of the layer
		* t : tail for printing queue of the layer
		*/
		int Nl = layers_[l].size();
		int h = print_queue_.size();
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
			printf(">>>layer %d is in processing, intial head index %d, tail index %d\n", l, h, t);
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
			int orig_i = ptr_wholegraph_->e_orig_id(layers_[l][i]);
			point u = ptr_frame_->GetEdge(orig_i)->pvert_->Position();
			point v = ptr_frame_->GetEdge(orig_i)->ppair_->pvert_->Position();
			min_z_ = min(min_z_, (double)min(u.z(), v.z()));
			max_z_ = max(max_z_, (double)max(u.z(), v.z()));
		}

		if (!GenerateSeq(l, h, t))
		{
			if (debug_)
			{
				printf("...all possible start edge at layer %d has been tried but no feasible sequence is obtained.\n", l);
			}
			bSuccess = false;
		}
	}

	if (fileout_)
	{
		//WriteLayerQueue();
		//WriteRenderPath();
	}

	FF_analyzer_.Stop();

	return bSuccess;
}


bool FFAnalyzer::GenerateSeq(int l, int h, int t)
{
	if (debug_)
	{
		printf("---searching edge #%d in layer %d, head %d, (tail %d)\n",
			print_queue_[h - 1]->ID() / 2, l, h, t);
	}

	/* exit */
	if (h == t)
	{
		if (debug_)
		{
			printf("***searching at layer %d finishes.\n", l);
		}

		return true;
	}

	/* last edge */
	WF_edge *ei = NULL;
	if (h != 0)
	{
		ei = print_queue_[h - 1];
	}

	/* next choice */
	multimap<double, int> choice;
	multimap<double, int>::iterator it;

	/* next edge in current layer */
	int Nl = layers_[l].size();
	for (int j = 0; j < Nl; j++)
	{
		/* cost weight */
		double cost = GenerateCost(l, j, ei);
		if (cost != -1)
		{
			choice.insert(pair<double, int>(cost, j));
		}
	}

	/* ranked by weight */
	for (it = choice.begin(); it != choice.end(); it++)
	{
		int dual_j = layers_[l][it->second];
		int orig_j = ptr_wholegraph_->e_orig_id(dual_j);
		WF_edge *ej = ptr_frame_->GetEdge(orig_j);
		print_queue_.push_back(ej);

		/* update printed subgraph */
		UpdateStructure(ej);

		/* update collision */
		vector<vector<lld>> tmp_angle(3);
		UpdateStateMap(dual_j, tmp_angle);

		if (debug_)
		{
			printf("^^^choose edge #%d in layer %d with cost %lf\n", it->second, l, it->first);
			printf("^^^entering next searching state.\n");
		}

		if (GenerateSeq(l, h + 1, t))
		{
			return true;
		}

		RecoverStateMap(dual_j, tmp_angle);
		RecoverStructure(ej);
		print_queue_.pop_back();
	}

	if (debug_)
	{
		printf("---searching at layer %d, head %d, (tail %d) ended.\n", l, h, t);
	}

	return false;
}


double FFAnalyzer::GenerateCost(int l, int j, WF_edge *ei)
{
	int dual_j = layers_[l][j];
	int orig_j = ptr_wholegraph_->e_orig_id(dual_j);
	WF_edge *ej = ptr_frame_->GetEdge(orig_j);

	if (!ptr_dualgraph_->isExistingEdge(orig_j))
	{
		double	P = 0;							// stabiliy weight
		double  A = 0;							// adjacency weight
		double	I = 0;							// influence weight

		if (debug_)
		{
			printf("###Attempting edge #%d, layer %d, head %d\n",
				orig_j / 2, l, print_queue_.size());
		}

		/* collision test */
		int free_angle = ptr_collision_->ColFreeAngle(angle_state_[dual_j]);
		if (free_angle == 0)
		{
			printf("...collision examination failed.\n");
			return -1;
		}

		/* stabiliy weight */
		WF_edge *ej = ptr_frame_->GetEdge(orig_j);
		int uj = ptr_frame_->GetEndu(orig_j);
		int vj = ptr_frame_->GetEndv(orig_j);
		bool exist_uj = ptr_dualgraph_->isExistingVert(uj);
		bool exist_vj = ptr_dualgraph_->isExistingVert(vj);
		double z = (ej->CenterPos().z() - min_z_) / (max_z_ - min_z_);

		if (exist_uj && exist_vj)
		{
			/* edge j share two ends with printed structure */
			if (debug_)
			{
				printf("^^^it shares two ends with printed structure\n");
			}
			P = z;
		}
		else
			if (exist_uj || exist_vj)
			{
				/* edge j share one end with printed structure */
				if (debug_)
				{
					printf("^^^it shares only one ends with printed structure\n");
				}

				double ang;
				point pos_uj = ptr_frame_->GetPosition(uj);
				point pos_vj = ptr_frame_->GetPosition(vj);
				if (exist_uj)
				{
					ang = Geometry::angle(point(0, 0, 1), pos_vj - pos_uj);
				}
				else
				{
					ang = Geometry::angle(point(0, 0, 1), pos_uj - pos_vj);
				}
				P = z * exp(ang);
			}
			else
			{
				if (debug_)
				{
					printf("...it floats, skip\n");
				}
				return -1;
			}


		/* adjacency weight */
		if (ei == NULL)
		{
			A = 0;
		}
		else
		{
			int ui = ei->pvert_->ID();
			int vi = ei->ppair_->pvert_->ID();
			if (ui == uj || ui == vj || vi == uj || vi == vj)
			{
				A = 0;
			}
			else
			{
				A = 1.0;
			}
		}


		/* stiffness test */
		/* insert a trail edge */
		UpdateStructure(ej);

		/* examinate stiffness on printing subgraph */
		if (!TestifyStiffness())
		{
			/* examination failed */
			printf("...Stiffness examination failed.\n");
			return -1;
		}

		/* remove the trail edge */
		RecoverStructure(ej);


		/* influence weight */
		int remaining = Nd_ - ptr_dualgraph_->SizeOfVertList();
		for (int dual_k = 0; dual_k < Nd_; dual_k++)
		{
			int orig_k = ptr_wholegraph_->e_orig_id(dual_k);
			if (dual_j != dual_k && !ptr_dualgraph_->isExistingEdge(orig_k))
			{
				vector<lld> tmp(3);
				ptr_collision_->DetectCollision(ptr_frame_->GetEdge(orig_k), ej, tmp);
				for (int o = 0; o < 3; o++)
				{
					tmp[o] |= angle_state_[dual_k][o];
				}

				double tmp_range = ptr_collision_->ColFreeAngle(tmp) * 1.0 /
					ptr_collision_->Divide();
				I += exp(-5 * tmp_range * tmp_range);
			}
		}
		I /= remaining;

		double cost = Wp_*P + Wa_*A + Wi_*I;
		if (debug_)
		{
			printf("###P: %lf, A: %lf, I: %lf\ncost: %f\n", P, A, I, cost);
		}
		return cost;
	}

	return -1;
}


void FFAnalyzer::PrintOutTimer()
{
	printf("***FFAnalyzer timer result:\n");
	FF_analyzer_.Print("FFAnalyzer:");
	upd_struct_.Print("UpdateStructure:");
	rec_struct_.Print("RecoverStructure:");
	upd_map_.Print("UpdateStateMap:");
	upd_map_collision_.Print("DetectCollision:");
	rec_map_.Print("RecoverStateMap:");
	test_stiff_.Print("TestifyStiffness:");
}


void FFAnalyzer::WriteRenderPath(int min_layer, int max_layer, char *ptr_path)
{
	int layer_size = ptr_frame_->SizeOfLayer();
	if (layer_size == 0)
	{
		return;
	}

	Init();
	min_layer--;
	max_layer--;

	layers_.resize(layer_size);
	for (int dual_i = 0; dual_i < Nd_; dual_i++)
	{
		int orig_i = ptr_wholegraph_->e_orig_id(dual_i);
		int layer = ptr_frame_->GetEdge(orig_i)->Layer();
		layers_[layer].push_back(dual_i);
	}

	/* skip pillars */
	int hi = 0;
	for (; hi < Nd_; hi++)
	{
		WF_edge *e = print_queue_[hi];
		if (!e->isPillar())
		{
			break;
		}
		UpdateStructure(e);
	}

	/* skip printed structure */
	for (; hi < Nd_; hi++)
	{
		WF_edge *e = print_queue_[hi];
		if (e->Layer() >= min_layer)
		{
			break;
		}
		UpdateStructure(e);
	}

	/* angle state with printed structure */
	for (int dual_i = 0; dual_i < Nd_; dual_i++)
	{
		int orig_i = ptr_wholegraph_->e_orig_id(dual_i);
		if (!ptr_dualgraph_->isExistingEdge(orig_i))
		{
			WF_edge *e = ptr_frame_->GetEdge(orig_i);
			ptr_collision_->DetectCollision(e, ptr_dualgraph_, angle_state_[dual_i]);
		}
	}

	/* print starting from min_layer */
	WF_edge *ei = NULL;
	for (; hi < Nd_ - 1; hi++)
	{
		WF_edge *ej = print_queue_[hi];
		int dual_j = ptr_wholegraph_->e_dual_id(ej->ID());
		int l = ej->Layer();
		int Nl = layers_[l].size();

		// init the next layer
		if (ei == NULL || ei->Layer() != l)
		{
			/* max_z_ and min_z_ in current layer */
			min_z_ = 1e20;
			max_z_ = -min_z_;
			for (int k = 0; k < Nl; k++)
			{
				int orig_k = ptr_wholegraph_->e_orig_id(layers_[l][k]);
				point u = ptr_frame_->GetEdge(orig_k)->pvert_->Position();
				point v = ptr_frame_->GetEdge(orig_k)->ppair_->pvert_->Position();
				min_z_ = min(min_z_, (double)min(u.z(), v.z()));
				max_z_ = max(max_z_, (double)max(u.z(), v.z()));
			}
		}

		vector<double> cost(Nd_);
		double min_cost = 1e20;
		double max_cost = -min_cost;
		for (int k = 0; k < Nl; k++)
		{
			int dual_k = layers_[l][k];
			cost[dual_k] = GenerateCost(l, k, ei);
			if (cost[dual_k] != -1)
			{
				if (cost[dual_k] < min_cost)
				{
					min_cost = cost[dual_k];
				}
				if (cost[dual_k] > max_cost)
				{
					max_cost = cost[dual_k];
				}
			}
		}

		char id[10];
		sprintf(id, "%d", hi);

		string path = ptr_path;
		string file = path + "/PathRender_" + id + ".txt";
		FILE *fp = fopen(file.c_str(), "w+");
		//vector<vector<double>> writer;
	/*	double cost_max = 0;
		double cost_min = 1;*/
		for (int dual_k = 0; dual_k < Nd_; dual_k++)
		{
			double r;
			double g;
			double b;
			int orig_k = ptr_wholegraph_->e_orig_id(dual_k);
			WF_edge *e = ptr_frame_->GetEdge(orig_k);

			if (ptr_dualgraph_->isExistingEdge(orig_k))
			{
				r = 0.5;
				g = 0.5;
				b = 0.5;
			}
			else
			if (e->Layer() != l)
			{
				r = 1.0;
				g = 1.0;
				b = 1.0;
			}
			else
			if (cost[dual_k] == -1)
			{
				r = 0;
				g = 0;
				b = 0;
			}
			else
			{
				double cost_k;
				if (max_cost == -1e20 || max_cost == min_cost)
				{
					cost_k = 0.0;
				}
				else
				{
					cost_k = (cost[dual_k] - min_cost) / (max_cost - min_cost);
				}

				r = 1.0;
				g = cost_k;
				b = 0.0;
			}
			point u = e->pvert_->RenderPos();
			point v = e->ppair_->pvert_->RenderPos();
			fprintf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
				u.x(), u.y(), u.z(), v.x(), v.y(), v.z(), r, g, b);
		/*	if (g > cost_max)
				cost_max = g;
			if (g < cost_min)
				cost_min = g;

			point u = e->pvert_->RenderPos();
			point v = e->ppair_->pvert_->RenderPos();
			vector<double> temp_9;
			temp_9.push_back(u.x());
			temp_9.push_back(u.y());
			temp_9.push_back(u.z());
			temp_9.push_back(v.x());
			temp_9.push_back(v.y());
			temp_9.push_back(v.z());
			temp_9.push_back(r);
			temp_9.push_back(g);
			temp_9.push_back(b);	
			writer.push_back(temp_9);*/
		}
		//for (int i = 0; i < writer.size();i++)
		//{
		//fprintf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
		//	writer[i][0], writer[i][1], writer[i][2], writer[i][3], writer[i][4], writer[i][5], writer[i][6], (writer[i][7]-cost_min)/(cost_max-cost_min), writer[i][8]);//mapping to [0,1]
		//}
		fclose(fp);

		vector<vector<lld>> tmp_angle(3);
		UpdateStateMap(dual_j, tmp_angle);

		UpdateStructure(ej);
		ei = ej;
	}
}

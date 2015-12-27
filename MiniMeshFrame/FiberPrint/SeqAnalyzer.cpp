#include "SeqAnalyzer.h"


SeqAnalyzer::SeqAnalyzer()
	:gamma_(100), Dt_tol_(0.1), Dr_tol_(10 * F_PI / 180), Wl_(10.0), Wp_(100.0)
{
	extru_ = false;
}


SeqAnalyzer::SeqAnalyzer(GraphCut *ptr_graphcut)
	:gamma_(100), Dt_tol_(0.1), Dr_tol_(10 * F_PI / 180), Wl_(10.0), Wp_(100.0)
{
	ptr_graphcut_ = ptr_graphcut;
	extru_ = false;
}


SeqAnalyzer::SeqAnalyzer(GraphCut *ptr_graphcut, FiberPrintPARM *ptr_parm, char *path)
{
	ptr_graphcut_ = ptr_graphcut;

	gamma_	= ptr_parm->gamma_;
	Dt_tol_	= ptr_parm->Dt_tol_;
	Dr_tol_ = ptr_parm->Dr_tol_;
	Wl_		= ptr_parm->Wl_;
	Wp_		= ptr_parm->Wp_;
	debug_  = 1;

	path_ = path;
	extru_ = false;
}


SeqAnalyzer::~SeqAnalyzer()
{
}


bool SeqAnalyzer::LayerPrint()
{
	DualGraph *ptr_dualgraph = ptr_graphcut_->ptr_dualgraph_;
	ptr_dualgraph->Dualization();
	int Nd = ptr_dualgraph->SizeOfVertList();

	/* ---- */
	WireFrame *ptr_wf = ptr_dualgraph->ptr_frame_;
	std::vector<WF_edge*> wf_edge_list = *ptr_wf->GetEdgeList();
	vector<double> ss;

	for (int i = 1; i < Nd; i++)
	{
		int e_id = ptr_dualgraph->e_orig_id(i);

		WF_edge *ei = ptr_wf->GetEdge(e_id);
		WF_vert *ui = ei->pvert_;
		WF_vert *vi = ei->ppair_->pvert_;

		printf("edge dual id : %d, end node coord:\n", i);
		printf("%f, %f, %f\n", ui->Position().x(), ui->Position().y(), ui->Position().z());
		ss.push_back(ui->Position().x());
		ss.push_back(ui->Position().y());
		ss.push_back(ui->Position().z());
		printf("%f, %f, %f\n", vi->Position().x(), vi->Position().y(), vi->Position().z());
		ss.push_back(vi->Position().x());
		ss.push_back(vi->Position().y());
		ss.push_back(vi->Position().z());
	}

	Statistics tmp_s("restrict", ss);
	tmp_s.GenerateStdVecFile();

	getchar();
	/* --- */


	WireFrame *ptr_frame = ptr_graphcut_->ptr_frame_;
	int N = ptr_frame->SizeOfVertList();

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
			QueueInfo start_edge = QueueInfo{ l, it->second, layers_[l][it->second] };
			layer_queue_.push_back(start_edge);

			if (GenerateSeq(l, h, t))
			{
				success = true;
				break;
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

	///* Feasible printing orientation */
	//wave_.clear();
	//for (int i = 0; i < Nd; i++)
	//{
	//	wave_.push_back(0);
	//}

	//angle_list_ = AngleList(layer_queue_);
	AngleDec();
}


bool SeqAnalyzer::GenerateSeq(int l, int h, int t)
{
	if (debug_)
	{
		printf("searching edge #%d in layer %d, head %d, (tail %d)\n", 
			layer_queue_[h].layer_id_, l, h, t);
	}

	DualGraph *ptr_dualgraph = ptr_graphcut_->ptr_dualgraph_;
	WireFrame *ptr_frame = ptr_subgraph_->ptr_frame_;
	int Nl = layers_[l].size();

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

	/* ranked by weight */
	for (it = choice.begin(); it != choice.end(); it++)
	{
		QueueInfo next_edge = QueueInfo{ l, it->second, layers_[l][it->second] };
		layer_queue_.push_back(next_edge);

		if (debug_)
		{
			printf("choose edge #%d in layer %d with cost %lf\n", it->second, l, it->first);
			printf("entering next searching state.\n");
		}

		if (GenerateSeq(l, h + 1, t))
		{
			return true;
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

	int dual_j = layers_[l][j];
	int orig_j = ptr_dualgraph->e_orig_id(dual_j);

	if (!ptr_subgraph_->isExistingEdge(orig_j))
	{
		double	P;							// adjacency weight
		double	L;							// collision weight

		if (debug_)
		{
			printf("Attempting edge #%d, layer %d\n", j, l);
		}

		/* adjacency weight */
		int u = ptr_frame->GetEndu(orig_j);
		int v = ptr_frame->GetEndv(orig_j);
		if (ptr_subgraph_->isExistingVert(u) && ptr_subgraph_->isExistingVert(v))
		{
			/* edge j share two ends with printed structure */
			if (debug_)
			{
				printf("it shares two ends with printed structure\n");
			}
			P = 0;
		}
		else
		if (ptr_subgraph_->isExistingVert(u) || ptr_subgraph_->isExistingVert(v))
		{
			/* edge j share one end with printed structure */
			if (debug_)
			{
				printf("it shares only one ends with printed structure\n");
			}
			P = 1;
		}
		else
		{
			if (debug_)
			{
				printf("it floats, skip\n");
			}
			return -1;
		}

		/* collision weight */
		Collision *ptr_collision = new Collision(ptr_frame, ptr_frame->GetEdge(orig_j));
		ptr_collision->DetectCollision(ptr_subgraph_);

		//L = 1 - (double)ptr_collision->AvailableAngle() / ptr_collision->Divide();
		L =  (double)ptr_collision->AvailableAngle() / ptr_collision->Divide();
		
		if (0 == L)
		{
			return -1;
		}
		
		delete ptr_collision;
		ptr_collision = NULL;

		///* stiffness */
		///* insert a trail edge */
		//ptr_subgraph_->UpdateDualization(ptr_frame->GetEdge(orig_j));

		///* examinate stiffness on printing subgraph */
		//Stiffness *ptr_stiffness = new Stiffness(ptr_subgraph_);
		//int Ns = ptr_subgraph_->SizeOfFreeFace();
		//VX D(Ns);
		//D.setZero();

		//printf("------------\n");
		//printf("Layers %d, head index %d\n", l, h);
		//printf("Trial Deformation calculation edge %d\n", dual_j);
		//bool stiff_success = true;
		//if (ptr_stiffness->CalculateD(D))
		//{
		//	for (int k = 0; k < Ns; k++)
		//	{
		//		VX offset(3);
		//		VX distortion(3);
		//		for (int h = 0; h < 3; h++)
		//		{
		//			offset[h] = D[j * 6 + h];
		//			distortion[h] = D[j * 6 + h + 3];
		//		}

		//		if (offset.norm() >= Dt_tol_ || distortion.norm() >= Dr_tol_)
		//		{
		//			stiff_success = false;
		//			break;
		//		}
		//	}
		//}
		//else
		//{
		//	stiff_success = false;
		//}

		///* remove the trail edge */
		//ptr_subgraph_->RemoveUpdation(ptr_frame->GetEdge(orig_j));

		//delete ptr_stiffness;
		//ptr_stiffness = NULL;

		///* examination failed */
		//if (!stiff_success)
		//{
		//	return -1;
		//}

		double cost = Wl_ * L + Wp_ * P;
		if (debug_)
		{
			printf("cost : %f\n", cost);
		}
		return cost;
	}

	return -1;
}


void SeqAnalyzer::GetQueue(vector<int> &layer_queue)
{
	int Nq = layer_queue_.size();
	for (int i = 0; i < Nq; i++)
	{
		layer_queue.push_back(layer_queue_[i].dual_id_);
	}
}


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

//vector<GeoV3> SeqAnalyzer::AngleList(vector<QueueInfo> *layer_queue)
//{
//	support_ = 0;
//	vector<GeoV3> temp;
//
//	cout << "------------------------ " << endl;
//	cout << "Feasible Angle Computing:" << endl;
//
//	WF_edge*  temp_edge = ptr_graphcut_->ptr_frame_->GetEdge(ptr_graphcut_->ptr_dualgraph_->e_orig_id((*layer_queue_)[0].dual_id_));
//	double base_value;
//	
//	base_value = temp_edge->pvert_->Position().z();
//	if (base_value > temp_edge->ppair_->pvert_->Position().z())
//		base_value = temp_edge->ppair_->pvert_->Position().z();
//
//	std::array<float, 3>s;
//	s[0] = 0; s[1] = 0; s[2] = 1;
//	table_.normal = s;
//	table_.constant = base_value;
//
//	for (int i = 0; i < layer_queue->size(); i++)
//	{
//		cout << "edge " << i << " angle in processing" << endl;
//		temp.push_back(AngleDec(i));
//	}
//	return temp;
//}
//
//GeoV3 SeqAnalyzer::AngleDec(int id)
//{
//	/* sampling number of orientation vector in 2*pi angle range */
//	int divide = 72;
//	vector<GeoV3> normal;
//	
//	WF_edge*  temp_edge = ptr_graphcut_->ptr_frame_->GetEdge(ptr_graphcut_->ptr_dualgraph_->e_orig_id((*layer_queue_)[id].dual_id_));
//	if (temp_edge->isPillar())
//	{
//		support_ += 1;
//		return GeoV3(0, 0, 1);
//	}
//
//	point start, end;
//	start = temp_edge->pvert_->Position();
//	end   = temp_edge->ppair_->pvert_->Position();
//	start_ = temp_edge->pvert_->Position();
//	end_   = temp_edge->ppair_->pvert_->Position();
//
//
//	GeoV3 u, v;
//
//	GeoV3 t = end_ - start_;
//	t.normalize();
//	GeoV3 z(0, 0, 1);
//
//	if (Geometry::cross(t, z).norm() < eps)
//	{
//		/* vertical case */
//		u = Geometry::Vector3d(1, 0, 0);
//		v = Geometry::Vector3d(0, 1, 0);
//	}
//	else
//	{
//		/* perpendicular to printing edge */
//		u = Geometry::cross(t, z);
//		u.normalize();
//		v = Geometry::cross(u, t);
//		v.normalize();
//	}
//
//	/* normal contains all sampling vectors */
//	for (int i = 0; i < divide; i++)
//	{
//		normal.push_back(u*cos(2 * F_PI / divide*i) + v*sin(2 * F_PI / divide*i));
//	}
//
//	/* i traverse all the printed edges until current state id */
//	for (int i = 0; i < id; i++)
//	{
//		vector<GeoV3>::iterator it;
//		/* influence from id to i 
//		* i.e. the collision cost of existence of edge i
//		* when printing edge id
//		*/
//		RAngle TestR = ptr_collision_->R2_Angle[(*layer_queue_)[id].dual_id_][(*layer_queue_)[i].dual_id_];
//
//		if (ptr_collision_->R2_state_[(*layer_queue_)[id].dual_id_][(*layer_queue_)[i].dual_id_] == 0)
//			continue;
//
//		for (it = normal.begin(); it != normal.end();)
//		{
//			if (IsColVec(TestR.u, TestR.v, *it ))
//			{
//				it = normal.erase(it);
//			}
//			else
//			{
//				++it;
//			}
//		}
//	}
//
//	if (normal.size() == 0)
//	{
//		cout << " Infeasible orientation!" << endl;
//		wave_[id] = 0;
//		return GeoV3(0, 0, 1);
//		return GeoV3(0, 0, 1);
//	}
//
//	if (normal.size() == 72)
//	{
//		wave_[id] = 2 * F_PI;
//		return normal[0];
//	}
//
//	ResolveAngle resolve;
//	resolve = ResolveAngle(normal);
//
//	wave_[id] = resolve.wave;
//	return resolve.dec;
//}
//
//bool  SeqAnalyzer::IsColVec(GeoV3 start, GeoV3 end, GeoV3 target)
//{
//	if (Geometry::angle(target, start) <= extruder_.Angle())
//		return true;
//
//	if (Geometry::angle(target, end) <= extruder_.Angle())
//		return true;
//
//	if (abs(Geometry::angle(target, start) + Geometry::angle(target, end) - Geometry::angle(target, end)) < eps)
//		return true;
//
//	//if (Geometry::angle(target, Geometry::Vector3d(0, 0, -1)) < 2*extruder_.Angle())
//	//	return true;
//
//	gte::Circle3<float>circle;
//	std::array<float,3> s,n;
//	
//	// Collision with Table
//	GeoV3 low = start_;
//	if (low.getZ()>end_.z());
//	   low = end_;
//	
//	   GeoV3 t = end_ - start_;
//	   t.normalize();
//	   GeoV3 u = target;
//	   u.normalize();
//	   GeoV3 v = Geometry::cross(u, t);
//	   v.normalize();
//
//	  GeoV3 test;
//	   test = low + target*extruder_.Height();
//	   s[0] = test.getX(); s[1] = test.getY(); s[2] = test.getZ();
//	   circle.center = s;
//	   n[0] = target.getX(); n[1] = target.getY(); n[2] = target.getZ();
//	   circle.normal = n;
//	   circle.radius = extruder_.Height()*tan(extruder_.Angle());
//
//	   gte::FIQuery<float, gte::Plane3<float>, gte::Circle3<float>> intersection;
//
//	   auto result = intersection(table_, circle);
//	   if (result.intersect)
//	   {		  
//		   return true;
//	   }
//	   if (s[2] < table_.constant)
//	   {
//		  
//		   return true;
//	   }
//	
//	return false;
//}
//
//void SeqAnalyzer::Print(Set *a)
//{
//	cout << a->min << " " << a->max << " " << endl;
//}
//
//void SeqAnalyzer::Print(vector<Set*> *a)
//{
//	for (int i = 0; i < a->size(); i++)
//	{
//
//		Print((*a)[i]);
//	}
//}
//
//void SeqAnalyzer::Debug()
//{
//}


void SeqAnalyzer::AngleDec()
{

	/*vector<int> quene{ 123, 127, 122, 124, 128, 125, 121, 120, 126, 129, 131, 130, 116, 119, 118, 117, 9, 77, 12, 15, 78, 80, 10, 11, 13, 6, 82, 7, 8, 14, 79, 81, 16, 30, 103, 34, 97, 96, 31, 75, 27, 29, 28, 99, 26, 72, 32, 33, 73, 104, 35, 110, 107, 25, 100, 108, 105, 71, 63, 109, 93, 94, 102, 101, 98, 70, 76, 74, 24, 23, 95, 20, 67, 22, 21, 62, 106, 54, 64, 55, 84, 18, 68, 61, 17, 19, 83, 65, 49, 69, 66, 50, 60, 53, 39, 40, 56, 87, 57, 51, 52, 38, 41, 88, 89, 58, 47, 59, 86, 46, 91, 113, 42, 111, 85, 48, 37, 45, 36, 43, 90, 44, 92, 114, 115, 112, 4, 5, 3, 1, 2, 0,
};
	layer_queue_.clear();
	for (int i = 0; i < quene.size(); i++)

	{

		QueueInfo test;
		test.dual_id_ = quene[i];
		layer_queue_.push_back(test);
	}
	
*/

	cout << "---------Angle Detection--------" << endl;
	DualGraph *ptr_dual = new DualGraph(ptr_graphcut_->ptr_frame_);

	ptr_dual->Dualization();

	vector<GeoV3 > Normal;
	vector<double> Wave;

	support_ = 0;
	for (int i = 0; i < layer_queue_.size(); i++)
	{
		WF_edge * target = ptr_graphcut_->ptr_frame_->GetEdge(ptr_dual->e_orig_id(layer_queue_[i].dual_id_));

		if (target->isPillar())
		{
			Normal.push_back(GeoV3(0, 0, 1));
			Wave.push_back(2 * F_PI);

			ExtruderCone extruder_;
			
			support_ += 1;
			continue;
		}
Collision col(ptr_graphcut_->ptr_frame_, target);
		for (int j = 0; j < i; j++)
		{
			
			col.DetectCollision(ptr_graphcut_->ptr_frame_->GetEdge(ptr_dual->e_dual_id(layer_queue_[j].dual_id_)));

			if (col.normal_.size() == 0)
			{
				cout << "Oops~~, What is wrong?!" << endl;
			}
			
			
		}
		ResolveAngle resolve(col.normal_);
Normal.push_back(resolve.dec);
			Wave.push_back(resolve.wave);

	}

	wave_ = Wave;



	//Extruder
	for (int i = 0; i < layer_queue_.size(); i++)
	{
		extru_ = true;
		ExtruderCone temp_extruder;

		/* original edge id */
		WF_edge*  temp_edge = ptr_graphcut_->ptr_frame_->GetEdge(  ptr_dual->e_orig_id((layer_queue_)[i].dual_id_));

		temp_extruder.Rotation(Normal[i], temp_edge->pvert_->Position(), temp_edge->ppair_->pvert_->Position());
		extruder_list_.push_back(temp_extruder);
	}




	cout << "---------Angle Detection done--------" << endl;
}
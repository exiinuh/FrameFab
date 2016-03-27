#include "ADMMCut.h"


ADMMCut::ADMMCut()
	:penalty_(0),
	Dt_tol_(0), Dr_tol_(0),
	pri_tol_(0), dual_tol_(0)
{
	// This default construction function should never be run
	// We need a mesh to begin with
}


ADMMCut::ADMMCut(WireFrame *ptr_frame)
	:penalty_(10e2),
	Dt_tol_(5), Dr_tol_(10 * F_PI / 180),
	pri_tol_(10e-3), dual_tol_(10e-3)
{
	ptr_frame_ = ptr_frame;
	ptr_dualgraph_ = new DualGraph(ptr_frame_);
	ptr_stiff_ = new Stiffness(ptr_dualgraph_);
}


ADMMCut::ADMMCut(WireFrame *ptr_frame, FiberPrintPARM *ptr_parm, char *path)
{
	ptr_frame_ = ptr_frame;
	ptr_dualgraph_ = new DualGraph(ptr_frame_);
	ptr_stiff_ = new Stiffness(ptr_dualgraph_, ptr_parm);

	Dt_tol_ = ptr_parm->Dt_tol_;
	Dr_tol_ = ptr_parm->Dr_tol_;
	penalty_ = ptr_parm->penalty_;
	pri_tol_ = ptr_parm->pri_tol_;
	dual_tol_ = ptr_parm->dual_tol_;

	path_ = path;
	debug_ = true;
}


ADMMCut::~ADMMCut()
{
	delete qp_;
	qp_ = NULL;
}


void ADMMCut::InitState()
{
	N_ = ptr_frame_->SizeOfVertList();
	M_ = ptr_frame_->SizeOfEdgeList();

	// set termination tolerance 
	stop_n_ = floor(N_ / 5);

	qp_ = QPFactory::make(static_cast<QPFactory::QPType>(1));

	// clear layer label
	for (int i = 0; i < M_; i++)
	{
		ptr_frame_->GetEdge(i)->SetLayer(0);
	}

	// upper dual id 
	for (int i = 0; i < M_; i++)
	{
		WF_edge *e = ptr_frame_->GetEdge(i);
		if (e->isCeiling())
		{
			cutting_edge_.push_back(i);
			//e->SetLayer(1);
			//e->ppair_->SetLayer(1);
		}
	}

	cout << "penalty : " << penalty_ << endl;
	cout << "primal tolerance : " << pri_tol_ << endl;
	cout << "dual tolerance : " << dual_tol_ << endl;
	cout << "ADMMCut Start" << endl;
	cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
}


void ADMMCut::SetStartingPoints(int count)
{
	if (count == 0)
	{
		ptr_dualgraph_->Dualization();
		Nd_w_ = ptr_dualgraph_->SizeOfVertList();
	}
	else
	{
		ptr_dualgraph_->UpdateDualization(&x_);
	}

	Nd_ = ptr_dualgraph_->SizeOfVertList();
	Md_ = ptr_dualgraph_->SizeOfEdgeList();
	Fd_ = ptr_dualgraph_->SizeOfFaceList();
	Ns_ = ptr_dualgraph_->SizeOfFreeFace();

	ptr_stiff_->Init();

	/* Reweighting Paramter */
	r_.resize(Nd_, Nd_);
	x_.resize(Nd_);
	lambda_.resize(6 * Ns_);
	a_.resize(Nd_);

	// Set all label x = 1 and calculate KD = F to obtain inital D_0
	r_.setOnes();
	x_.setOnes();
	lambda_.setZero();
	a_.setZero();

	//ptr_stiff_->Debug();
}


void ADMMCut::SetBoundary()
{
	// Set lower boundary and upper boundary
	// equality constraints W*x = d
	// Identify Base nodes(1) and Upper Nodes(2)
	// Here we just take the edge with biggest height
	d_.resize(Nd_);
	d_.setZero();

	W_.resize(Nd_, Nd_);
	W_.setZero();

	VectorXi bound(Nd_);
	bound.setZero();

	// upper dual
	int cuts = cutting_edge_.size();
	for (int i = 0; i < cuts; i++)
	{
		int e_id = ptr_dualgraph_->e_dual_id(cutting_edge_[i]);
		bound[e_id] = 2;
		x_[e_id] = 0;
	}

	// base dual 
	// find boundary nodes, in current stage boundary edge = base edge (for cat_head)
	for (int i = 0; i < M_; i++)
	{
		WF_edge *e = ptr_frame_->GetEdge(i);
		if (e->ID() < e->ppair_->ID())
		{
			int e_id = ptr_dualgraph_->e_dual_id(i);
			int u = e->ppair_->pvert_->ID();
			int v = e->pvert_->ID();
			if (e_id != -1 && (ptr_frame_->isFixed(u) || ptr_frame_->isFixed(v)))
			{
				bound[e_id] = 1;
			}
		}
	}

	vector<Eigen::Triplet<double>> W_list;
	for (int e_id = 0; e_id < Nd_; e_id++)
	{
		if (bound[e_id] == 1)
		{
			d_[e_id] = 1;
			W_list.push_back(Eigen::Triplet<double>(e_id, e_id, 1));
		}
		if (bound[e_id] == 2)
		{
			d_[e_id] = 0;
			W_list.push_back(Eigen::Triplet<double>(e_id, e_id, 1));
		}
	}
	W_.setFromTriplets(W_list.begin(), W_list.end());
}


void ADMMCut::CreateA()
{
	A_.resize(Md_, Nd_);
	vector<Triplet<double>> A_list;
	for (int i = 0; i < Md_; i++)
	{
		int u = ptr_dualgraph_->u(i);
		int v = ptr_dualgraph_->v(i);
		A_list.push_back(Triplet<double>(i, ptr_dualgraph_->u(i), 1));
		A_list.push_back(Triplet<double>(i, ptr_dualgraph_->v(i), -1));
	}
	A_.setFromTriplets(A_list.begin(), A_list.end());
}


void ADMMCut::CreateC(int cut, int rew)
{
	C_.resize(Md_, Md_);
	vector<Triplet<double>> C_list;
	vector<double> v_r;
	vector<double> v_c;

	for (int i = 0; i < Md_; i++)
	{
		int u = ptr_dualgraph_->u(i);
		int v = ptr_dualgraph_->v(i);
		C_list.push_back(Triplet<double>(i, i, pow(ptr_dualgraph_->Weight(i), 2) * r_(u, v)));
		v_r.push_back(r_(u, v));
		v_c.push_back(pow(ptr_dualgraph_->Weight(i), 2) * r_(u, v));
	}
	C_.setFromTriplets(C_list.begin(), C_list.end());

	string str_c = "Cut_" + to_string(cut) + "_Rew_" + to_string(rew) + "_C";
	Statistics s_c(str_c, v_c);
	s_c.GenerateStdVecFile();

	//string str_r = "Cut_" + to_string(cut) + "_Rew_" + to_string(rew) + "_R";
	//Statistics s_r(str_r, v_r);
	//s_r.GenerateStdVecFile();

	H1_ = SpMat(Nd_, Nd_);
	H1_ = A_.transpose() * C_ * A_;
}


bool ADMMCut::CheckLabel(int count)
{
	int l = 0;													// Number of dual vertex in lower set
	int u = 0;													// Number of dual vertex in upper set

	for (int i = 0; i < Nd_; i++)
	{
		if (x_[i] == 1)
		{
			l++;
		}
		else
		{
			u++;
		}
	}

	cout << "--------------------------------------------" << endl;
	cout << "ADMMCut REPORT" << endl;
	cout << "ADMMCut Round : " << count << endl;
	cout << "Lower Set edge number : " << l << "\\ " << Nd_w_ << " (Whole dual graph Nd)" << endl;
	cout << "Lower Set percentage  : " << double(l) / double(Nd_w_) * 100 << "%" << endl;
	cout << "--------------------------------------------" << endl;

	if (l < 20 || l < ptr_frame_->SizeOfPillar())
	{
		return true;
	}
	else
	{
		return false;
	}
}


bool ADMMCut::TerminationCriteria(int count)
{
	if (count >= 20)
	{
		return true;
	}

	if (primal_res_.norm() <= pri_tol_ && dual_res_.norm() <= dual_tol_)
	{
		return true;
	}
	else
	{
		double p_r = primal_res_.norm();
		double d_r = dual_res_.norm();

		if (p_r > d_r)
		{
			penalty_ *= 2;
		}

		if (p_r < d_r)
		{
			penalty_ /= 2;
		}

		return false;
	}
}


void ADMMCut::MakeLayers()
{
	// Initial Cutting Edge Setting
	InitState();

	debug_ = false;
	int cut_count = 0;
	vector<double> cut_energy;
	vector<double> res_energy;
	do
	{
		/* Recreate dual graph at the beginning of each cut */
		SetStartingPoints(cut_count);
		CreateA();

		ptr_stiff_->CalculateD(D_, x_, 0, 0, cut_count);

		///* for rendering */
		//if (cut_count == 0)
		//{
		//	WriteWeight();
		//	WriteStiffness("offset1.txt", "rotation1.txt");
		//	getchar();
		//}
		//if (cut_count == 4)
		//{
		//	WriteStiffness("offset2.txt", "rotation2.txt");
		//	getchar();
		//}
		//if (cut_count == 6)
		//{
		//	WriteStiffness("offset3.txt", "rotation3.txt");
		//	getchar();
		//}

		/* set x for intial cut setting */
		SetBoundary();

		/*
		* energy specify:
		* cut		 energy : | A * x |
		* defomation energy : norm(K D - F)
		*/
		ptr_stiff_->CreateGlobalK(x_);
		ptr_stiff_->CreateF(x_);
		SpMat K_init = *(ptr_stiff_->WeightedK());
		VX	  F_init = *(ptr_stiff_->WeightedF());


		double icut_energy = 0;
		VX V_Cut = A_ * x_;
		for (int i = 0; i < Md_; i++)
		{
			icut_energy += abs(V_Cut[i]);
		}
		double ideform_energy = (K_init * D_ - F_init).norm();

		cout << "****************************************" << endl;
		cout << "ADMMCut Round : " << cut_count << endl;
		cout << "Initial cut energy before entering ADMM: " << icut_energy << endl;
		cout << "Initial Lagrangian energy before entering ADMM: " << ideform_energy << endl;
		cout << "---------------------------------" << endl;

		int rew_count = 0;
		VX x_prev;
		VX D_prev;

		/* Output energy list for reweighting process in a single
		graph cut problem, energy.size() = number of reweighting
		process performed.
		cut energy = |A_ * x_| = sum_{e_ij} w_ij * |x_i - x_j|
		res energy = (K(x)D - F(x)).norm()
		*/

		cut_energy.clear();
		res_energy.clear();

		cut_energy.push_back(icut_energy);
		res_energy.push_back(ideform_energy);

		do
		{
			/* Reweighting loop for cut */

			int ADMM_count = 0;
			x_prev = x_;
			CreateC(cut_count, rew_count);

			do
			{
				cout << "ADMMCut Round: " << cut_count << ", reweight iteration:" << rew_count
					<< ", ADMM " << ADMM_count << " iteration." << endl;

				/*-------------------ADMM loop-------------------*/
				CalculateX();

				D_prev = D_;
				CalculateD();

				UpdateLambda();

				/*-------------------Residual Calculation-------------------*/
				SpMat Q_prev;
				SpMat Q_new;
				CalculateQ(D_prev, Q_prev);
				CalculateQ(D_, Q_new);

				/* Update K reweighted by new x */
				ptr_stiff_->CreateGlobalK(x_);
				ptr_stiff_->CreateF(x_);
				SpMat K_new = *(ptr_stiff_->WeightedK());
				VX    F_new = *(ptr_stiff_->WeightedF());

				dual_res_ = penalty_ * (D_prev - D_).transpose() * K_new.transpose() * Q_prev
					+ lambda_.transpose() * (Q_prev - Q_new);
				primal_res_ = K_new * D_ - F_new;

				/*-------------------Screenplay-------------------*/
				double new_cut_energy = x_.dot(H1_ * x_);

				cout << "new quadratic func value record: " << new_cut_energy << endl;
				cout << "dual_residual : " << dual_res_.norm() << endl;
				cout << "primal_residual(KD-F) : " << primal_res_.norm() << endl;

				cout << "---------------------" << endl;
				ADMM_count++;
			} while (!TerminationCriteria(ADMM_count));

			/* One reweighting process ended! */
			/* Output energy and residual */

			double energy = 0;
			VX V_Cut = A_ * x_;
			for (int i = 0; i < Md_; i++)
			{
				energy += ptr_dualgraph_->Weight(i) * abs(V_Cut[i]);
			}

			/*-------------------Screenplay-------------------*/
			double res_tmp = primal_res_.norm();
			cout << "Cut " << cut_count << " Reweight " << rew_count << " completed." << endl;
			cout << "Cut Energy :" << energy << endl;
			cout << "Res Energy :" << res_tmp << endl;
			cout << "<-<-<-<-<-<-<-<-<-<-<-<-<-<-<-<-<-" << endl;

			cut_energy.push_back(energy);
			res_energy.push_back(res_tmp);

			/* write x distribution to a file */
			string str_x = "Cut_" + to_string(cut_count) + "_Rew_" + to_string(rew_count) + "_x";
			Statistics tmp_x(str_x, x_);
			tmp_x.GenerateVectorFile();

			rew_count++;
		} while (!UpdateR(x_prev, rew_count));

		/* Output reweighting energy history for last cut process */
		string str_eC = "Cut_" + to_string(cut_count) + "_Cut_Energy";
		Statistics s_eC(str_eC, cut_energy);
		s_eC.GenerateStdVecFile();

		string str_eR = "Cut_" + to_string(cut_count) + "_Res_Energy";
		Statistics s_eR(str_eR, res_energy);
		s_eR.GenerateStdVecFile();

		/* Update New Cut information to Rendering (layer_label_) */

		UpdateCut();

		fprintf(stdout, "ADMMCut No.%d process is Finished!\n", cut_count);
		cut_count++;

	} while (!CheckLabel(cut_count));

	fprintf(stdout, "All done!\n");
}


void ADMMCut::CalculateX()
{
	// Construct Hessian Matrix for D-Qp problem
	SpMat Q;
	CalculateQ(D_, Q);

	SpMat H2 = Q.transpose() * Q;
	SpMat H = 2 * H1_ + penalty_ * H2;

	// Construct Linear coefficient for x-Qp problem
	a_ = Q.transpose() * lambda_;

	// Inequality constraints A*x <= b
	SpMat A(6 * Fd_, 6 * Fd_);
	A.setZero();
	VX b(6 * Fd_);
	b.setZero();

	// Variable constraints x >= lb, x <= ub
	VX lb(Nd_), ub(Nd_);
	lb.setZero();
	ub.setOnes();

	qp_->solve(H, a_, A, b, W_, d_, lb, ub, x_, NULL, NULL, debug_);
}


void ADMMCut::CalculateQ(const VX _D, SpMat &Q)
{
	// Construct Hessian Matrix for D-Qp problem
	Q.resize(6 * Ns_, Nd_);
	vector<Eigen::Triplet<double>> Q_list;

	for (int i = 0; i < Ns_; i++)
	{
		int u = ptr_dualgraph_->v_orig_id(i);
		WF_edge *edge = ptr_frame_->GetNeighborEdge(u);

		while (edge != NULL)
		{
			int e_id = ptr_dualgraph_->e_dual_id(edge->ID());
			if (e_id != -1)
			{
				int v = edge->pvert_->ID();
				int j = ptr_dualgraph_->v_dual_id(v);

				MX eKuu = ptr_stiff_->eKv(edge->ID());
				MX eKeu = ptr_stiff_->eKe(edge->ID());
				VX Fe = ptr_stiff_->Fe(edge->ID());
				VX Di(6);
				VX Dj(6);

				if (i < Ns_ && j < Ns_)
				{
					for (int k = 0; k < 6; k++)
					{
						Di[k] = _D[6 * i + k];
						Dj[k] = _D[6 * j + k];
					}
				}
				else
				{
					if (i < Ns_)
					{
						for (int k = 0; k < 6; k++)
						{
							Di[k] = _D[6 * i + k];
							Dj[k] = 0;
						}
					}

					if (j < Ns_)
					{
						for (int k = 0; k < 6; k++)
						{
							Di[k] = 0;
							Dj[k] = _D[6 * j + k];
						}
					}
				}
				VX Gamma = eKuu * Di + eKeu * Dj - Fe;

				for (int k = 0; k < 6; k++)
				{
					Q_list.push_back(Triplet<double>(6 * i + k, e_id, Gamma[k]));
				}
			}

			edge = edge->pnext_;
		}
	}

	Q.setFromTriplets(Q_list.begin(), Q_list.end());
}


void ADMMCut::CalculateD()
{
	// Construct Hessian Matrix for D-Qp problem
	// Here, K is continuous-x weighted
	ptr_stiff_->CreateGlobalK(x_);
	SpMat K = *(ptr_stiff_->WeightedK());
	SpMat Q = penalty_ * K.transpose() * K;

	// Construct Linear coefficient for D-Qp problem
	ptr_stiff_->CreateF(x_);
	VX F = *(ptr_stiff_->WeightedF());

	VX a = K.transpose() * lambda_ - penalty_ * K.transpose() * F;

	/* 10 degree rotation tolerance, from degree to radians */
	qp_->solve(Q, a, D_, Dt_tol_, Dr_tol_, debug_);
}


void ADMMCut::UpdateLambda()
{
	// Recompute K(x_{k+1}) and F(x_{k+1})
	ptr_stiff_->CreateGlobalK(x_);
	SpMat K = *(ptr_stiff_->WeightedK());

	ptr_stiff_->CreateF(x_);
	VX F = *(ptr_stiff_->WeightedF());

	lambda_ = lambda_ + penalty_ * (K * D_ - F);
}


void ADMMCut::UpdateCut()
{
	// Convert previous continuous computed result to discrete 1-0 label
	for (int i = 0; i < M_; i++)
	{
		int e_id = ptr_dualgraph_->e_dual_id(i);
		WF_edge *e = ptr_frame_->GetEdge(i);
		int layer = e->Layer();
		if (e_id == -1)
		{
			e->SetLayer(layer + 1);
		}
		else
		{
			if (x_[e_id] >= 0.5)
			{
				x_[e_id] = 1;
			}
			else
			{
				x_[e_id] = 0;
				e->SetLayer(layer + 1);
			}
		}
	}

	// Update cut
	cutting_edge_.clear();
	VectorXd J(Md_);
	J = A_ * x_;
	for (int i = 0; i < Md_; i++)
	{
		if (J[i] == 1 || J[i] == -1)
		{
			int dual_u = ptr_dualgraph_->u(i);
			int dual_v = ptr_dualgraph_->v(i);
			int u = ptr_dualgraph_->e_orig_id(dual_u);
			int v = ptr_dualgraph_->e_orig_id(dual_v);
			if (x_[dual_u] == 1)
			{
				// u is the lower dual vertex of cutting-edge 
				cutting_edge_.push_back(u);
			}
			else
			{
				// v is the lower dual vertex of cutting-edge 
				cutting_edge_.push_back(v);
			}
		}
	}
}


bool ADMMCut::UpdateR(VX &x_prev, int count)
{
	double max_improv = 0;
	for (int i = 0; i < Nd_; i++)
	{
		/* if No significant improvment found */
		double improv = fabs(x_prev[i] - x_[i]) / x_prev[i];
		if (improv > max_improv)
		{
			max_improv = improv;
		}
	}

	cout << "---UpdateR Rew Check---" << endl;
	cout << "Reweighting Process No." << count - 1 << endl;
	cout << "Max Relative Improvement = " << max_improv << endl;
	cout << "---" << endl;

	vector<DualEdge*> dual_edge = *(GetDualEdgeList());

	for (int i = 0; i < Md_; i++)
	{
		int    u = ptr_dualgraph_->u(i);
		int    v = ptr_dualgraph_->v(i);
		double w = dual_edge[i]->w();

		r_(u, v) = 1.0 / (1e-5 + w * abs(x_[u] - x_[v]));
	}


	if (max_improv < 1e-2 || count > 20)
	{
		/* Exit Reweighting */
		return true;
	}
	else
	{
		return false;
	}

}


void ADMMCut::WriteWeight()
{
	string path = path_;

	string weight_path = path + "/point_weight.txt";
	string line_path = path + "/line.txt";

	FILE *fp = fopen(weight_path.c_str(), "w+");

	int N = ptr_frame_->SizeOfVertList();
	double minz = ptr_dualgraph_->minZ();
	double maxz = ptr_dualgraph_->maxZ();
	vector<double> ww(N);
	for (int i = 0; i < N; i++)
	{
		if (ptr_dualgraph_->isExistingVert(i))
		{
			if (ptr_frame_->GetDegree(i) > 1)
			{
				//double w = 1 - (verts[i]->Position().z() - minz) / (maxz - minz);
				double w = exp(-3 * pow((ptr_frame_->GetPosition(i).z() - minz) / (maxz - minz), 2));
				ww[i] = w;
			}
			else
			{
				ww[i] = 1.0;
			}
		}
	}

	for (int i = 0; i < N; i++)
	{
		point p = ptr_frame_->GetVert(i)->RenderPos();
		fprintf(fp, "%lf %lf %lf ", p.x(), p.y(), p.z());

		double r;
		double g;
		double b;

		if (ww[i] < 0.25)
		{
			r = 0.0;
			g = ww[i] * 4.0;
			b = 1.0;
		}
		else
			if (ww[i] < 0.5)
			{
				r = 0.0;
				g = 1.0;
				b = (0.5 - ww[i]) * 4.0;
			}
			else
				if (ww[i] < 0.75)
				{
					r = (ww[i] - 0.5) * 4.0;
					g = 1.0;
					b = 0.0;
				}
				else
				{
					r = 1.0;
					g = (1.0 - ww[i]) * 4.0;
					b = 0.0;
				}

		fprintf(fp, "%lf %lf %lf\n", r, g, b);
	}

	fclose(fp);
	//ptr_frame_->ExportLines(line_path.c_str());
}


void ADMMCut::WriteStiffness(string offset, string rotation)
{
	string path = path_;

	string offset_path = path + "/" + offset;
	string rotation_path = path + "/" + rotation;

	vector<FILE*> fp(2);
	fp[0] = fopen(offset_path.c_str(), "w+");
	fp[1] = fopen(rotation_path.c_str(), "w+");

	fprintf(fp[0], "#offset colormap#\r\n");
	fprintf(fp[1], "#rotation colormap#\r\n",
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

	int N = ptr_frame_->SizeOfVertList();
	vector<vector<double>> ss(N);
	for (int i = 0; i < N; i++)
	{
		ss[i].resize(2);
		if (ptr_dualgraph_->isExistingVert(i) && !ptr_frame_->isFixed(i))
		{
			int j = ptr_dualgraph_->v_dual_id(i);

			VX offset(3);
			VX rotation(3);
			for (int k = 0; k < 3; k++)
			{
				offset[k] = D_[j * 6 + k];
				rotation[k] = D_[j * 6 + k + 3];
			}

			if (offset.norm() >= Dt_tol_ || rotation.norm() >= Dr_tol_)
			{
				printf(".............. %lf %lf\n", offset.norm(), rotation.norm());
				getchar();
			}
			ss[i][0] = offset.norm() / Dt_tol_;
			ss[i][1] = rotation.norm() / Dr_tol_;
		}
		else
		{
			ss[i][0] = 0.0;
			ss[i][1] = 0.0;
		}

		//if (ptr_dualgraph_->isExistingVert(i))
		{
			point p = ptr_frame_->GetVert(i)->RenderPos();
			for (int j = 0; j < 2; j++)
			{
				fprintf(fp[j], "%lf %lf %lf ", p.x(), p.y(), p.z());

				double r;
				double g;
				double b;

				if (ss[i][j] < 0.25)
				{
					r = 0.0;
					g = ss[i][j] * 4.0;
					b = 1.0;
				}
				else
					if (ss[i][j] < 0.5)
					{
						r = 0.0;
						g = 1.0;
						b = (0.5 - ss[i][j]) * 4.0;
					}
					else
						if (ss[i][j] < 0.75)
						{
							r = (ss[i][j] - 0.5) * 4.0;
							g = 1.0;
							b = 0.0;
						}
						else
						{
							r = 1.0;
							g = (1.0 - ss[i][j]) * 4.0;
							b = 0.0;
						}

				fprintf(fp[j], "%lf %lf %lf\r\n", r, g, b);
			}
		}
	}

	fclose(fp[0]);
	fclose(fp[1]);
}


void ADMMCut::Debug()
{
	int cut_count = 0;
	SetStartingPoints(cut_count);

	ptr_stiff_->CalculateD(D_, x_, 1, 1, 0);
}
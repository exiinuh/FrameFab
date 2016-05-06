#include "ADMMCut.h"


ADMMCut::ADMMCut()
	:penalty_(0), D_tol_(0), pri_tol_(0), dual_tol_(0)
{
	// This default construction function should never be run
	// We need a mesh to begin with

	ptr_dualgraph_ = NULL;
	ptr_stiffness_ = NULL;
	ptr_collision_ = NULL;
}


ADMMCut::ADMMCut(
	DualGraph			*ptr_dualgraph,
	QuadricCollision	*ptr_collision,
	Stiffness			*ptr_stiffness,
	FiberPrintPARM		*ptr_parm,
	char				*ptr_path
	)
{
	ptr_frame_ = ptr_dualgraph->ptr_frame_;
	ptr_dualgraph_ = ptr_dualgraph;
	ptr_stiffness_ = ptr_stiffness;
	ptr_collision_ = ptr_collision;
	ptr_path_ = ptr_path;

	ptr_qp_ = NULL;

	D_tol_ = ptr_parm->ADMM_D_tol_;
	penalty_ = ptr_parm->penalty_;
	pri_tol_ = ptr_parm->pri_tol_;
	dual_tol_ = ptr_parm->dual_tol_;

	debug_ = false;
}


ADMMCut::~ADMMCut()
{
	delete ptr_qp_;
	ptr_qp_ = NULL;
}


void ADMMCut::MakeLayers()
{
	ADMM_cut_.Start();

	cut_round_ = 0;

	// Initial Cutting Edge Setting
	InitState();
	InitCollisionWeight();

	vector<double> cut_energy;
	vector<double> res_energy;
	do
	{
		reweight_round_ = 0;

		/* Recreate dual graph at the beginning of each cut */
		SetStartingPoints();

#ifdef ZHANG_SAID_L
#else
		CreateA();
#endif

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
		ptr_stiffness_->CreateGlobalK(&x_);
		ptr_stiffness_->CreateF(&x_);
		SpMat K_init = *(ptr_stiffness_->WeightedK());
		VX	  F_init = *(ptr_stiffness_->WeightedF());


		cout << "****************************************" << endl;
		cout << "ADMMCut Round : " << cut_round_ << endl;
		cout << "---------------------------------" << endl;

		VX x_prev;
		VX D_prev;

		/* Output energy list for reweighting process in a single
		graph cut problem, energy.size() = number of reweighting
		process performed.
		cut energy = |A_ * x_| = sum_{e_ij} w_ij * |x_i - x_j|
		res energy = (K(x)D - F(x)).norm()
		*/

		do
		{
			penalty_ = 1000;
			ADMM_round_ = 0;
			/* Reweighting loop for cut */
			x_prev = x_;

#ifdef ZHANG_SAID_L
			CreateL();
#else
			CreateC();
#endif


			do
			{
				cout << "ADMMCut Round: " << cut_round_ << ", reweight iteration:" << reweight_round_
					<< ", ADMM " << ADMM_round_ << " iteration." << endl;

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
				ptr_stiffness_->CreateGlobalK(&x_);
				ptr_stiffness_->CreateF(&x_);
				SpMat K_new = *(ptr_stiffness_->WeightedK());
				VX    F_new = *(ptr_stiffness_->WeightedF());

				dual_res_ = penalty_ * (D_prev - D_).transpose() * K_new.transpose() * Q_prev
					+ lambda_.transpose() * (Q_prev - Q_new);
				primal_res_ = K_new * D_ - F_new;

				/*-------------------Screenplay-------------------*/
				double new_cut_energy = x_.dot(L_ * x_);

				cout << "new quadratic func value record: " << new_cut_energy << endl;
				cout << "dual_residual : " << dual_res_.norm() << endl;
				cout << "primal_residual(KD-F) : " << primal_res_.norm() << endl;

				cout << "---------------------" << endl;
				ADMM_round_++;
			} while (!TerminationCriteria());

			/* One reweighting process ended! */
			/* Output energy and residual */

			/*-------------------Screenplay-------------------*/
			double res_tmp = primal_res_.norm();
			cout << "Cut " << cut_round_ << " Reweight " << reweight_round_ << " completed." << endl;
			//cout << "Cut Energy :" << energy << endl;
			cout << "Res Energy :" << res_tmp << endl;
			cout << "<-<-<-<-<-<-<-<-<-<-<-<-<-<-<-<-<-" << endl;

			//cut_energy.push_back(energy);
			res_energy.push_back(res_tmp);

			///* write x distribution to a file */
			string str_x = "Cut_" + to_string(cut_round_) + "_Rew_" + to_string(reweight_round_) + "_x";
			Statistics tmp_x(str_x, x_);
			tmp_x.GenerateVectorFile();

			reweight_round_++;
		} while (!UpdateR(x_prev));

		string str_eR = "Cut_" + to_string(cut_round_) + "_Res_Energy";
		Statistics s_eR(str_eR, res_energy);
		s_eR.GenerateStdVecFile();

		/* Update New Cut information to Rendering (layer_label_) */

		UpdateCut();

		fprintf(stdout, "ADMMCut No.%d process is Finished!\n", cut_round_);
		cut_round_++;

	} while (!CheckLabel());

	ptr_frame_->Unify();

	fprintf(stdout, "All done!\n");

	ADMM_cut_.Stop();
}


void ADMMCut::InitState()
{
	ptr_dualgraph_->Dualization();
	Nd_ = ptr_dualgraph_->SizeOfVertList();
	Md_ = ptr_dualgraph_->SizeOfEdgeList();
	Fd_ = ptr_dualgraph_->SizeOfFaceList();
	Ns_ = ptr_dualgraph_->SizeOfFreeFace();
	N_ = ptr_frame_->SizeOfVertList();
	M_ = ptr_frame_->SizeOfEdgeList();

	// set termination tolerance 
	stop_n_ = floor(N_ / 5);

	ptr_qp_ = QPFactory::make(static_cast<QPFactory::QPType>(1));

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


void ADMMCut::InitCollisionWeight()
{
	init_collision_.Start();

	int halfM = M_ / 2;
	weight_.resize(halfM, halfM);
	vector<Triplet<double>> weight_list;

	for (int i = 0; i < Md_; i++)
	{
		int orig_u = ptr_dualgraph_->e_orig_id(ptr_dualgraph_->u(i));
		int orig_v = ptr_dualgraph_->e_orig_id(ptr_dualgraph_->v(i));
		WF_edge *e1 = ptr_frame_->GetEdge(orig_u);
		WF_edge *e2 = ptr_frame_->GetEdge(orig_v);
		vector<lld> tmp(3);
		double Fij, Fji;
		double tmp_range;
		double tmp_weight;
		double tmp_height;

		tmp_range = ptr_dualgraph_->Weight(i);
		tmp_height = exp(-6 * tmp_range * tmp_range);

		ptr_collision_->DetectCollision(e1, e2, tmp);
		Fji = ptr_collision_->ColFreeAngle(tmp) * 1.0 / ptr_collision_->Divide();

		ptr_collision_->DetectCollision(e2, e1, tmp);
		Fij = ptr_collision_->ColFreeAngle(tmp) * 1.0 / ptr_collision_->Divide();

		tmp_range = max(Fij - Fji, 0.0);
		tmp_weight = exp(-6 * tmp_range * tmp_range) * tmp_height;
		if (tmp_weight > SPT_EPS)
		{
			weight_list.push_back(Triplet<double>(orig_u / 2, orig_v / 2, tmp_weight));
		}

		tmp_range = max(Fji - Fij, 0.0);
		tmp_weight = exp(-6 * tmp_range * tmp_range) * tmp_height;
		if (tmp_weight > SPT_EPS)
		{
			weight_list.push_back(Triplet<double>(orig_v / 2, orig_u / 2, tmp_weight));
		}
	}

	weight_.setFromTriplets(weight_list.begin(), weight_list.end());

	init_collision_.Stop();
}


void ADMMCut::SetStartingPoints()
{
	if (cut_round_ == 0)
	{
		Nd_w_ = ptr_dualgraph_->SizeOfVertList();
	}
	else
	{
		ptr_dualgraph_->UpdateDualization(&x_);
		Nd_ = ptr_dualgraph_->SizeOfVertList();
		Md_ = ptr_dualgraph_->SizeOfEdgeList();
		Fd_ = ptr_dualgraph_->SizeOfFaceList();
		Ns_ = ptr_dualgraph_->SizeOfFreeFace();
	}

	ptr_stiffness_->Init();

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

	//ptr_stiffness_->Debug();
}


void ADMMCut::SetBoundary()
{
	set_bound_.Start();

	ptr_stiffness_->CalculateD(D_, &x_);

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

		WF_edge *e = ptr_frame_->GetEdge(cutting_edge_[i]);
		int u = e->pvert_->ID();
		int v = e->ppair_->pvert_->ID();
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

	set_bound_.Stop();
}


void ADMMCut::CreateA()
{
	create_a_.Start();

	A_.resize(2 * Md_, Nd_);
	vector<Triplet<double>> A_list;
	for (int i = 0; i < Md_; i++)
	{
		int dual_u = ptr_dualgraph_->u(i);
		int dual_v = ptr_dualgraph_->v(i);
		A_list.push_back(Triplet<double>(i, dual_u, 1));
		A_list.push_back(Triplet<double>(i, dual_v, -1));
		A_list.push_back(Triplet<double>(i + Md_, dual_v, 1));
		A_list.push_back(Triplet<double>(i + Md_, dual_u, -1));
	}

	A_.setFromTriplets(A_list.begin(), A_list.end());

	create_a_.Stop();
}


void ADMMCut::CreateC()
{
	create_c_.Start();

	C_.resize(2 * Md_, 2 * Md_);
	vector<Triplet<double>> C_list;

	for (int i = 0; i < Md_; i++)
	{
		int dual_u = ptr_dualgraph_->u(i);
		int dual_v = ptr_dualgraph_->v(i);
		int u = ptr_dualgraph_->e_orig_id(dual_u) / 2;
		int v = ptr_dualgraph_->e_orig_id(dual_v) / 2;

		C_list.push_back(Triplet<double>(i, i, 
			pow(weight_.coeff(u, v), 2) * r_(dual_u, dual_v)));
		C_list.push_back(Triplet<double>(i + Md_, i + Md_, 
			pow(weight_.coeff(v, u), 2) * r_(dual_v, dual_u)));
	}

	C_.setFromTriplets(C_list.begin(), C_list.end());

	L_ = A_.transpose() * C_ * A_ / 2;

	string path = "C:/Users/DELL/Desktop/result";
	char cut_id[30];
	sprintf(cut_id, "%d", cut_round_);
	char reweight[30];
	sprintf(reweight, "%d", reweight_round_);

	string file = path + "/" + "L_AC_" + cut_id + "_" + reweight + ".txt";
	FILE *fp = fopen(file.c_str(), "w");
	for (int i = 0; i < Nd_; i++)
	{
		for (int j = 0; j < Nd_; j++)
		{
			fprintf(fp, "%lf ", L_.coeff(i, j));
		}
		fprintf(fp, "\n");
	}
	fclose(fp);

	string file2 = path + "/" + "L_AC_r_" + cut_id + "_" + reweight + ".txt";
	FILE *fp1 = fopen(file2.c_str(), "w");
	for (int i = 0; i < Nd_; i++)
	{
		for (int j = 0; j < Nd_; j++)
		{
			fprintf(fp1, "%lf ", r_(i, j));
		}
		fprintf(fp1, "\n");
	}

	fclose(fp1);

	create_c_.Stop();
}


void ADMMCut::CreateL()
{
	L_.resize(Nd_, Nd_);
	vector<Triplet<double>> L_list;

	for (int i = 0; i < Md_; i++)
	{
		int dual_u = ptr_dualgraph_->u(i);
		int dual_v = ptr_dualgraph_->v(i);

		int u = ptr_dualgraph_->e_orig_id(dual_u) / 2;
		int v = ptr_dualgraph_->e_orig_id(dual_v) / 2;

		L_list.push_back(Triplet<double>(dual_u, dual_u, 
			pow(weight_.coeff(u, v), 2) * r_(dual_u, dual_v)));
		L_list.push_back(Triplet<double>(dual_v, dual_v, 
			pow(weight_.coeff(v, u), 2) * r_(dual_v, dual_u)));
		L_list.push_back(Triplet<double>(dual_u, dual_v, 
			-pow(weight_.coeff(u, v), 2) * r_(dual_u, dual_v)));
		L_list.push_back(Triplet<double>(dual_v, dual_u, 
			-pow(weight_.coeff(v, u), 2) * r_(dual_v, dual_u)));
	}

	L_.setFromTriplets(L_list.begin(), L_list.end());

	//string path = "C:/Users/DELL/Desktop/result";
	//char cut_id[30];
	//sprintf(cut_id, "%d", cut_round_);
	//char reweight[30];
	//sprintf(reweight, "%d", reweight_round_);

	//string file = path + "/" + "L_" + cut_id + "_" + reweight + ".txt";
	//FILE *fp = fopen(file.c_str(), "w");
	//for (int i = 0; i < Nd_; i++)
	//{
	//	for (int j = 0; j < Nd_; j++)
	//	{
	//		fprintf(fp, "%lf ", L_.coeff(i, j));
	//	}
	//	fprintf(fp, "\n");
	//}
	//fclose(fp);

	//string file2 = path + "/" + "L_r_" + cut_id + "_" + reweight + ".txt";
	//FILE *fp1 = fopen(file2.c_str(), "w");
	//for (int i = 0; i < Nd_; i++)
	//{
	//	for (int j = 0; j < Nd_; j++)
	//	{
	//		fprintf(fp1, "%lf ", r_(i, j));
	//	}
	//	fprintf(fp1, "\n");
	//}

	//fclose(fp1);
}


void ADMMCut::CalculateX()
{
	cal_x_.Start();

	// Construct Hessian Matrix for D-Qp problem
	SpMat Q;
	CalculateQ(D_, Q);

	SpMat H2 = Q.transpose() * Q;
	SpMat H = 2 * L_ + penalty_ * H2;

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

	cal_qp_.Start();
	ptr_qp_->solve(H, a_, A, b, W_, d_, lb, ub, x_, NULL, NULL, debug_);
	cal_qp_.Stop();
	cal_x_.Stop();
}


void ADMMCut::CalculateQ(const VX _D, SpMat &Q)
{
	cal_q_.Start();

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

				MX eKuu = ptr_stiffness_->eKv(edge->ID());
				MX eKeu = ptr_stiffness_->eKe(edge->ID());
				VX Fe = ptr_stiffness_->Fe(edge->ID());
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

	cal_q_.Stop();
}


void ADMMCut::CalculateD()
{
	cal_d_.Start();

	// Construct Hessian Matrix for D-Qp problem
	// Here, K is continuous-x weighted
	ptr_stiffness_->CreateGlobalK(&x_);
	SpMat K = *(ptr_stiffness_->WeightedK());
	SpMat Q = penalty_ * K.transpose() * K;

	// Construct Linear coefficient for D-Qp problem
	ptr_stiffness_->CreateF(&x_);
	VX F = *(ptr_stiffness_->WeightedF());

	VX a = K.transpose() * lambda_ - penalty_ * K.transpose() * F;

	/* 10 degree rotation tolerance, from degree to radians */
	cal_qp_.Start();
	ptr_qp_->solve(Q, a, D_, D_tol_, debug_);
	cal_qp_.Stop();
	cal_d_.Stop();
}


void ADMMCut::UpdateLambda()
{
	update_lambda_.Start();

	// Recompute K(x_{k+1}) and F(x_{k+1})
	ptr_stiffness_->CreateGlobalK(&x_);
	SpMat K = *(ptr_stiffness_->WeightedK());

	ptr_stiffness_->CreateF(&x_);
	VX F = *(ptr_stiffness_->WeightedF());

	lambda_ = lambda_ + penalty_ * (K * D_ - F);

	update_lambda_.Stop();
}


void ADMMCut::UpdateCut()
{
	update_cut_.Start();

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
	for (int i = 0; i < Md_; i++)
	{
		int dual_u = ptr_dualgraph_->u(i);
		int dual_v = ptr_dualgraph_->v(i);
		if (x_[dual_u] != x_[dual_v])
		{
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

	update_cut_.Stop();
}


bool ADMMCut::UpdateR(VX &x_prev)
{
	update_r_.Start();

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
	cout << "Reweighting Process No." << reweight_round_ - 1 << endl;
	cout << "Max Relative Improvement = " << max_improv << endl;
	cout << "---" << endl;

	for (int i = 0; i < Md_; i++)
	{
		int dual_u = ptr_dualgraph_->u(i);
		int dual_v = ptr_dualgraph_->v(i);
		int u = ptr_dualgraph_->e_orig_id(dual_u) / 2;
		int v = ptr_dualgraph_->e_orig_id(dual_v) / 2;

		r_(dual_u, dual_v) = 1.0 / 
			(1e-5 + weight_.coeff(u, v) * abs(x_[dual_u] - x_[dual_v]));
		r_(dual_v, dual_u) = 1.0 / 
			(1e-5 + weight_.coeff(v, u) * abs(x_[dual_u] - x_[dual_v]));
	}

	update_r_.Stop();

	if (max_improv < 1e-2 || reweight_round_ > 20)
	{
		/* Exit Reweighting */
		return true;
	}
	else
	{
		return false;
	}
}


bool ADMMCut::CheckLabel()
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
	cout << "ADMMCut Round : " << cut_round_ << endl;
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


bool ADMMCut::TerminationCriteria()
{
	if (ADMM_round_ >= 20)
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


void ADMMCut::PrintOutTimer()
{
	printf("***ADMMCut timer result:\n");
	ADMM_cut_.Print("ADMMCut:");
	init_collision_.Print("InitCollisionWeight:");
	set_bound_.Print("SetBoundary:");
	create_l_.Print("CreateL:");
	cal_x_.Print("CalculateX:");
	cal_q_.Print("CalculateQ:");
	cal_d_.Print("CalculateD:");
	cal_qp_.Print("qp:");
	update_lambda_.Print("UpdateLambda:");
	update_cut_.Print("UpdateCut:");
	update_r_.Print("UpdateR:");
}


void ADMMCut::WriteWeight()
{
	//string path = path_;

	//string weight_path = path + "/point_weight.txt";
	//string line_path = path + "/line.txt";

	//FILE *fp = fopen(weight_path.c_str(), "w+");

	//int N = ptr_frame_->SizeOfVertList();
	//double minz = ptr_dualgraph_->minZ();
	//double maxz = ptr_dualgraph_->maxZ();
	//vector<double> ww(N);
	//for (int i = 0; i < N; i++)
	//{
	//	if (ptr_dualgraph_->isExistingVert(i))
	//	{
	//		if (ptr_frame_->GetDegree(i) > 1)
	//		{
	//			//double w = 1 - (verts[i]->Position().z() - minz) / (maxz - minz);
	//			double w = exp(-3 * pow((ptr_frame_->GetPosition(i).z() - minz) / (maxz - minz), 2));
	//			ww[i] = w;
	//		}
	//		else
	//		{
	//			ww[i] = 1.0;
	//		}
	//	}
	//}

	//for (int i = 0; i < N; i++)
	//{
	//	point p = ptr_frame_->GetVert(i)->RenderPos();
	//	fprintf(fp, "%lf %lf %lf ", p.x(), p.y(), p.z());

	//	double r;
	//	double g;
	//	double b;

	//	if (ww[i] < 0.25)
	//	{
	//		r = 0.0;
	//		g = ww[i] * 4.0;
	//		b = 1.0;
	//	}
	//	else
	//		if (ww[i] < 0.5)
	//		{
	//			r = 0.0;
	//			g = 1.0;
	//			b = (0.5 - ww[i]) * 4.0;
	//		}
	//		else
	//			if (ww[i] < 0.75)
	//			{
	//				r = (ww[i] - 0.5) * 4.0;
	//				g = 1.0;
	//				b = 0.0;
	//			}
	//			else
	//			{
	//				r = 1.0;
	//				g = (1.0 - ww[i]) * 4.0;
	//				b = 0.0;
	//			}

	//	fprintf(fp, "%lf %lf %lf\n", r, g, b);
	//}

	//fclose(fp);
	////ptr_frame_->ExportLines(line_path.c_str());
}


void ADMMCut::WriteStiffness(string offset, string rotation)
{
	//string path = path_;

	//string offset_path = path + "/" + offset;
	//string rotation_path = path + "/" + rotation;

	//vector<FILE*> fp(2);
	//fp[0] = fopen(offset_path.c_str(), "w+");
	//fp[1] = fopen(rotation_path.c_str(), "w+");

	//fprintf(fp[0], "#offset colormap#\r\n");
	//fprintf(fp[1], "#rotation colormap#\r\n",
	//	0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

	//int N = ptr_frame_->SizeOfVertList();
	//vector<vector<double>> ss(N);
	//for (int i = 0; i < N; i++)
	//{
	//	ss[i].resize(2);
	//	if (ptr_dualgraph_->isExistingVert(i) && !ptr_frame_->isFixed(i))
	//	{
	//		int j = ptr_dualgraph_->v_dual_id(i);

	//		VX offset(3);
	//		for (int k = 0; k < 3; k++)
	//		{
	//			offset[k] = D_[j * 6 + k];
	//		}

	//		if (offset.norm() >= Dt_tol_)
	//		{
	//			printf(".............. %lf\n", offset.norm());
	//			getchar();
	//		}
	//		ss[i][0] = offset.norm() / Dt_tol_;
	//	}
	//	else
	//	{
	//		ss[i][0] = 0.0;
	//		ss[i][1] = 0.0;
	//	}

	//	//if (ptr_dualgraph_->isExistingVert(i))
	//	{
	//		point p = ptr_frame_->GetVert(i)->RenderPos();
	//		for (int j = 0; j < 2; j++)
	//		{
	//			fprintf(fp[j], "%lf %lf %lf ", p.x(), p.y(), p.z());

	//			double r;
	//			double g;
	//			double b;

	//			if (ss[i][j] < 0.25)
	//			{
	//				r = 0.0;
	//				g = ss[i][j] * 4.0;
	//				b = 1.0;
	//			}
	//			else
	//				if (ss[i][j] < 0.5)
	//				{
	//					r = 0.0;
	//					g = 1.0;
	//					b = (0.5 - ss[i][j]) * 4.0;
	//				}
	//				else
	//					if (ss[i][j] < 0.75)
	//					{
	//						r = (ss[i][j] - 0.5) * 4.0;
	//						g = 1.0;
	//						b = 0.0;
	//					}
	//					else
	//					{
	//						r = 1.0;
	//						g = (1.0 - ss[i][j]) * 4.0;
	//						b = 0.0;
	//					}

	//			fprintf(fp[j], "%lf %lf %lf\r\n", r, g, b);
	//		}
	//	}
	//}

	//fclose(fp[0]);
	//fclose(fp[1]);
}


void ADMMCut::Debug()
{
	int cut_count = 0;
	SetStartingPoints();
	ptr_stiffness_->CalculateD(D_, x_, 0, false, false, false);
	int temp[24] = { 82, 66, 76, 62, 58, 168, 64, 60, 78, 192, 80, 98, 110, 96, 196, 190, 56, 4, 2, 92, 94, 54, 8, 14 };
	int temp_2[42] = { 82, 66, 76, 62, 58, 168, 64, 60, 78, 192, 80, 98, 110, 96, 196, 190, 56, 4, 2, 92, 94, 54,
		8, 84, 72, 74, 194, 68, 70, 90, 210, 180, 184, 102, 108, 188, 100, 178, 106, 186, 86, 14
	};




}
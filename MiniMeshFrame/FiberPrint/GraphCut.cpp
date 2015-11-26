#include "GraphCut.h"


GraphCut::GraphCut()
		 :debug_(false), penalty_(0), D_tol_(0), pri_tol_(0), dual_tol_(0)
{
	// This default construction function should never be run
	// We need a mesh to begin with
}


GraphCut::GraphCut(WireFrame *ptr_frame)
		 :debug_(false), penalty_(10e2), D_tol_(0.1), pri_tol_(10e-3), dual_tol_(10e-3)
{
	ptr_frame_ = ptr_frame;
	ptr_dualgraph_ = new DualGraph(ptr_frame_);
	ptr_stiff_ = new Stiffness(ptr_dualgraph_);
}


GraphCut::GraphCut(WireFrame *ptr_frame, FiberPrintPARM *ptr_parm)
		 :debug_(false)
{
	ptr_frame_ = ptr_frame;
	ptr_dualgraph_ = new DualGraph(ptr_frame_);
	ptr_stiff_ = new Stiffness(ptr_dualgraph_, ptr_parm);

	penalty_ = ptr_parm->penalty_;
	D_tol_ = ptr_parm->D_tol_;
	pri_tol_ = ptr_parm->pri_tol_;
	dual_tol_ = ptr_parm->dual_tol_;
}


GraphCut::~GraphCut()
{
	delete ptr_dualgraph_;
	ptr_dualgraph_ = NULL;

	delete ptr_stiff_;
	ptr_stiff_ = NULL;

	delete qp_;
	qp_ = NULL;
}


void GraphCut::InitState()
{
	N_ = ptr_frame_->SizeOfVertList();
	M_ = ptr_frame_->SizeOfEdgeList();

	/* set termination tolerance */
	stop_n_ = floor(N_ / 5);						

	qp_ = QPFactory::make(static_cast<QPFactory::QPType>(1));

	layer_label_.resize(M_);
	fill(layer_label_.begin(), layer_label_.end(), 0);

	/* Find nodes with max height */
	int max_id = 0;
	for (int i = 1; i < N_; i++)
	{
		if (ptr_frame_->GetPosition(i).z() > ptr_frame_->GetPosition(max_id).z())
		{
			max_id = i;
		}
	}

	/* upper dual id */
	WF_edge *edge = ptr_frame_->GetNeighborEdge(max_id);					
	while (edge != NULL)
	{
		cutting_edge_.push_back(edge->ID()); 
		edge = edge->pnext_;
	}


	cout << "penalty : " << penalty_ << endl;
	cout << "primal tolerance : " << pri_tol_ << endl;
	cout << "dual tolerance : " << dual_tol_ << endl;
	cout << "GraphCut Start" << endl;
	cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
}


void GraphCut::SetStartingPoints(int count)
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

	x_.resize(Nd_);
	D_.resize(3 * Fd_);
	lambda_.resize(3 * Fd_);
	a_.resize(Nd_);

	// Set all label x = 1 and calculate KD = F to obtain inital D_0	
	x_.setOnes();
	D_.setZero();
	lambda_.setZero();
	a_.setZero();

	//ptr_stiff_->Debug();
}


void GraphCut::CreateAandC()
{
	vector<DualEdge*> dual_edge = *(GetDualEdgeList());

	A_.resize(Md_, Nd_); 
	vector<Triplet<double>> A_list;
	for (int i = 0; i < Md_; i++)
	{
		A_list.push_back(Triplet<double>(i, dual_edge[i]->u(), 1));
		A_list.push_back(Triplet<double>(i, dual_edge[i]->v(), -1));
	}
	A_.setFromTriplets(A_list.begin(), A_list.end());

	C_.resize(Md_, Md_);
	vector<Triplet<double>> C_list;
	for (int i = 0; i < Md_; i++)
	{
		C_list.push_back(Triplet<double>(i, i, pow(dual_edge[i]->w(), 2)));
	}
	C_.setFromTriplets(C_list.begin(), C_list.end());
	
	Statistics s_C("C", C_);
	s_C.GenerateSpFile();

	H1_ = SpMat(Nd_, Nd_);
	H1_ = A_.transpose() * C_ * A_;
}


void GraphCut::SetBoundary(VX &d, SpMat &W)
{
	// Set lower boundary and upper boundary
	// equality constraints W*x = d
	// Identify Base nodes(1) and Upper Nodes(2)
	// Here we just take the edge with biggest height
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

	d.setZero();
	vector<Eigen::Triplet<double>> W_list;
	for (int e_id = 0; e_id < Nd_; e_id++)
	{
		if (bound[e_id] == 1)
		{
			d[e_id] = 1;
			W_list.push_back(Eigen::Triplet<double>(e_id, e_id, 1));
		}
		if (bound[e_id] == 2)
		{
			d[e_id] = 0;
			W_list.push_back(Eigen::Triplet<double>(e_id, e_id, 1));
		}
	}
	W.setFromTriplets(W_list.begin(), W_list.end());
}


bool GraphCut::CheckLabel(int iter_count)
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
	cout << "GRAPHCUT REPORT" << endl;
	cout << "GraphCut Round : " << iter_count << endl;
	cout << "Lower Set edge number : " << l << "\\ " << Nd_w_ << " (Whole dual graph Nd)" << endl;
	cout << "Lower Set percentage  : " << double(l) / double(Nd_w_) * 100 << "%" << endl;
	cout << "--------------------------------------------" << endl;

	if (l < 20)
	{
		return true;
	}
	else
	{
		return false;
	}
}


bool GraphCut::TerminationCriteria()
{
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

void GraphCut::MakeLayers()
{
	InitState();

	vector<double> record;
	int cut_count = 0;
	do
	{
		SetStartingPoints(cut_count);
		CreateAandC();

		VX d(Nd_);
		SpMat W(Nd_, Nd_);
		SetBoundary(d, W);

		//ptr_stiff_->Debug();
		ptr_stiff_->CalculateD(&D_, &x_);

		//ptr_stiff_->CalculateD(&D_, &x_);

		double old_energy = x_.dot((H1_)*x_);
		cout << "****************************************" << endl;
		cout << "GraphCut Round : " << cut_count << endl;
		cout << "Initial energy before entering ADMM: " << old_energy << endl;

		int ADMM_count = 0;
		record.clear();

		cout << "---------------------------------" << endl;
		do
		{
			cout << "GraphCut Round: " << cut_count << ", ADMM " << ADMM_count << " iteration." << endl;
			
			string str = "cut_" + to_string(cut_count) + "_iter_" + to_string(ADMM_count) + "_x";
			Statistics s_x(str, x_);
			s_x.GenerateVectorFile();

			VX x_prev = x_;
			CalculateX(d, W);

			VX D_prev = D_;
			CalculateD();

			UpdateLambda();
			
			//Residual calculation
			SpMat Q_prev;
			SpMat Q_new;
			CalculateQ(D_prev, Q_prev);
			CalculateQ(D_, Q_new);

			//ptr_stiff_->CreateK(&x_);
			SpMat K_new = *(ptr_stiff_->WeightedK());

			/*dual_res_ = penalty_ * (x_ - x_prev).transpose() * Q_prev.transpose() * Q_prev
				+ lambda_.transpose() * (Q_prev - Q_new);*/
			dual_res_ = penalty_ * (D_prev - D_).transpose() * K_new.transpose() * Q_prev
				+ lambda_.transpose() * (Q_prev - Q_new);
			primal_res_ = Q_new * x_;


			double obj_func = x_.dot(H1_ * x_);
			record.push_back(obj_func);

			cout << "new energy func value record: " << obj_func << endl;
			cout << "dual_residual : " << dual_res_.norm() << endl;
			cout << "primal_residual : " << primal_res_.norm() << endl;

			for (int i = 0; i < record.size(); i++)
			{
				cout << record[i] << " << ";
			}
			putchar('\n');
			
			ADMM_count++;
			if (ADMM_count >= 20)
			{
				break;
			}
		} while (!TerminationCriteria());

		UpdateCut();													// Update New Cut information to Rendering (layer_label_)
		
		printf("One iteration done!\n");
		cut_count++;
	} while (!CheckLabel(cut_count));

	ptr_dualgraph_->Dualization();										// for sequence analyzer
	
	printf("All done!\n");

}


void GraphCut::CalculateX(VX &d, SpMat &W)
{
	// Construct Hessian Matrix for D-Qp problem
	SpMat Q;
	CalculateQ(D_, Q);

	SpMat H2 = Q.transpose() * Q;
	SpMat H = 2 * H1_ + penalty_ * H2;

	// Construct Linear coefficient for x-Qp problem
	a_ = Q.transpose() * lambda_;
	
	// Inequality constraints A*x <= b
	SpMat A(3 * Fd_, 3 * Fd_);
	A.setZero();
	VX b(3 * Fd_);
	b.setZero();

	// Variable constraints x >= lb, x <= ub
	VX lb(Nd_), ub(Nd_);													
	lb.setZero();
	ub.setOnes();

	qp_->solve(H, a_, A, b, W, d, lb, ub, x_, NULL, NULL, debug_);
}


void GraphCut::CalculateQ(const VX _D, SpMat &Q)
{
	// Construct Hessian Matrix for D-Qp problem
	Q.resize(6 * Fd_, Nd_);
	vector<Eigen::Triplet<double>> Q_list;
	for (int i = 0; i < Fd_; i++)
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

				//MX eKuu = ptr_stiff_->Me(edge->ID());
				//MX eKeu = ptr_stiff_->Me(edge->ID());
				//VX Fe = ptr_stiff_->Fe(edge->ID());
				//VX Di()
				//VX Di(_D[3 * i], _D[3 * i + 1], _D[3 * i + 2]);
				//VX Dj(_D[3 * j], _D[3 * j + 1], _D[3 * j + 2]);
				//VX Gamma = M * (Dj - Di) - 0.5 * Fe;

				//Q_list.push_back(Triplet<double>(3 * i, e_id, Gamma[0]));
				//Q_list.push_back(Triplet<double>(3 * i + 1, e_id, Gamma[1]));
				//Q_list.push_back(Triplet<double>(3 * i + 2, e_id, Gamma[2]));
			}

			edge = edge->pnext_;
		}
	}

	Q.setFromTriplets(Q_list.begin(), Q_list.end());
}


void GraphCut::CalculateD()
{
	// Construct Hessian Matrix for D-Qp problem
	//ptr_stiff_->CreateK(&x_);
	//SpMat K = *(ptr_stiff_->WeightedK());
	//SpMat Q = penalty_ * K.transpose() * K;

	//// Construct Linear coefficient for D-Qp problem
	//ptr_stiff_->CreateFv(&x_);
	//VX F = *(ptr_stiff_->WeightedF());

	//VX a = K.transpose() * lambda_ - penalty_ * K.transpose() * F;
	//
	//// Inequality constraints A*D <= b
	//SpMat A(3 * Fd_, 3 * Fd_);
	//A.setIdentity();	
	//VX b(3 * Fd_);
	//b.setOnes();
	//b = b * D_tol_;

	//// Equality constraints C*D = d
	//SpMat C(3 * Fd_, 3 * Fd_);
	//C.setZero();
	//VX d(3 * Fd_);
	//d.setZero();

	//// Variable constraints D >= lb, D <= ub
	//VX lb(3 * Fd_), ub(3 * Fd_);
	//lb.setOnes();
	//ub.setOnes();
	//lb = lb * (-MYINF);
	//ub = ub * MYINF;

	//qp_->solve(Q, a, A, b, C, d, lb, ub, D_, NULL, NULL, debug_);	
}


void GraphCut::UpdateLambda()
{
	//// Recompute K(x_{k+1}) and F(x_{k+1})
	//ptr_stiff_->CreateK(&x_);
	//SpMat K = *(ptr_stiff_->WeightedK());

	//ptr_stiff_->CreateFv(&x_);
	//VX F = *(ptr_stiff_->WeightedF());

	//lambda_ = lambda_ + penalty_ * (K * D_ - F);
}


void GraphCut::UpdateCut()
{
	// Convert previous continuous computed result to discrete 1-0 label
	for (int i = 0; i < M_; i++)
	{
		int e_id = ptr_dualgraph_->e_dual_id(i);
		if (e_id == -1)
		{
			layer_label_[i] ++;
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
				layer_label_[i] ++;
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

void	GraphCut::Debug()
{
	InitState();

	int cut_count = 0;
	SetStartingPoints(cut_count);

	VX d(Nd_);
	SpMat W(Nd_, Nd_);
	SetBoundary(d, W);

	//ptr_stiff_->Debug();
	ptr_stiff_->CalculateD(&D_, &x_);

}

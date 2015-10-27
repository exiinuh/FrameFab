#include "GraphCut.h"


GraphCut::GraphCut()
		:debug_(false), penalty_(10e2), pri_tol_(10e-3), dual_tol_(10e-3), D_tol_(0.1)
{
	// This default construction function should never be run
	// We need a mesh to begin with
}


GraphCut::GraphCut(WireFrame *ptr_frame)
		:debug_(false), penalty_(10e2), pri_tol_(10e-3), dual_tol_(10e-3), D_tol_(0.001)
{
	ptr_frame_ = ptr_frame;
	ptr_dualgraph_ = new DualGraph(ptr_frame_);
	ptr_stiff_ = new Stiffness(ptr_frame_, ptr_dualgraph_);
}


GraphCut::~GraphCut()
{
}


void GraphCut::CreateAandC()
{
	vector<DualEdge*> dual_edge = *(ptr_dualgraph_->get_edge_list());
	N_ = ptr_frame_->SizeOfVertList();
	M_ = ptr_frame_->SizeOfEdgeList();
	Nd_ = M_ / 2;
	Md_ = dual_edge.size();

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

	H1 = new SpMat(Nd_, Nd_);
	*H1 = A_.transpose() * C_ * A_;
}


void GraphCut::SetStartingPoints()
{
	x_.resize(Nd_);
	D_.resize(3 * N_);
	lambda_.resize(3 * N_);
	a_.resize(Nd_);
	x_render_.resize(Nd_);

	// Set all label x = 1 and calculate KD = F to obtain inital D_0	
	x_.setOnes();
	D_.setZero();
	lambda_.setZero();							
	a_.setZero();
	x_render_.setZero();
	
	//ptr_stiff_->Debug();

	ptr_stiff_->CreateFe();
}


void GraphCut::SetBoundary(VX &d, SpMat &W, int count)
{
	// Set lower boundary and upper boundary
	// Equality constraints W*x = d
	// Identify Base nodes(1) and Upper Nodes(2)
	// here we just take the edge with biggest height
	vector<WF_vert*> verts = *(ptr_frame_->GetVertList());
	VectorXi bound(Nd_);
	bound.setZero();

	if (count == 0)
	{
		// Find nodes with max height
		int max_id = 0;
		for (int i = 1; i < N_; i++)
		{
			if (verts[i]->Position().z() > verts[max_id]->Position().z())
			{
				max_id = i;
			}
		}

		WF_edge *edge = verts[max_id]->pedge_;								// Upper dual id
		while (edge != NULL)
		{
			int e_id = ptr_dualgraph_->dual_id(edge->ID());
			bound[e_id] = 2;
			x_[e_id] = 0;

			edge = edge->pnext_;
		}
	}
	else
	{
		// later iteration of ADMM
		VectorXd J(Md_);
		J = A_ * x_;

		for (int i = 0; i < Md_; i++)
		{
			if (J[i] == 1 || J[i] == -1)
			{
				int u = ptr_dualgraph_->u(i);
				int v = ptr_dualgraph_->v(i);
				if (x_[u] == 1)
				{
					// u is the lower dual vertex of cutting-edge 
					bound[u] = 2;
					x_[u] = 0;
				}
				else
				{
					// v is the lower dual vertex of cutting-edge 
					bound[v] = 2;
					x_[v] = 0;
				}
			}
		}
	}

	// Base dual 
	// find boundary nodes, in current stage boundary edge = base edge (for cat_head)
	vector<WF_edge*> edges = *(ptr_frame_->GetEdgeList());
	for (int i = 0; i < M_; i++)
	{
		if (edges[i]->ID() < edges[i]->ppair_->ID())
		{
			int e_id = ptr_dualgraph_->dual_id(i);
			int u = edges[i]->ppair_->pvert_->ID();
			int v = edges[i]->pvert_->ID();
			if (verts[u]->IsFixed() || verts[v]->IsFixed())
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


void GraphCut::UpdateX()
{
	// Convert previous continuous computed result to discrete 1-0 label
	for (int i = 0; i < x_.size(); i++)
	{
		if (x_[i] >= 0.5)
		{
			x_[i] = 1;
		}
		else
		{
			x_[i] = 0;
		}
		x_render_[i] += !x_[i];
	}
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
	cout << "iteration: " << iter_count << endl;
	cout << "Lower Set edge number : " << l << endl;
	cout << "Lower Set percentage" << double(l) / double(M_) * 100 << "%" << endl;
	
	if (iter_count == 8)
	//if (l < 20)
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
	if (primal_res.norm() <= pri_tol_ && dual_res.norm() <= dual_tol_)
	{
		return true;
	}
	else
	{
		double p_r = primal_res.norm();
		double d_r = dual_res.norm();

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
	ptr_dualgraph_->Dualization();

	int n = (ptr_dualgraph_->get_vert_list())->size();
	stop_n_ = floor(n / 5);										// set termination tolerance

	CreateAandC();
	SetStartingPoints();
	qp = QPFactory::make(static_cast<QPFactory::QPType>(1));

	cout << "penalty : " << penalty_ << endl;
	cout << "primal tolerance : " << pri_tol_ << endl;
	cout << "dual tolerance : "   << dual_tol_ << endl;

	vector<double> record;
	int cut_count = 0;
	do
	{
		VX d(Nd_);
		SpMat W(Nd_, Nd_);
		SetBoundary(d, W, cut_count);

		ptr_stiff_->CalculateD(&D_, &x_);

		double old_energy = x_.dot((*H1)*x_);
		cout << "Initial energy : " << old_energy << endl;

		int ADMM_count = 0;
		record.clear();

		cout << "---------------------------------" << endl;
		do
		{
			cout << "iteration: " << cut_count << ", ADMM " << ADMM_count << " round " << endl;
			
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
			dual_res = penalty_ * (x_ - x_prev).transpose() * Q_prev.transpose() * Q_prev 
				+ lambda_.transpose() * (Q_prev - Q_new);
			primal_res = Q_new * x_;


			double obj_func = x_.dot((*H1) * x_);
			record.push_back(obj_func);

			cout << "********" << endl;
			cout << "new energy func value record: " << obj_func << endl;
			cout << "dual_residual : " << dual_res.norm() << endl;
			cout << "primal_residual : " << primal_res.norm() << endl;

			for (int i = 0; i < record.size(); i++)
			{
				cout << record[i] << " << ";
			}
			putchar('\n');
			
			ADMM_count++;
		} while (!TerminationCriteria());

		UpdateX();
		
		printf("One iteration done!\n");
		cut_count++;
	} while (!CheckLabel(cut_count));
}

void GraphCut::CalculateQ(const VX _D, SpMat &Q)
{
	vector<WF_vert*> verts = *(ptr_frame_->GetVertList());
	vector<WF_edge*> edges = *(ptr_frame_->GetEdgeList());

	Q.resize(3 * N_, Nd_);
	vector<Eigen::Triplet<double>> Q_list;
	for (int i = 0; i < N_; i++)
	{
		WF_edge *edge = verts[i]->pedge_;
		while (edge != NULL)
		{
			int e_id = ptr_dualgraph_->dual_id(edge->ID());
			int j = edge->pvert_->ID();

			Matrix3d M = ptr_stiff_->Me(e_id);
			Vector3d Fe = ptr_stiff_->Fe(e_id);
			Vector3d Di(_D[3 * i], _D[3 * i + 1], _D[3 * i + 2]);
			Vector3d Dj(_D[3 * j], _D[3 * j + 1], _D[3 * j + 2]);
			Vector3d Gamma = M * (Dj - Di) - 0.5 * Fe;

			Q_list.push_back(Triplet<double>(3 * i, e_id, Gamma[0]));
			Q_list.push_back(Triplet<double>(3 * i + 1, e_id, Gamma[1]));
			Q_list.push_back(Triplet<double>(3 * i + 2, e_id, Gamma[2]));

			edge = edge->pnext_;
		}
	}
	Q.setFromTriplets(Q_list.begin(), Q_list.end());
}

void GraphCut::CalculateX(VX &d, SpMat &W)
{
	// Construct Hessian Matrix for D-Qp problem
	SpMat Q;
	CalculateQ(D_, Q);

	SpMat H2 = Q.transpose() * Q;
	SpMat H = 2 * (*H1) + penalty_ * H2;

	// Construct Linear coefficient for x-Qp problem
	a_ = Q.transpose() * lambda_;
	
	// Inequality constraints A*x <= b
	SpMat A(3 * N_, 3 * N_);
	A.setZero();
	VX b(3 * N_);
	b.setZero();

	// Variable constraints x >= lb, x <= ub
	VX lb(Nd_), ub(Nd_);													
	lb.setZero();
	ub.setOnes();

	qp->solve(H, a_, A, b, W, d, lb, ub, x_, NULL, NULL, debug_);
}


void GraphCut::CalculateD()
{
	// Construct Hessian Matrix for D-Qp problem
	ptr_stiff_->CreateK(&x_);
	SpMat K = *(ptr_stiff_->WeightedK());
	SpMat Q = penalty_ * K.transpose() * K;

	// Construct Linear coefficient for D-Qp problem
	ptr_stiff_->CreateFv(&x_);
	VX F = *(ptr_stiff_->WeightedF());

	VX a = K.transpose() * lambda_ - penalty_ * K.transpose() * F;
	
	// Inequality constraints A*D <= b
	SpMat A(3*N_,3*N_);
	A.setIdentity();	
	VX b(3*N_);
	b.setOnes();
	b = b * D_tol_;

	// Equality constraints C*D = d
	SpMat C(3 * N_, 3 * N_);
	C.setZero();
	VX d(3*N_);
	d.setZero();

	// Variable constraints D >= lb, D <= ub
	VX lb(3 * N_), ub(3 * N_);
	lb.setOnes();
	ub.setOnes();
	lb = lb * (-MYINF);
	ub = ub * MYINF;

	qp->solve(Q, a, A, b, C, d, lb, ub, D_, NULL, NULL, debug_);
}


void GraphCut::UpdateLambda()
{
	// Recompute K(x_{k+1}) and F(x_{k+1})
	ptr_stiff_->CreateK(&x_);
	SpMat K = *(ptr_stiff_->WeightedK());

	ptr_stiff_->CreateFv(&x_);
	VX F = *(ptr_stiff_->WeightedF());

	lambda_ = lambda_ + penalty_ * (K * D_ - F);
}


vector<DualVertex*> *GraphCut::GetDualVertexList()
{
	return ptr_dualgraph_->get_vert_list();
}


VectorXi *GraphCut::GetLabel()
{
	return &x_render_;
}
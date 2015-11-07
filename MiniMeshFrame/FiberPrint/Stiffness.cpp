#include "Stiffness.h"


Stiffness::Stiffness()
{
}


Stiffness::Stiffness(WireFrame *ptr_frame, DualGraph *ptr_dualgraph)
{
	ptr_frame_ = ptr_frame;
	ptr_dualgraph_ = ptr_dualgraph;
	ptr_parm_ = new FiberPrintPARM();		// default parameter set
}


Stiffness::Stiffness(WireFrame *ptr_frame, DualGraph *ptr_dualgraph, FiberPrintPARM *ptr_parm)
{
	ptr_frame_ = ptr_frame;
	ptr_dualgraph_ = ptr_dualgraph;
	ptr_parm_ = ptr_parm;
}


Stiffness::~Stiffness()
{
}


void Stiffness::CreateM()
{
	vector<WF_vert*> verts = *(ptr_frame_->GetVertList());
	int N = ptr_frame_->SizeOfVertList();
	int M = ptr_frame_->SizeOfEdgeList();
	int Nd = ptr_dualgraph_->SizeOfVertList();
	int Fd = ptr_dualgraph_->SizeOfFaceList();

	M_.resize(Nd);
	for (int i = 0; i < Nd; i++)
	{
		M_[i].setZero();
	}

	for (int i = 0; i < Fd; i++)
	{
		int u = ptr_dualgraph_->v_orig_id(i);
		WF_edge *edge = verts[u]->pedge_;
		while (edge != NULL)
		{
			int e_id = ptr_dualgraph_->e_dual_id(edge->ID());
			if (edge->ID() < edge->ppair_->ID() && e_id != -1)
			{
				WF_vert *neighbor = edge->pvert_;

				double dx = verts[u]->Position().x() - neighbor->Position().x();
				double dy = verts[u]->Position().y() - neighbor->Position().y();
				double dz = verts[u]->Position().z() - neighbor->Position().z();
				double len = sqrt(dx*dx + dy*dy + dz*dz);
				double volume = pow(len, 3);

				double G = ptr_parm_->ShearModulus();
				double E = ptr_parm_->YoungsModulus();
				double material = G - E;
				double radius = ptr_parm_->Radius();

				Matrix3d Muv;
				Muv.setZero();

				/*x axis*/
				Muv(0, 0) = -material * M_PI * radius * radius / volume * dx * dx
					+ G * M_PI * radius * radius / len;
				Muv(0, 1) = -material * M_PI * radius * radius / volume * dx * dy;
				Muv(0, 2) = -material * M_PI * radius * radius / volume * dx * dz;

				/*y axis*/
				Muv(1, 0) = -material * M_PI * radius * radius / volume * dx * dy;
				Muv(1, 1) = -material * M_PI * radius * radius / volume * dy * dy
					+ G * M_PI * radius * radius / len;
				Muv(1, 2) = -material * M_PI * radius * radius / volume * dz * dy;

				/*z axis*/
				Muv(2, 0) = -material * M_PI * radius * radius / volume * dx * dz;
				Muv(2, 1) = -material * M_PI * radius * radius / volume * dy * dz;
				Muv(2, 2) = -material * M_PI * radius * radius / volume * dz * dz
					+ G * M_PI * radius * radius / len;

				M_[e_id] = Muv;
				
				//MatrixXd M_tmp = Muv;
				//Statistics s_M("LocalStiffMatrix_CreateM", M_tmp);
				//s_M.GenerateMatrixFile();
			}
			edge = edge->pnext_;
		}
	}
}

/*
void Stiffness::CreateInitK()
{
	vector<WF_edge*> edges = *(ptr_frame_->GetEdgeList());
	vector<WF_vert*> verts = *(ptr_frame_->GetVertList());

	int N = ptr_frame_->SizeOfVertList();
	int M = ptr_frame_->SizeOfEdgeList();
	int Nd = M / 2;

	vector<Matrix3d> Mn;
	Mn.resize(N);
	for (int i = 0; i < N; i++)
	{
		Mn[i].setZero();
	}

	vector<Triplet<double>> K_list;

	Nk_ = 0;
	v_id_.resize(N);
	fill(v_id_.begin(), v_id_.end(), 0);

	for (int i = 0; i < N; i++)
	{
		bool flag = false;
		for (int j = 0; j < 3; j++)
		{
			for (int k = 0; k < 3; k++)
			{
				if (Mn[i](j, k) != 0)
				{
					K_list.push_back(Triplet<double>(Nk_ * 3 + j, Nk_ * 3 + k, Mn[i](j, k)));
					flag = true;
				}
			}
		}
		if (!flag)
		{
			if (verts[i]->IsFixed())
			{
				Mn[i](0, 0) = Mn[i](1, 1) = Mn[i](2, 2) = 1;
				K_list.push_back(Triplet<double>(Nk_ * 3, Nk_ * 3, Mn[i](0, 0)));
				K_list.push_back(Triplet<double>(Nk_ * 3 + 1, Nk_ * 3 + 1, Mn[i](1, 1)));
				K_list.push_back(Triplet<double>(Nk_ * 3 + 2, Nk_ * 3 + 2, Mn[i](2, 2)));
				v_id_[Nk_++] = i;
			}
		}
		else
		{
			v_id_[Nk_++] = i;
		}
	}

	K_.resize(3 * Nk_, 3 * Nk_);
	K_.setFromTriplets(K_list.begin(), K_list.end());
}
*/

void Stiffness::CreateK(const VectorXd *ptr_x)
{
	vector<WF_edge*> edges = *(ptr_frame_->GetEdgeList());
	vector<WF_vert*> verts = *(ptr_frame_->GetVertList());
	int N = ptr_frame_->SizeOfVertList();
	int M = ptr_frame_->SizeOfEdgeList();
	int Nd = ptr_dualgraph_->SizeOfVertList();
	int Fd = ptr_dualgraph_->SizeOfFaceList();			// Fd = Number of nodes in Current orig graph

	Statistics s_x("x_CreateK", *ptr_x);
	s_x.GenerateVectorFile();

	vector<Matrix3d> Mn;
	Mn.resize(Fd);
	for (int i = 0; i < Fd; i++)
	{
		Mn[i].setZero();
	}
	
	vector<Triplet<double>> K_list;
	for (int e_id = 0; e_id < Nd; e_id++)
	{
		int i = ptr_dualgraph_->e_orig_id(e_id);
		int u = edges[i]->ppair_->pvert_->ID();			// node id at orig graph
		int v = edges[i]->pvert_->ID();
		int dual_u = ptr_dualgraph_->v_dual_id(u);		// DualFace Id (orig graph node's renumbering id after previous cut)
		int dual_v = ptr_dualgraph_->v_dual_id(v);

		if (!verts[u]->IsFixed())
		{
			Mn[dual_u] -= (*ptr_x)[e_id] * M_[e_id];

			if (!verts[v]->IsFixed())
			{
				for (int j = 0; j < 3; j++)
				{
					for (int k = 0; k < 3; k++)
					{
						if (M_[e_id](j, k) != 0)
						{
							K_list.push_back(Triplet<double>(dual_u * 3 + j, dual_v * 3 + k, M_[e_id](j, k)));
						}
					}
				}
			}
		}

		if (!verts[v]->IsFixed())
		{
			Mn[dual_v] -= (*ptr_x)[e_id] * M_[e_id];
			if (!verts[u]->IsFixed())
			{
				for (int j = 0; j < 3; j++)
				{
					for (int k = 0; k < 3; k++)
					{
						if (M_[e_id](j, k) != 0)
						{
							K_list.push_back(Triplet<double>(dual_v * 3 + j, dual_u * 3 + k, M_[e_id](j, k)));
						}
					}
				}
			}
		}
	}

	for (int i = 0; i < Fd; i++)
	{
		bool flag = false;
		for (int j = 0; j < 3; j++)
		{
			for (int k = 0; k < 3; k++)
			{
				if (Mn[i](j, k) != 0)
				{
					K_list.push_back(Triplet<double>(i * 3 + j, i * 3 + k, Mn[i](j, k)));
					flag = true;
				}
			}
		}
		if (!flag)
		{
			Mn[i](0, 0) = Mn[i](1, 1) = Mn[i](2, 2) = 1;
			K_list.push_back(Triplet<double>(i * 3, i * 3, Mn[i](0, 0)));
			K_list.push_back(Triplet<double>(i * 3 + 1, i * 3 + 1, Mn[i](1, 1)));
			K_list.push_back(Triplet<double>(i * 3 + 2, i * 3 + 2, Mn[i](2, 2)));
		}
	}

	K_.resize(3 * Fd, 3 * Fd);
	K_.setFromTriplets(K_list.begin(), K_list.end());
}


void Stiffness::CreateFv(const VectorXd *ptr_x)
{
	vector<WF_vert*> verts = *(ptr_frame_->GetVertList());
	int Fd = ptr_dualgraph_->SizeOfFaceList();

	Fv_.resize(3 * Fd);
	Fv_.setZero();

	for (int i = 0; i < Fd; i++)
	{
		double gravity = 0;
		int u = ptr_dualgraph_->v_orig_id(i);

		// if not boundary
		if (!verts[u]->IsFixed())
		{
			WF_edge *edge = verts[u]->pedge_;
			while (edge != NULL)
			{
				int e_id = ptr_dualgraph_->e_dual_id(edge->ID());

				if (e_id != -1)
				{
					gravity += (*ptr_x)[e_id] * Fe_[edge->ID()];
				}

				edge = edge->pnext_;
			}
		}
		else
		{
			// do nothing
			//printf("Bdy; node %i: gravity %.8lf \n", u, gravity);
		}
		Fv_[3 * i + 2] = gravity / 2;
		//printf("Fv_[%i] = %.12lf \n", 3 * u + 2, gravity / 2);
	}
}


void Stiffness::CreateFe()
{
	double r = ptr_parm_->Radius();
	double density = ptr_parm_->Density();
	double g = ptr_parm_->G();

	vector<WF_edge*> edges = *(ptr_frame_->GetEdgeList());
	int M = ptr_frame_->SizeOfEdgeList();

	Fe_.resize(M);
	Fe_.setZero();

	for (int i = 0; i < M; i++)
	{
		int j = edges[i]->ppair_->ID();
		if (i < j)
		{
			WF_vert *u = edges[i]->ppair_->pvert_;
			WF_vert *v = edges[i]->pvert_;

			double dx = u->Position().x() - v->Position().x();
			double dy = u->Position().y() - v->Position().y();
			double dz = u->Position().z() - v->Position().z();
			double len = sqrt(dx*dx + dy*dy + dz*dz);
			double gravity = M_PI * r * r * len * density * g;

			Fe_[i] = Fe_[j] = gravity;
		}
	}
}


void Stiffness::CalculateD(VectorXd *ptr_D, const VectorXd *ptr_x)
{
	CreateM();
	CreateK(ptr_x);
	CreateFv(ptr_x);

	//SparseQR<SparseMatrix<double>, COLAMDOrdering<int>> solver;
	//BiCGSTAB<SparseMatrix<double>> solver;
	//SparseLU<SparseMatrix<double>, COLAMDOrdering<int>> solver;
	FullPivLU<MatrixXd> solver;

	K_.makeCompressed();
	solver.compute(K_);

	/*
	for (int i = 0; i < 3 * Nk_; i++)
	{
		if (K_.coeff(i, i) == 0)
		{
			printf("%d\n", i);
		}
	}
	*/

	cout << "column number of K_ : " << K_.cols() << endl;
	cout << "rank of K_ : "			 << solver.rank() << endl;
	//getchar();
	
	(*ptr_D) = solver.solve(Fv_);
	/*
	VectorXd D;
	D.resize(3 * Nk_);
	D.setZero();
//	assert(solver.info() == Success);
	D = solver.solve(Fv_);
	for (int i = 0; i < Nk_; i++)
	{
		(*ptr_D)[3 * v_id_[i] + 2] = D[3 * i + 2];
	}
//	assert(solver.info() == Success);
*/
}


Matrix3d Stiffness::Me(int ei)
{
	int e_id = ptr_dualgraph_->e_dual_id(ei);
	return M_[e_id];
}


Vector3d Stiffness::Fe(int ei)
{
	Vector3d Fei;
	Fei[0] = 0;
	Fei[1] = 0;
	Fei[2] = Fe_[ei];
	return Fei;
}


void Stiffness::Debug()
{
	FILE *fp = fopen("E:\\test.txt", "wb+"); 
	vector<WF_edge*> &edges = *(ptr_frame_->GetEdgeList());
	int N = ptr_frame_->SizeOfVertList();
	int M = ptr_frame_->SizeOfEdgeList();
	int Nd = M / 2;

	VectorXd x;
	x.resize(Nd);
	x.setOnes();

	CreateM();
	CreateK(&x);
	CreateFv(&x);

	for (int i = 0; i < N; i++)
	{
		if (K_.coeff(i * 3, i * 3) == 0 || K_.coeff(i * 3 + 1, i * 3 + 1) == 0 || K_.coeff(i * 3 + 2, i * 3 + 2) == 0)
		{
			for (int p = 0; p < 3; p++)
			{
				for (int q = 0; q < 3; q++)
				{
					printf("%lf ", K_.coeff(i * 3 + p, i * 3 + q));
				}
				printf("\n");
			}
		}
	}

	for (int i = 0; i < Nd; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			for (int k = 0; k < 3; k++)
			{
				//if (M_[i](j, k) != 0)
				{
					fprintf(fp, "%.8f   ", M_[i](j, k));
				}
			}
			fprintf(fp, "\r\n");
		}
		fprintf(fp, "\r\n");
	}


	for (int i = 0; i < 3 * N; i++)
	{
		for (int j = 0; j < 3 * N; j++)
		{
			if (K_.coeff(i, j) != 0)
			{
				fprintf(fp, "(%d, %d)   %.4f\r\n", i + 1, j + 1, K_.coeff(i, j));
			}
		}
	}
	fclose(fp);

	VectorXd D;
	D.resize(3 * N);
	D.setZero();
	CalculateD(&D, &x);
	for (int i = 0; i < N; i++)
	{
		cout << D[i * 3] << " " << D[i * 3 + 1] << " " << D[i * 3 + 2] << endl;
	}
	getchar();
}
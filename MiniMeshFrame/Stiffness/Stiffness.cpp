#include "Stiffness.h"


Stiffness::Stiffness()
{
}


Stiffness::Stiffness(DualGraph *ptr_dualgraph)
:r_(0.0015), nr_(0), density_(0.001), g_(9.80), G_(1586), E_(1387), v_(0.16)
{
	ptr_dualgraph_ = ptr_dualgraph;

	Init();
}


Stiffness::Stiffness(DualGraph *ptr_dualgraph, FiberPrintPARM *ptr_parm)
{
	ptr_dualgraph_ = ptr_dualgraph;

	r_ = ptr_parm->radius_;
	nr_ = 0.0;
	density_ = ptr_parm->density_;
	g_ = ptr_parm->g_;
	G_ = ptr_parm->shear_modulus_;
	E_ = ptr_parm->youngs_modulus_;
	v_ = 0.16;

	Init();
}


Stiffness::~Stiffness()
{
}


void Stiffness::Init()
{
	CreateFe();
	CreateElasticK();
}


void Stiffness::CreateFe()
{
	WireFrame *ptr_frame = ptr_dualgraph_->ptr_frame_;
	int Nd = ptr_dualgraph_->SizeOfVertList();
	int Fd = ptr_dualgraph_->SizeOfFaceList();

	CoordTrans coord_trans;
	double Ax = M_PI * r_ * r_;
	double gx = 0;
	double gy = 0;
	double gz = g_;

	Fe_.resize(Nd);
	for (int i = 0; i < Nd; i++)
	{
		Fe_[i].setZero();

		WF_edge *ei = ptr_frame->GetEdge(ptr_dualgraph_->e_orig_id(i));
		WF_edge *ej = ei->ppair_;
		int dual_u = ptr_dualgraph_->v_dual_id(ej->pvert_->ID());
		int dual_v = ptr_dualgraph_->v_dual_id(ei->pvert_->ID());

		double t0, t1, t2, t3, t4, t5, t6, t7, t8;
		coord_trans.CreateTransMatrix(ej->pvert_->Position(), ei->pvert_->Position(),
			t0, t1, t2, t3, t4, t5, t6, t7, t8, 0.0);

		VectorXd Fei(12);
		Fei.setZero();

		double L = ei->Length();
		Fei[0] = Fei[6] = density_ * Ax * L * gx / 2.0;
		Fei[1] = Fei[7] = density_ * Ax * L * gy / 2.0;
		Fei[2] = Fei[8] = density_ * Ax * L * gz / 2.0;
		Fei[3] = density_ * Ax * L * L / 12.0 * ((-t3*t7 + t4*t6)*gy + (-t3*t8 + t5*t6)*gz);
		Fei[4] = density_ * Ax * L * L / 12.0 * ((-t4*t6 + t3*t7)*gx + (-t4*t8 + t5*t7)*gz);
		Fei[5] = density_ * Ax * L * L / 12.0 * ((-t5*t6 + t3*t8)*gx + (-t5*t7 + t4*t8)*gy);
		Fei[9] = density_ * Ax * L * L / 12.0 * ((t3*t7 - t4*t6)*gy + (t3*t8 - t5*t6)*gz);
		Fei[10] = density_ * Ax * L * L / 12.0 * ((t4*t6 - t3*t7)*gx + (t4*t8 - t5*t7)*gz);
		Fei[11] = density_ * Ax * L * L / 12.0 * ((t5*t6 - t3*t8)*gx + (t5*t7 - t4*t8)*gy);

		Fe_[i] = Fei;
	}
}


void Stiffness::CreateF(const VectorXd *ptr_x)
{
	WireFrame *ptr_frame = ptr_dualgraph_->ptr_frame_;
	int Nd = ptr_dualgraph_->SizeOfVertList();
	int Fd = ptr_dualgraph_->SizeOfFaceList();

	F_.resize(6 * Fd);
	F_.setZero();

	for (int i = 0; i < Nd; i++)
	{
		WF_edge *ei = ptr_frame->GetEdge(ptr_dualgraph_->e_orig_id(i));
		WF_edge *ej = ei->ppair_;
		int dual_u = ptr_dualgraph_->v_dual_id(ej->pvert_->ID());
		int dual_v = ptr_dualgraph_->v_dual_id(ei->pvert_->ID());

		for (int j = 0; j < 6; j++)
		{
			F_[dual_u * 6 + j] += (*ptr_x)[i] * Fe_[i][j];
			F_[dual_v * 6 + j] += (*ptr_x)[i] * Fe_[i][j + 6];
		}
	}
}


void Stiffness::CreateElasticK()
{
	WireFrame *ptr_frame = ptr_dualgraph_->ptr_frame_;
	int Nd = ptr_dualgraph_->SizeOfVertList();
	int Fd = ptr_dualgraph_->SizeOfFaceList();

	double material = G_ - E_;
	double Ax = M_PI * r_ * r_;
	double Asy = Ax * (6 + 12 * v_ + 6 * v_*v_) / (7 + 12 * v_ + 4 * v_*v_);
	double Asz = Asy;
	double Jxx = 0.5 * M_PI * r_ * r_ * r_ * r_;
	double Iyy = Jxx / 2;
	double Izz = Iyy;

	eK_.resize(Nd);
	for (int i = 0; i < Nd; i++)
	{
		eK_[i].setZero();

		WF_edge *ei = ptr_frame->GetNeighborEdge(ptr_dualgraph_->v_orig_id(i));
		int u = ei->ppair_->pvert_->ID();
		int v = ei->pvert_->ID();
		double L = ei->Length();
		double Le = L - 2 * nr_;

		MatrixXd eKuv(12, 12);
		eKuv.setZero();

		double Ksy = 12.0 * E_ * Izz / (G_ * Asy * Le * Le);
		double Ksz = 12.0 * E_ * Iyy / (G_ * Asz * Le * Le);

		eKuv(0, 0) = eKuv(6, 6) = E_ * Ax / Le;
		eKuv(1, 1) = eKuv(7, 7) = 12.0 * E_ * Izz / (Le * Le * Le * (1.0 + Ksy));
		eKuv(2, 2) = eKuv(8, 8) = 12.0 * E_ * Iyy / (Le * Le * Le * (1.0 + Ksz));
		eKuv(3, 3) = eKuv(9, 9) = G_ * Jxx / Le;
		eKuv(4, 4) = eKuv(10, 10) = (4.0 + Ksz) * E_ * Iyy / (Le * (1.0 + Ksz));
		eKuv(5, 5) = eKuv(11, 11) = (4.0 + Ksy) * E_ * Izz / (Le * (1.0 + Ksy));

		eKuv(4, 2) = eKuv(2, 4) = -6.0 * E_ *Iyy / (Le * Le * (1.0 + Ksz));
		eKuv(5, 1) = eKuv(1, 5) = 6.0 * E_ *Izz / (Le * Le * (1.0 + Ksy));

		eKuv(6, 0) = eKuv(0, 6) = -eKuv(0, 0);
		eKuv(7, 1) = eKuv(1, 7) = -eKuv(1, 1);
		eKuv(8, 2) = eKuv(2, 8) = -eKuv(2, 2);
		eKuv(9, 3) = eKuv(3, 9) = -eKuv(3, 3);

		eKuv(10, 2) = eKuv(2, 10) = eKuv(4, 2);
		eKuv(11, 1) = eKuv(1, 11) = eKuv(5, 1);
		eKuv(10, 8) = eKuv(8, 10) = eKuv(8, 4) = eKuv(4, 8) = -eKuv(4, 2);
		eKuv(11, 7) = eKuv(7, 11) = eKuv(7, 5) = eKuv(5, 7) = -eKuv(5, 1);

		eKuv(10, 5) = eKuv(5, 10) = (2.0 - Ksz) * E_ * Iyy / (Le * (1.0 + Ksz));
		eKuv(11, 5) = eKuv(5, 11) = (2.0 - Ksy) * E_ * Izz / (Le * (1.0 + Ksy));

		for (int k = 0; k < 12; k++)
		{
			for (int l = 0; l < 12; l++)
			{
				if (eKuv(k, l) != eKuv(l, k))
				{
					eKuv(k, l) = eKuv(l, k) = (eKuv(k, l) + eKuv(l, k)) / 2;
				}
			}
		}

		eK_[i] = eKuv;
	}
}


void Stiffness::CreateGlobalK(const VectorXd *ptr_x)
{
	WireFrame *ptr_frame = ptr_dualgraph_->ptr_frame_;
	int Nd = ptr_dualgraph_->SizeOfVertList();
	int Fd = ptr_dualgraph_->SizeOfFaceList();

	vector<Triplet<double>> K_list;

	K_.resize(6 * Fd, 6 * Fd);
	for (int i = 0; i < Nd; i++)
	{
		WF_edge *ei = ptr_frame->GetEdge(ptr_dualgraph_->e_orig_id(i));
		WF_edge *ej = ei->ppair_;
		int u = ej->pvert_->ID();
		int v = ei->pvert_->ID();
		int dual_u = ptr_dualgraph_->v_dual_id(u);
		int dual_v = ptr_dualgraph_->v_dual_id(v);

		for (int k = 0; k < 6; k++)
		{
			for (int l = 0; l < 6; l++)
			{
				K_list.push_back(Triplet<double>(dual_u * 6 + k, dual_u * 6 + l,
					(*ptr_x)[i] * eK_[i](k, l)));
				K_list.push_back(Triplet<double>(dual_v * 6 + k, dual_v * 6 + l,
					(*ptr_x)[i] * eK_[i](k + 6, l + 6)));
				K_list.push_back(Triplet<double>(dual_u * 6 + k, dual_v * 6 + l, 
					(*ptr_x)[i] * eK_[i](k, l + 6)));
				K_list.push_back(Triplet<double>(dual_v * 6 + k, dual_u * 6 + l,
					(*ptr_x)[i] * eK_[i](k + 6, l)));
			}
		}
	}
	K_.setFromTriplets(K_list.begin(), K_list.end());
}


void Stiffness::CalculateD(VectorXd *ptr_D)
{
	int Nd = ptr_dualgraph_->SizeOfVertList();
	VX x(Nd); 
	x.setOnes();

	CreateGlobalK(&x);
	CreateF(&x);

	//SparseQR<SparseMatrix<double>, COLAMDOrdering<int>> solver;
	//BiCGSTAB<SparseMatrix<double>> solver;
	//SparseLU<SparseMatrix<double>, COLAMDOrdering<int>> solver;
	FullPivLU<MatrixXd> solver;

	cout << "Stiffness: Calculating Initial D." << endl;

	//K_.makeCompressed();

	solver.compute(K_);
	//assert(solver.info() == Success);

	//cout << "column number of K_ : " << K_.cols() << endl;
	//cout << solver.rank() << endl;
	//getchar();

	(*ptr_D) = solver.solve(F_);
	//assert(solver.info() == Success);

	cout << "Stiffness: Initial D Calculation Completed." << endl;
}


void Stiffness::CalculateD(VectorXd *ptr_D, const VectorXd *ptr_x)
{
	CreateGlobalK(ptr_x);
	CreateF(ptr_x);

	//SparseQR<SparseMatrix<double>, COLAMDOrdering<int>> solver;
	//BiCGSTAB<SparseMatrix<double>> solver;
	//SparseLU<SparseMatrix<double>, COLAMDOrdering<int>> solver;
	FullPivLU<MatrixXd> solver;

	cout << "Stiffness: Calculating Initial D." << endl;
	
	//K_.makeCompressed();
	
	solver.compute(K_);
	//assert(solver.info() == Success);

	//cout << "column number of K_ : " << K_.cols() << endl;
	//cout << solver.rank() << endl;
	//getchar();
	
	(*ptr_D) = solver.solve(F_);
	//assert(solver.info() == Success);

	cout << "Stiffness: Initial D Calculation Completed." << endl;
}


MatrixXd Stiffness::eKe(int ei)
{
	MX tmpK(6, 6);
	tmpK.setZero();

	int dual_id = ptr_dualgraph_->e_dual_id(ei);
	if (ptr_dualgraph_->e_orig_id(dual_id) == ei)
	{
		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 6; j++)
			{
				tmpK(i, j) = eK_[dual_id](i, j + 6);
			}
		}
	}
	else
	{
		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 6; j++)
			{
				tmpK(i, j) = eK_[dual_id](i + 6, j);
			}
		}
	}

	return tmpK;
}


MatrixXd Stiffness::eKv(int ei)
{
	MX tmpK(6, 6);
	tmpK.setZero();

	int dual_id = ptr_dualgraph_->e_dual_id(ei);
	if (ptr_dualgraph_->e_orig_id(dual_id) == ei)
	{
		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 6; j++)
			{
				tmpK(i, j) = eK_[dual_id](i, j);
			}
		}
	}
	else
	{
		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 6; j++)
			{
				tmpK(i, j) = eK_[dual_id](i + 6, j + 6);
			}
		}
	}

	return tmpK;
}


VectorXd Stiffness::Fe(int ei)
{
	VX tmpF(6);
	tmpF.setZero();

	int dual_id = ptr_dualgraph_->e_dual_id(ei);
	if (ptr_dualgraph_->e_orig_id(dual_id) == ei)
	{
		for (int j = 0; j < 6; j++)
		{
			tmpF[j] = Fe_[ei][j];
		}
	}
	else
	{
		for (int j = 6; j < 12; j++)
		{
			tmpF[j] = Fe_[ei][j];
		}
	}
	return tmpF;
}


void Stiffness::Debug()
{
	/*
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
	*/
}
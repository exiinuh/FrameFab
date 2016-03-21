#include "Stiffness.h"


Stiffness::Stiffness()
{
}

Stiffness::Stiffness(WireFrame *ptr_frame, DualGraph *ptr_dualgraph, FiberPrintPARM *ptr_parm)
{
	ptr_frame_ = ptr_frame;
	ptr_dualgraph_ = ptr_dualgraph;

	//radius_ = ptr_parm->radius_;
	//density_ = ptr_parm->density_;
	//g_ = ptr_parm->g_;
	//G_ = ptr_parm->shear_modulus_;
	//E_ = ptr_parm->youngs_modulus_;
	//material_ = G_ - E_;

	assert(nE_ != 0);

	L_.resize(nE_);			/* length of each element		*/
	Le_.resize(nE_);		/* effective length of each element	*/

	N1_.resize(nE_);		/* node #1 of each element		*/
	N2_.resize(nE_);		/* node #2 of each element		*/

	Ax_.resize(nE_);		/* cross section area of each element	*/
	Asy_.resize(nE_);		/* shear area in local y direction 	*/
	Asz_.resize(nE_);		/* shear area in local z direction	*/
	Jx_.resize(nE_);		/* torsional moment of inertia 		*/
	Iy_.resize(nE_);		/* bending moment of inertia about y-axis */
	Iz_.resize(nE_);		/* bending moment of inertia about z-axis */

	E_.resize(nE_);		/* frame element Young's modulus	*/
	G_.resize(nE_);		/* frame element shear modulus		*/
	p_.resize(nE_);		/* element rotation angle about local x axis */
	d_.resize(nE_);		/* element mass density			*/
}

Stiffness::~Stiffness()
{
}

void Stiffness::ElasticK(MX &k, int ElemId, int shear)
{
	double   t1, t2, t3, t4, t5, t6, t7, t8, t9;     // coordinate transformation matrix, t1, t2, ..., t9
	double	 Ksy, Ksz;							     // shear deformation coefficients
	int      i, j;

	int n1 = N1_[ElemId];
	int n2 = N2_[ElemId];

	double r = rj_[ElemId];
	double L  = L_[ElemId];
	double Le = Le_[ElemId];
	double Ax   = Ax_[ElemId];
	double Asy  = Asy_[ElemId];
	double Asz  = Asz_[ElemId];
	double Jx = Jx_[ElemId];
	double Iy = Iy_[ElemId];
	double Iz = Iz_[ElemId];
	double E = E_[ElemId];
	double G = G_[ElemId];
	double p = p_[ElemId];

	trsf_.CreateTransMatrix(xyz_, L, n1, n2,
		t1, t2, t3, t4, t5, t6, t7, t8, t9, p);

	k.resize(12, 12);
	k.setZero();

	if (shear)
	{
		Ksy = 12.*E*Iz / (G*Asy*Le*Le);
		Ksz = 12.*E*Iy / (G*Asz*Le*Le);
	}
	else
	{
		Ksy = Ksz = 0.0;
	}

	k(0,0) = k(6,6) = E*Ax / Le;
	k(1,1) = k(7,7) = 12.*E*Iz / (Le*Le*Le*(1. + Ksy));
	k(2,2) = k(8,8) = 12.*E*Iy / (Le*Le*Le*(1. + Ksz));
	k(3,3) = k(9,9) = G*Jx / Le;
	k(4,4) = k(10,10) = (4. + Ksz)*E*Iy / (Le*(1. + Ksz));
	k(5,5) = k(11,11) = (4. + Ksy)*E*Iz / (Le*(1. + Ksy));

	k(4,2) = k(2,4) = -6.*E*Iy / (Le*Le*(1. + Ksz));
	k(5,1) = k(1,5) = 6.*E*Iz / (Le*Le*(1. + Ksy));
	k(6,0) = k(0,6) = -k(0,0);

	k(11,7) = k(7,11) = k(7,5) = k(5,7) = -k(5,1);
	k(10,8) = k(8,10) = k(8,4) = k(4,8) = -k(4,2);
	k(9,3) = k(3,9) = -k(3,3);
	k(10,2) = k(2,10) = k(4,2);
	k(11,1) = k(1,11) = k(5,1);

	k(7,1) = k(1,7) = -k(1,1);
	k(8,2) = k(2,8) = -k(2,2);
	k(10,4) = k(4,10) = (2. - Ksz)*E*Iy / (Le*(1. + Ksz));
	k(11,5) = k(5,11) = (2. - Ksy)*E*Iz / (Le*(1. + Ksy));

	trsf_.TransLocToGlob(t1, t2, t3, t4, t5, t6, t7, t8, t9, k, rj_[n1], rj_[n2]);	// globalize element stiffness matrix k

	/* check and enforce symmetry of elastic element stiffness matrix */
	for (i = 0; i < 12; i++)
	{
		for (j = i ; j < 12; j++)
		{
			if (k(i,j) != k(j,i)) 
			{
				if (abs(k(i,j) / k(j,i) - 1.0) > 1.0e-6 &&
					 (fabs(k(i,j) / k(i,i)) > 1e-6 || fabs(k(j,i) / k(i,i)) > 1e-6))  
				{
					fprintf(stderr, "elastic_K: element stiffness matrix not symetric ...\n");
					fprintf(stderr, " ... k[%d][%d] = %15.6e \n", i, j, k(i,j));
					fprintf(stderr, " ... k[%d][%d] = %15.6e   ", j, i, k(j,i));
					fprintf(stderr, " ... relative error = %e \n", fabs(k(i,j) / k(j,i) - 1.0));
					fprintf(stderr, " ... element matrix saved in file 'kt'\n");
					
					Statistics s_kt("kt", k);
					s_kt.GenerateMatrixFile();
				}

				k(i,j) = k(j,i) = 0.5 * (k(i,j) + k(j,i));
			}
		}
	}
}

void Stiffness::AssembleK()
{
	MX	k;		// element stiffness matrix in global coord
	int	res = 0,
		i, j, ii, jj, l, kk;
	//char	stiffness_fn[FILENMAX];

	DoF_ = 6 * nN_;
	K_.resize(DoF_, DoF_);

	k.resize(12, 12);
	ind_.resize(12, nE_);

	// Initialize Displacement, reactions to 0
	D_.setZero();
	R_.setZero();

	// formulate element dof index table
	for (i = 1; i <= nE_; i++)
	{
		ind_(1,i) = 6 * N1_[i] - 5;		ind_(7,i)  = 6 * N2_[i] - 5;
		ind_(2,i) = ind_(1,i) + 1;		ind_(8,i)  = ind_(7,i) + 1;
		ind_(3,i) = ind_(1,i) + 2;		ind_(9,i)  = ind_(7,i) + 2;
		ind_(4,i) = ind_(1,i) + 3;		ind_(10,i) = ind_(7,i) + 3;
		ind_(5,i) = ind_(1,i) + 4;		ind_(11,i) = ind_(7,i) + 4;
		ind_(6,i) = ind_(1,i) + 5;		ind_(12,i) = ind_(7,i) + 5;
	}

	for (i = 1; i <= nE_; i++)
	{

		ElasticK(k, i, shear_);

		//if (geom_)
		//	geometric_K(k, xyz, r, L[i], Le[i], N1[i], N2[i],
		//	Ax[i], Asy[i], Asz[i],
		//	Jx[i], Iy[i], Iz[i],
		//	E[i], G[i], p[i], -Q[i][1], shear);

		// at element i, for each dof 
		for (l = 1; l <= 12; l++) 
		{
			ii = ind_(l,i);
			for (kk = 0; kk < 12; kk++) 
			{
				jj = ind_(kk,i);
				K_(ii,jj) += k(l,kk);
			}
		}
	}
	return;
}

void	Stiffness::SolveSystem()
{
	int		sfrv = 0;			// scanf return value
	FILE    *fp;				// input and output file pointer
	char	stripped_inputpath[512];	// temp data path
	char	*data_dir = "F:\\FiberPrintProject\\ResultData\\Frame3dd_data\\";

	// Set datafile
	strcpy(IN_file_, "\0");
	strcpy(OUT_file_, "\0");

	printf(" FiberPrint Matrix Structual Analysis SubModule:\n");
	printf(" Please enter the  input data file name: ");
	sfrv = scanf("%s", IN_file_);
	assert(sfrv == 1);

	fprintf(stderr, " Please enter the output data file name: ");
	sfrv = scanf("%s", OUT_file_);
	assert(sfrv == 1);

	sprintf(stripped_inputpath, "%s%c", data_dir, IN_file_);
	if ((fp = fopen(stripped_inputpath, "w")) == NULL)
	{
		printf("cannot get parsed input data file : '%s' \n", stripped_inputpath);
		exit(12);
	}


	// open the input data file
	if ((fp = fopen(IN_file_, "r")) == NULL) 
	{ 
		printf("ERROR: Cannot open input data file '%s'\n", IN_file_);
		exit(11);
	}

}
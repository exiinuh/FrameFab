#include "IllCondDetector.h"

// mapping from two-dim index to packed 'Up' column major layout storage
#define MAP(i,j,N) (i*N + j)

IllCondDetector::IllCondDetector(EigenSp const &K)
{
	EIGEN_LAP(K);
}

IllCondDetector::~IllCondDetector()
{
	if (A_ != NULL)
	{
		free(A_);
	}
}

void IllCondDetector::EIGEN_LAP(EigenSp const &K)
{
	// Convert Eigen library storage into LAPACK storage

	N_ = K.cols();
	A_ = (double*)calloc(N_*N_, sizeof(double));

	for (int i = 0; i < N_; i++)
	{
		for (int j = 0; j < N_; j++)
		{
			A_[MAP(i, j, N_)] = K.coeff(i, j);
		}
	}
	
	// 1-norm computing
	std::vector<double> tmp;
	for (int i = 0; i < N_; i++)
	{
		double sum = 0;
		for (int j = 0; j < N_; j++)
		{
			sum += A_[MAP(i, j, N_)];
		}
		tmp.push_back(sum);
	}
	auto max = std::max_element(tmp.begin(), tmp.end());
	Anorm_ = *max;
}

void IllCondDetector::SetParm(int _ns, int _nl, int _condthres, int _gap)
{
	assert(_ns > 0 || _nl > 0 || _condthres > 1 || _gap > 0);
	ns_ = _ns;
	nl_ = _nl;
	cond_thres_ = _condthres;
	gap_ = _gap;
}

void IllCondDetector::ComputeCondNum()
{
	// rank checking
	char *jobu  = "N";		// No colums of U    are computed
	char *jobvt = "N";		// No rows   of V**T are computed
	int	  lda = N_;			// Leading dimension

	double *A_copy = (double*)malloc(N_ * N_ * sizeof(double));
	for (int i = 0; i < N_*N_; i++)
	{
		A_copy[i] = A_[i];
	}

	double *S = (double*)malloc(N_ * sizeof(double));	// The singular values of A
	double *U = (double*)malloc(N_*N_*sizeof(double));
	double *Vt = (double*)malloc(N_*N_*sizeof(double));
	int	   lwork = 5 * N_;
	double *work = (double*)malloc(lwork*sizeof(double));
	int	   info = 1;

	dgesvd_(jobu, jobvt, &N_, &N_, A_copy, &lda, S, U, &N_, Vt, &N_, work, &lwork, &info);
	assert(0 == info);

	int rank = 0;
	std::vector<double> compare;
	double max_tmp = 0;

	for (int i = 0; i < N_; i++)
	{
		compare.push_back(S[i]);
		if (S[i] > max_tmp)
		{
			max_tmp = S[i];
		}
	}
	double max = *std::max_element(compare.begin(), compare.end());

	cout << "-----------------" << endl;
	cout << "Ill Condition Detector Rank Stat" << endl;
	for (int i = 0; i < compare.size(); i++)
	{
		if (10-15 * max < compare[i])
		{
			rank++;
		}
	}

	Statistics s_svd("SVD_IllCond", compare);
	s_svd.GenerateStdVecFile();

	std::cout << "Rank of K_ : " << rank << std::endl;
	std::cout << "ColN of K_ : " << N_ << std::endl;
	
	char  uplo[] = "U";

	//Eigen::MatrixXd l_p;
	//l_p.resize(N_, N_);
	//for (int i = 0; i < N_; i++)
	//{
	//	for (int j = 0; j < N_; j++)
	//	{
	//		l_p(i, j) = A_[MAP(i, j, N_)];
	//		if (0 == l_p(i, j) && i == j)
	//		{
	//			cout << "leading minor of 1 at index " << i << " = 0" << endl;
	//		}
	//	}
	//}
	//Statistics s_pA("A", l_p);
	//s_pA.GenerateMatrixFile();

	// Since a mechanism results in a singular stiffness matrix, an attempt to find its Cholesky factor
	// is likely to break down and hence signal the problem.
	dpotrf_(uplo, &N_, A_, &lda, &info);

	//Eigen::MatrixXd l;
	//l.resize(N_, N_);
	//for (int i = 0; i < N_; i++)
	//{
	//	for (int j = 0; j < N_; j++)
	//	{
	//		if (i <= j)
	//		{
	//			l(i, j) = A_[MAP(i, j, N_)];
	//		}
	//		else
	//		{
	//			l(i, j) = 0; 
	//			A_[MAP(i, j, N_)] = 0;
	//		}
	//	}
	//}
	//Statistics s_l("l_new", l);
	//s_l.GenerateMatrixFile();

	cout << "info : " << info << endl;
	assert(info == 0);

	//double workcon;
	//int    lworkcon;
	//dpocon_(uplo, &N_, A_, &lda, &Anorm_, &cond_num_, &workcon, &lworkcon, &info);

	//assert(info == 0);

	//std::cout << "condition number of K_ : " << 1/cond_num_ << std::endl;
	free(A_copy);
	free(S);
	free(U);
	free(Vt);
	free(work);
}

bool IllCondDetector::StabAnalysis()
{
	// Condition number computing

	// compute ns_ number of smallest eigenvalues, lambda_1, ..., lambda_ns,
	// and nl_ number of largest eigenvalues of stiffness matrix K
	// normalise associated eigenvectors

	// With the smallest eigenpairs: determine if a gap exists, i.e., if there is a k < ns_,
	// s.t. lambda_{k-1}/lambda_{k} > gap_ * lambda_k/lambda_{k+1}
	return true;
}


void IllCondDetector::Debug()
{
	/* 3x3 matrix A
	* 76 25 11
	* 27 89 51
	* 18 60 32
	*/
	double A[9] = { 76, 27, 18, 25, 89, 60, 11, 51, 32 };
	double b[3] = { 10, 7, 43 };

	int N = 3;
	int nrhs = 1;
	int lda = 3;
	int ipiv[3];
	int ldb = 3;
	int info;

	dgesv_(&N, &nrhs, A, &lda, ipiv, b, &ldb, &info);

	if (info == 0) /* succeed */
		printf("The solution is %lf %lf %lf\n", b[0], b[1], b[2]);
	else
		fprintf(stderr, "dgesv_ fails %d\n", info);

}
#include "IllCondDetector.h"

// mapping from two-dim index to packed 'Up' column major layout storage
#define MAP(i,j) (i + j*(j+1)/2)

IllCondDetector::IllCondDetector(EigenSp const &K)
{
	EIGEN_LAP(K);
}

void IllCondDetector::EIGEN_LAP(EigenSp const &K)
{
	// Convert Eigen library storage into LAPACK storage
	// We use packed column major layout for LAPACK matrix layout, with size n*(n+1)/2
	
	// uplo = "U", for 0 <= i <= j <= n-1, k(i,j) = i + j(j + 1)/2
	N_ = K.cols();
	A_ = (double*)calloc(N_*(N_+1)/2, sizeof(double));

	for (int i = 0; i < N_; i++)
	{
		for (int j = i; i < N_; j++)
		{
			A_[MAP(i, j)] = K.coeff(i, j);
		}
	}
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
	double *L = (double *)calloc(N_*(N_+1)/2, sizeof(double));
	for (int i = 0; i < N_*(N_ + 1) / 2; i++)
	{
		L[i] = A_[i];
	}
	
	char  *uplo = "U";
	int   n = N_;
	int	  lda = N_;
	int	  info;
	dpotrf_(uplo, &n, L, &lda, &info);
	
	delete L;
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
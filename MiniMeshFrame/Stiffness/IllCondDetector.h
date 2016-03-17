#pragma once
// This file implement the model stability analysis algorithm, addressed in
// "Detecting the Causes of Ill-Conditioning in Structural Finite Element Method", Ram. K et. al, July 2013.
// Check http://eprints.ma.man.ac.uk/1997/01/covered/MIMS_ep2013_35.pdf for detail of their paper.
// About LAPACK, doc  : http://www.netlib.org/lapack/explore-html/d0/da2/spotrf_8f.html#aaf31db7ab15b4f4ba527a3d31a15a58e
//			     info : https://software.intel.com/zh-cn/node/521001
// Author  :Yijiang Huang. 
// Contact :duckie@mail.ustc.edu.cn
// Nov. 2015

#ifndef ILLCONDDECTOR_H
#define ILLCONDDECTOR_H

#include <iostream>
#include <stdlib.h>
#include "assert.h"
#include <Eigen/Sparse>

#include "I_O\Statistics.h"

extern "C" void dgesv_(const int *N, const int *nrhs, double *A, const int *lda, int
	*ipiv, double *b, const int *ldb, int *info);

extern "C" void dgels_(const char *trans, const int *M, const int *N, const int *nrhs,
	double *A, const int *lda, double *b, const int *ldb, double *work,
	const int * lwork, int *info);

// Computes the Cholesky factorization of a real symmetric
// positive definite matrix A.
// A = Ut * U, for uplo = "U"
// A = L * Lt, for uplo = "L"
extern "C" void dpotrf_(const char *UPLO, const int *N, double *A, const int *LDA, int *info);

// Estimates the reciprocal of the condition number (in the 1 - norm)
// of a real symmetric positive definite packed matrix using
// the Cholesky factorization A = U**T*U or A = L*L**T computed by
// DPPTRF.
// refer : http://www.netlib.org/lapack/explore-html/d0/d9b/dppcon_8f.html#a8107a68e3c7d948fe246bf0feae0470b
/*
* Note by Y.J. Huang @Mar/15/2016
* I don't know why this routine fails for stiffness funtion (bunny head 60 unrestrained nodes)
* but works fine for simple example in debug.
*
* [BugFix @Mar/15/2016] Please specify WORK and LWORK as:
* 	double *workcon  = (double*)malloc(3 * N_ * sizeof(double));
*   int    *lworkcon = (int*)malloc(N_ * sizeof(int));
*/
extern "C" void dpocon_(const char *UPLO, const int *N, const double *AP, const int *lda, const double *ANORM,
	double *RCOND, double *WORK, int *LWORK, int *INFO);

// Rank computing, the rank of input matrix is the number of singular values that are not zero
// A = U * SIGMA * transpose(V)
// refer : http://icl.cs.utk.edu/lapack-forum/viewtopic.php?f=2&t=818
extern "C" void dgesvd_(const char *JOBU,  const char *JOBVT, const int *M, const int *N, double *A, const int *LDA,
	double *S, double *U, const int *LDU, double *VT, const int *LDVT, double *WORK, const int *LWORK, int *info);

class IllCondDetector{
public:
	typedef Eigen::SparseMatrix<double> EigenSp;

public:
	IllCondDetector(){};
	IllCondDetector(EigenSp const &K);
	~IllCondDetector();

public:
	// I\O
	void inline SetParm(int _ns, int _nl, int _condthres, int _gap);
	double GetCondNum() const { return rcond_num_; }
	
	// Library Compatibility
	void EigenLap(EigenSp const &K);

	void ComputeCondNum();
	bool StabAnalysis();
	
	void Debug();
private:
	int			ns_;			// ns_ : The number of smallest eigenpairs
	int			nl_;			// nl_ : The number of largest  eigenpairs
	int			cond_thres_;	// cond_thres_ : The condition number threshold for triggering analysis
	int			gap_;			// gap_ : The order of the gap between a cluster of smallest eigenvalues
								// and the next largest eigen values

	/*
	* Matrices are well-conditioned if the 
	* reciprocal condition number is near 1 and ill-conditioned if it is near zero.
	*
	* About numerical value of condition number, please refer to:
	* http://math.stackexchange.com/questions/675474/what-is-the-practical-impact-of-a-matrixs-condition-number
	* "Condition number exceeds 10e10 could be problematic. condition number from 10e^3~6 could be acceptable."
	*/
	int			N_;				// N_       : the matrix's row number
	double		rcond_num_;		// rcond_num :the reciprocal of the condition number
	double		*A_;			// A[]		: LAPACK storage of the matrix
	double		Anorm_;			// Anorm_	: 1 norm = max_j{ sum_{i}abs(a_ij)}
};

#endif
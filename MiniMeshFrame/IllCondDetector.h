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

#include "assert.h"
#include <Eigen/Sparse>

extern "C" void dgesv_(const int *N, const int *nrhs, double *A, const int *lda, int
	*ipiv, double *b, const int *ldb, int *info);

extern "C" void dgels_(const char *trans, const int *M, const int *N, const int *nrhs,
	double *A, const int *lda, double *b, const int *ldb, double *work,
	const int * lwork, int *info);

extern "C" void dpotrf_(const char* UPLO, const int *N, double* A, const int *lda, int *info);

extern "C" void dppcon(const char *UPLO, const int *N, double* AP, ANORM, RCOND, WORK, IWORK, INFO)
class IllCondDetector{
public:
	typedef Eigen::SparseMatrix<double> EigenSp;

public:
	IllCondDetector(){};
	IllCondDetector(EigenSp const &K);
	~IllCondDetector(){};

public:
	// I\O
	void inline SetParm(int _ns, int _nl, int _condthres, int _gap);
	double GetCondNum() const { return cond_num_; }
	
	// Library Compatibility
	void EIGEN_LAP(EigenSp const &K);

	void ComputeCondNum();
	bool StabAnalysis();
	
	void Debug();
private:
	int			ns_;			// ns_ : The number of smallest eigenpairs
	int			nl_;			// nl_ : The number of largest  eigenpairs
	int			cond_thres_;	// cond_thres_ : The condition number threshold for triggering analysis
	int			gap_;			// gap_ : The order of the gap between a cluster of smallest eigenvalues
								// and the next largest eigen values

	int			N_;				// N_       : the matrix's row number
	double		cond_num_;		// cond_num : The condition number
	double		*A_;				// A[]		: LAPACK storage of the matrix
};

#endif
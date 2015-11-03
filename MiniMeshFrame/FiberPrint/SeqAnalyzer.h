#pragma once

#include "SeqAnalyzer.h"
#include <Eigen\Sparse>

#include "LPMosek.h"
#include "LPFactory.h"

#include "TSPLIB_Loader.h"
#include "Statistics.h"

class SeqAnalyzer
{
public:
	typedef Eigen::SparseMatrix<double> SpMat;
	typedef Eigen::MatrixXd				MX;
	typedef Eigen::VectorXd				VX;

public:
	SeqAnalyzer();
	~SeqAnalyzer();

	// Traveling Salesman Problem (TSP) solver
	void TSPSolver();

	void debug();

private:
	VX						x_;		// label varible for edge of a complete graph, dimension : Nd*Nd
	
	LinearP					*lp;	// solve the lp problem min c^t * x subject to lc < A * x < uc, x >= lb, x <= ub
};


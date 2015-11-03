#pragma once

#include <iostream>

#include <Eigen/dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCore>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseLU>
#include <Eigen/SparseQR>
#include <Eigen/Core>

#include "WireFrame\WireFrame.h"
#include "Stiffness.h"

#include "QPMosek.h"
#include "QPFactory.h"
#include "Statistics.h"

#include "LPFactory.h"
#include "TSPLIB_Loader.h"

using namespace std;
using namespace Eigen;


class GraphCut
{

public:
	typedef Eigen::SparseMatrix<double> SpMat;
	typedef Eigen::MatrixXd MX;
	typedef Eigen::Matrix3d M3;
	typedef Eigen::VectorXd VX;
	typedef Eigen::Vector3d V3;

	GraphCut();
	GraphCut(WireFrame *ptr_frame);
	~GraphCut();

public:
	//Initialization
	void		CreateAandC();						// Construct edge-incidence matrix A and weight diagonal matrix C
	void		SetStartingPoints();				// Set D and lambda variable's starting value
	void		SetBoundary(VX &d, SpMat &W, int count);
	void		UpdateX();

	//Termination
	bool		CheckLabel(int iter_count);			// Stopping Criterion for iteratively apply ADMM to find several cuts
	bool		TerminationCriteria();				// Termination Criteria for ADMM process of a single cut using a threshold node number

	//ADMM
	void		MakeLayers();						// Main loop of ADMM
	void 		CalculateQ(const VX _D, SpMat &Q);	// Calculate Q for x_Qp problem
	void		CalculateX(VX &d, SpMat &W);		// QP optimization for x at every iteration
	void		CalculateD();						// QP optimization for D at every iteration
	void		UpdateLambda();						// Dual variable update at every iteration

	//Debug
	void		debug();

	vector<DualVertex*>		*GetDualVertexList();
	VectorXi				*GetLabel();

public:
//private:
	WireFrame		*ptr_frame_;
	DualGraph		*ptr_dualgraph_;
	Stiffness		*ptr_stiff_;	// Store 3*3 stiffness and caluculate weighted global stiffness matrix

	int				N_;				// N : Number of nodes in orig graph
	int				M_;				// M : Number of edges in orig graph 
	int				Nd_;			// Nd : Number of node in dual graph
	int				Md_;			// Md : number of edges in dual graph

	double			penalty_;		// penalty  : penalty factor used in ADMM  
	double			D_tol_;			// D_tol    : tolerance in D-Qp problem constraints
	int				stop_n_;		// stop_n   : termination criteria for GraphCut process, number of dual nodes in LowerSet
	double			pri_tol_;		// pri_tol  : primal residual tolerance for ADMM termination criterion
	double			dual_tol_;		// dual_tol : dual   residual tolerance for ADMM termination criterion

	bool			debug_;

	SpMat			A_;
	SpMat			C_;
	VX				x_;
	VX				D_;
	VX				lambda_;
	VX				a_;				// linear coefficient used in x_Qp
	VectorXi		layer_label_;	// passed to render

	VX				dual_res;		// dual residual for ADMM termination criteria
	VX				primal_res;		// dual residual for ADMM termination criteria

	QP				*qp;			// Solves the quadratic programming problem:
									// min 0.5* xt*H*x + ft*x subject to A*x <= b, C*x = d, x >= lb, x <= ub
	SpMat			*H1;			// Part 1 of hessian matrix for x-Qp problem
};

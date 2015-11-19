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
#include "FiberPrintPARM.h"

#include "QPMosek.h"
#include "QPFactory.h"
#include "Statistics.h"

using namespace std;
using namespace Eigen;


class GraphCut
{
	typedef Eigen::SparseMatrix<double> SpMat;
	typedef Eigen::MatrixXd MX;
	typedef Eigen::Matrix3d M3;
	typedef Eigen::VectorXd VX;
	typedef Eigen::Vector3d V3;

public:
	GraphCut();
	GraphCut(WireFrame *ptr_frame);
	GraphCut(WireFrame *ptr_frame, FiberPrintPARM *ptr_parm);
	~GraphCut();

public:
	//Initialization
	void		InitState();
	void		SetStartingPoints(int count);		// Set D and lambda variable's starting value
	void		CreateAandC();						// Construct edge-incidence matrix A and weight diagonal matrix C
	void		SetBoundary(VX &d, SpMat &W);

	//Termination
	bool		CheckLabel(int iter_count);			// Stopping Criterion for iteratively apply ADMM to find several cuts
	bool		TerminationCriteria();				// Termination Criteria for ADMM process of a single cut using a threshold node number

	//ADMM
	void		MakeLayers();						// Main loop of ADMM
	void		CalculateX(VX &d, SpMat &W);		// QP optimization for x at every iteration
	void 		CalculateQ(const VX _D, SpMat &Q);	// Calculate Q for x_Qp problem
	void		CalculateD();						// QP optimization for D at every iteration
	void		UpdateLambda();						// Dual variable update at every iteration
	void		UpdateCut();

	vector<DualVertex*>		*GetDualVertList()		{ return ptr_dualgraph_->GetVertList(); }
	vector<DualEdge*>		*GetDualEdgeList()		{ return ptr_dualgraph_->GetEdgeList(); }
	vector<DualFace*>		*GetDualFaceList()		{ return ptr_dualgraph_->GetFaceList(); }

	vector<int>				*GetLabel()				{ return &layer_label_; }
	vector<int>				*GetCut()				{ return &cutting_edge_; }

	void Debug();

public:
//private:
	WireFrame		*ptr_frame_;
	DualGraph		*ptr_dualgraph_;
	Stiffness		*ptr_stiff_;	// Store 3*3 stiffness and caluculate weighted global stiffness matrix

	SpMat			A_;
	SpMat			C_;
	VX				x_;
	VX				D_;
	VX				lambda_;
	VX				a_;				// linear coefficient used in x_Qp
	vector<int>		layer_label_;	// passed to render
	vector<int>		cutting_edge_;

	VX				dual_res_;		// dual residual for ADMM termination criteria
	VX				primal_res_;	// dual residual for ADMM termination criteria

	QP				*qp_;			// Solves the quadratic programming problem:
									// min 0.5* xt*H*x + ft*x subject to A*x <= b, C*x = d, x >= lb, x <= ub
	SpMat			H1_;			// Part 1 of hessian matrix for x-Qp problem

	int				N_;				// N :    Number of nodes in orig graph
	int				M_;				// M :    Number of edges in orig graph 
	int				Nd_;			// Nd :   Number of node in dual graph
	int				Md_;			// Md :   Number of edges in dual graph
	int				Fd_;			// Fd :   Number of faces in dual graph
	int				Nd_w_;		    // Nd_w_: Number of nodes in WHOLE dual graph 

	int				stop_n_;		// stop_n   : termination criteria for GraphCut process, number of dual nodes in LowerSet
	double			penalty_;		// penalty  : penalty factor used in ADMM  
	double			D_tol_;			// D_tol    : tolerance in D-Qp problem constraints
	double			pri_tol_;		// pri_tol  : primal residual tolerance for ADMM termination criterion
	double			dual_tol_;		// dual_tol : dual   residual tolerance for ADMM termination criterion

	bool			debug_;
};


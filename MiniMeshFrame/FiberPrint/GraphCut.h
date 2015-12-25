/*
* ==========================================================================
*
*       class: GraphCut
*
*    Description:  This file is a part of implementation fo paper "WirePrint : a fast&stable way to fabricate wireframe"
*				   The GraphCut submodule takes charge of dividing the wireframe into several structually-stable sections, 
*				   scaling the problem down, enabling further tool path searching part numerically tractable.
*
*	 Version:  1.0
*	 Created:  Oct/10/2015
*    Updated:  Nov/02/2015
*
*	 Author:   Xin Hu, Yijiang Huang, Guoxian Song
*	 Company:  GCL@USTC
*
*    WARNING:  DO NOT insert node and edge after you dualize the graph, 
*    as we append all project fixed point at the end of dual face.
*    Further inserting will cause stiffness matrix and force creation error.
* ==========================================================================
*/

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
#include "Stiffness\Stiffness.h"
#include "FiberPrintPARM.h"

#include "QP/QPMosek.h"
#include "QP/QPFactory.h"
#include "I_O\Statistics.h"

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
	GraphCut(WireFrame *ptr_frame, FiberPrintPARM *ptr_parm, char *path);
	~GraphCut();

public:
	//Initialization
	void		InitState();
	void		SetStartingPoints(int count);		// Set D and lambda variable's starting value
	void		SetBoundary();
	void		CreateA();							// Construct edge-incidence matrix A
	void		CreateC(int cut, int rew);					
    // Construct weight diagonal matrix C and H1

	//Termination
	bool		CheckLabel(int count);				// Stopping Criterion for iteratively apply ADMM to find several cuts
	bool		TerminationCriteria(int count);		// Termination Criteria for ADMM process of a single cut using a threshold node number

	//ADMM
	void		MakeLayers();						// Main loop of cut
	void		CalculateX();						// QP optimization for x at every iteration
	void 		CalculateQ(const VX _D, SpMat &Q);	// Calculate Q for x_Qp problem
	void		CalculateD();						// QP optimization for D at every iteration
	void		UpdateLambda();						// Dual variable update at every iteration
	void		UpdateCut();
	bool		UpdateR(VX &x_prev, int count);

	vector<DualVertex*>		*GetDualVertList()		{ return ptr_dualgraph_->GetVertList(); }
	vector<DualEdge*>		*GetDualEdgeList()		{ return ptr_dualgraph_->GetEdgeList(); }
	vector<DualFace*>		*GetDualFaceList()		{ return ptr_dualgraph_->GetFaceList(); }
	vector<int>				*GetCut()				{ return &cutting_edge_; }

	void		WriteWeight();
	void		WriteStiffness();
	void		Debug();

public:
//private:
	WireFrame		*ptr_frame_;
	DualGraph		*ptr_dualgraph_;
	Stiffness		*ptr_stiff_;	// Store 3*3 stiffness and caluculate weighted global stiffness matrix
	char			*path_;

	SpMat			A_;
	SpMat			C_;
	MX				r_;				// for updation of C
	VX				x_;
	VX				D_;
	VX				lambda_;
	VX				a_;				// linear coefficient used in x_Qp
	vector<int>		cutting_edge_;

	VX				d_;				// for setting boundary & QP x
	SpMat			W_;

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
    int             Ns_;
	int				Nd_w_;		    // Nd_w_: Number of nodes in WHOLE dual graph 

	int				stop_n_;		// stop_n   : termination criteria for GraphCut process, number of dual nodes in LowerSet
	double			Dt_tol_;		// Dt_tol   : tolerance of offset in stiffness
	double			Dr_tol_;		// Dr_tol   : tolerance of rotation in stiffness
	double			penalty_;		// penalty  : penalty factor used in ADMM  
	double			pri_tol_;		// pri_tol  : primal residual tolerance for ADMM termination criterion
	double			dual_tol_;		// dual_tol : dual   residual tolerance for ADMM termination criterion

	bool			debug_;
	bool			matlab_;
	bool			maya_;
};


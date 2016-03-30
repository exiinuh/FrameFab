/*
* ==========================================================================
*
*       class: Stiffness
*
*    Description:  This class is a part of FiberPrint, which takes in charge of 
*				   stiffness matrix related problem arised in GraphCut submodule.
*
*	 Capability :  Create elastic Stiffness matrix
*				   Calculate displacement for gravity force involved only frame structure
*				   Output Gnuplot file for explicit displacement display
*				   Output .3dd file for frame3dd analysis
*
*	 Version:  1.0
*	 Created:  Oct/15/2015 by Xin Hu
*	 Updated:  Mar/30/2016 by Xin Hu
*
*	 Author:   Xin Hu,  Yijiang Huang, Guoxian Song
*	 Company:  GCL@USTC
*	 Note:	   The core stiffness matrix construction part of this file 
*			is modified from frame3dd_io.c, which is a part of Frame3dd.
*			You can get original C file of Frame3dd from http://frame3dd.sourceforge.net/.
*
*			You can download Gnuplot at	http://sourceforge.net/projects/gnuplot/
*			
*			About the theory of matrix analysis of structure, please refer to 
*			Kassimali A. Matrix Analysis of Structures SI Version[M]. Cengage Learning, 2011.
*			
*			The structures discussed in this software are space frames, where axial forces,
*			bending moments and torsional forces(torque) are all taken into consideration.
*			The elements have common CIRCULAR cross-sections.
*		
*			If you don't know whether shear deformation should be taken into consideration,
*			check Section 9.7 [SHEAR DEFORMATION] of Kassimali's book.
* ==========================================================================
*/


#ifndef FIBERPRINT_STIFFNESS_H
#define FIBERPRINT_STIFFNESS_H

#include <iostream>
#include <assert.h>

#include <Eigen/dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCore>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseLU>
#include <Eigen/SparseQR>
#include <Eigen/Core>
#include <Eigen/OrderingMethods>
#include <Eigen/IterativeLinearSolvers>

#include "WireFrame\WireFrame.h"
#include "FiberPrint\DualGraph.h"
#include "FiberPrint\FiberPrintPARM.h"
#include "CoordTrans.h"

#include "GCommon.h"
#include "StiffnessIO.h"
#include "StiffnessSolver.h"
#include "IllCondDetector.h"

using namespace std;
using namespace Eigen;

class Stiffness
{
public:
	typedef Eigen::SparseMatrix<double> SpMat;
	typedef Eigen::MatrixXd				MX;
	typedef Eigen::VectorXd				VX;
	typedef Eigen::VectorXi				VXi;
	typedef	Eigen::MatrixXi				MXi;
	typedef trimesh::point				point;

public:
	Stiffness();
	Stiffness(DualGraph *ptr_dualgraph);
	Stiffness(DualGraph *ptr_dualgraph, FiberPrintPARM *ptr_parm);
	~Stiffness();

public:
	void		Init();
	void		CreateFe();
	void		CreateF(const VectorXd &x);
	void		CreateElasticK();
	void		CreateGlobalK(const VectorXd &x);

	/* Socket to GraphCut */
	bool		CalculateD(VectorXd &D);
	bool		CalculateD(VectorXd &D, const VectorXd &x, 
							int write_matrix, int write_3dd, int cut_count);

	/* Socket to SeqAnalyzer */
	bool		CalculateD(VectorXd &D, VectorXd &D0);
	bool		CalculateD(VectorXd &D, VectorXd &D0, const VectorXd &x, 
							int verbose, int write_data, int seq_id);

	/* Check condition number */
	bool		CheckIllCondition(IllCondDetector &stiff_inspector, int verbose);
	bool		CheckError(IllCondDetector &stiff_inspector, VX &D, int verbose);

	/* Debug */
	void		WriteData(VectorXd &D, int verbose, int id, char *fname);

	/* Data I/O */
	SpMat		*WeightedK(){ assert(&K_); return &K_; }
	VX			*WeightedF(){ assert(&F_); return &F_; }

	MX			eKe(int ei);			// ei: orig e id
	MX			eKv(int ei);			// ei: orig e id
	VX			Fe(int ei);				// ei: orig e id
	
private:
	//private:
	DualGraph			*ptr_dualgraph_;
	FiberPrintPARM		*ptr_parm_;

	StiffnessIO			stiff_io_;
	StiffnessSolver		stiff_solver_;

	CoordTrans			transf_;

	SpMat			K_;						// x-Weighted global stiffness matrix, 6n*6n
	vector<MX>		eK_;					// elastic K, indexed by dual id
	VX				F_;
	vector<VX>		Fe_;

	int				Ns_;

	double			r_;						// radius of frame
	double			nr_;					// radius of node
	double			density_;
	double			g_;
	double			G_;						// shear modulus
	double			E_;						// young's modulus;
	double			v_;						// possion ratio

	bool			shear_;					// 1 : shear deformation taken into consideration; 0 : not
};
#endif
/*
This file is a part of Project FiberPrint:
Structually stable, collison free robotic tool path generator 
for spatial robotic printing.

This file uses idea and modify some functions from FRAME3DD :
Static and dynamic structural analysis of 2D and 3D frames and trusses with
elastic and geometric stiffness.

More detail about Matrix Structual Analysis, please refer to:
http://people.duke.edu/~hpgavin/cee421/
We use naming convention in Frame3dd, more info at: 
http://svn.code.sourceforge.net/p/frame3dd/code/trunk/doc/Frame3DD-manual.html
-------------------------------------------------------------------------- -
http://frame3dd.sourceforge.net/
-------------------------------------------------------------------------- -
Copyright(C) 1992 - 2014  Henri P.Gavin

Author : Xin Hu, Guoxian S. and Yijiang H.
@file  : Calculate weighted stiffness matrix for frame structure.
Time   : Nov. 2015.

Note:
--Nov18--		We append .3dd I\O to check our stiffness matrix's validity.
				The input file format for FRAME is defined in doc/user_manual.html(website above)
*/

#pragma once

#include <iostream>
#include <assert.h>

#include <Eigen/dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCore>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseLU>
#include <Eigen/SparseQR>
#include <Eigen/OrderingMethods>
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>

#include "WireFrame\WireFrame.h"
#include "DualGraph.h"
#include "FiberPrintPARM.h"

#include "I_O\Statistics.h"
#include "Stiffness\CoordTrans.h"

#include "Stiffness\GCommon.h"
#include "Stiffness\GUtil.h"

using namespace std;
using namespace Eigen;

class Stiffness
{
public:
	typedef		Eigen::MatrixXd MX;
	typedef		Eigen::Matrix3d M3;
	typedef		Eigen::VectorXd VX;
	typedef		Eigen::Vector3d V3;
	typedef		Eigen::VectorXi VXi;
	typedef		Eigen::MatrixXi MXi;

	Stiffness();
	Stiffness(WireFrame *ptr_frame, DualGraph *ptr_dualgraph, FiberPrintPARM *ptr_parm);
	~Stiffness();

public:
	// Data I/O
	void		ReadNodeData();				// ReadNodeData		 : Read node coordinate data
	void		ReadElementData();			// ReadFrameData	 : Read frame element property data
	void		ReadReactionData();			// ReadReactionData  : Read fixed node displacement boundary conditions
	void		ReadAssembleLoads();		// ReadAssembleLoads : Reaed load information data, form un-restrained load vector
	void		ReadMassData();				// ReadMassData      : Read member densities and extra inertial mass data
	
	void		GetLineNoComment(FILE *fp, char *s, int lim);
	// -> interface to .3dd file


	// Gnuplot data display
	void		GPlt_StaticMesh();			// GPltStaticMesh	  : create mesh data of deformed and undeformed mesh, use gnuplot
											// useful gnuplot options : set noxtics noytics noztics noborder view nokey
	void		GPlt_CubicBentBeam();		// GPlt_CubicBentBeam :   *	computes cubic deflection functions from end deflections
											// and end rotations.Saves deflected shapes to a file.
											// These bent shapes are exact for mode - shapes, and for frames
											// loaded at their nodes.

	void		ReadRunData();				// ReadRunData   : Read data controlling certain aspects of the analysis


	// Stiffness Matrix Calculation
	void		ElasticK(MX &k, int ElemId ,int shear);		// ElasticK : space frame element stiffness matrix in global coordinates				

	void		AssembleK();								// AssembleK : assemble global stiffness matrix from individual elements
	
	void		SolveSystem();								// SolveSystem : solve {F} =   [K]{D} via L D L' decomposition
															// Prescribed displacements are "mechanical loads" not "temperature loads"


public:
	//private:
	WireFrame				*ptr_frame_;
	DualGraph				*ptr_dualgraph_;

	// Stiffness Assembly
	MXi						ind_;					// ind_:		Member-structure DoF index table, dimension: 12 * nE_ 	
	VXi						N1_,N2_;				// N1_[i]:		store the index of endnode		


	std::vector<V3>			xyz_;					// xyz_:		X,Y,Z node coordinates (global)
	VX						rj_;					// rj_[i] :		Node size radius, for finite size
	VX						L_;						// L :			Node-to-node length of each element
	VX						Le_;					// Le_ :		Effective length, account for node size
	VX						Ax_;					// Ax_[i]:		Cross-sectional area of a prismatic frame element
													// (The x - axis is along the element length, in local coordinates)
	VX						Asy_, Asz_;				// Asy_[i]:		Shear area in the local y-axis(z-axis) of a prismatic frame element
	VX						Jx_;					// Jx_[i]:		Torsional moment of inertia of a frame element
	VX						Iy_, Iz_;				// Iy_[i]:		Moment of inertia for bending about the local y axis
	VX						E_;						// E_[i]:		Modulus of elasticity of a frame element
	VX						G_;						// G_[i];		Shear modulus of elasticity of frame element i
	VX						p_;						// p_[i]:		The roll angle of the frame element, in degrees
	VX						d_;						// d_[i]:		Element mass density

	MX						K_;						// K_ :			Global stiffness matrix, dimension: Dof_ * Dof_
	VX						D_;						// D_ :			Displacement vector
	VX						R_;						// R_ :			Total reaction force vector
		
	int						DoF_;					// Dof_:		total number of degrees of freedom
	int						nE_;					// nE_:			number of elements(edges) in the frame
	int						nN_;					// nN_:			number of nodes in the frame

	CoordTrans				trsf_;
	
	// Option in Matrix Structual Analysis
	int						shear_;					// shear_:      1: include shear deformations, 0: do not.
	int						geom_;					// geom_:		1: include geometric stiffness effects, 0: do not.
	
	// Options for Gnuplot display
	int						pan_ = 1.0;				// pan :		>0	pan during animation; 0: don't
	int						scale_ = 1.0;			// scale_ :		zoom scale for 3D plotting in Gnuplot
	int						dx_ = 1.0;				// dx_ :		x-increment for internal force data

	// Path define
	char					*IN_file_;				// IN_file :	the input  data filename
	char					*OUT_file_;				// OUT_file:    the output data filename
	char					*title_;				// title :		the title of the analysis
	char					*errMsg_;				// errMsg :		the text of an error message
	char					*meshpath_;				// meshpath:	mesh data path
	char					*plotpath_;				// plotpath :	plot file path
	char					*infcpath_;				// infcpath_ :  int  file path
};

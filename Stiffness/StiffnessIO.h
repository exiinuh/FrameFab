/*
* ==========================================================================
*
*		class:	StiffnessIO
*
*		This file is part of the implementation of
*
*		<Sustainable Fabrication of Frame Shapes>
*		Yijiang Huang, Juyong Zhang, Xin Hu, Guoxian Song, Zhongyuan Liu, Lei Yu, Ligang Liu
*		In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2016)
*
*		Description: This module takes charge of outputting stiffness matrix related results,
*
*		Version:  2.0
*		Created: Oct/10/2015
*		Updated: Aug/24/2016
*
*		Author:  Yijiang Huang, Xin Hu
*		Company:  GCL@USTC
* ==========================================================================
*/

#ifndef STIFFNESS_IO_H
#define STIFFNESS_IO_H

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <Eigen/dense>

#include "CoordTrans.h"
#include "I_O\Statistics.h"

#include "GlobalFunctions\GCommon.h"
#include "StiffnessSolver.h"

#include "FiberPrint\FiberPrintPARM.h"
#include "FiberPrint\DualGraph.h"
#include "CoordTrans.h"

#define MYOUT std::cout
#define MYEND std::endl
class StiffnessIO
{
public:
	typedef Eigen::MatrixXd MX;
	typedef Eigen::Matrix3d M3;
	typedef Eigen::VectorXd VX;
	typedef Eigen::Vector3d V3;
	typedef Eigen::VectorXi VXi;
	typedef	Eigen::MatrixXi MXi;

	typedef std::vector<V3> vec3;

public:
	StiffnessIO(){};
	~StiffnessIO(){};

public:

	void	OutputPath(const char *fname, char fullpath[], const int len, char *default_outdir, int verbose);

	/*--- GnuPlot file output ---*/
	/*
	* GnuPltStaticMesh - create mesh data of deformed and undeformed mesh, use gnuplot	 Mar/20/2016
	*/
	void GnuPltStaticMesh(
		const char *fpath,
		const char *meshpath, const char *plotpath,
		VX &D,
		double exagg_static, float scale,
		DualGraph *ptr_dualgraph, WireFrame *ptr_frame
		);

	/*
	* GnuPltCubicBentBeam -											Mar/20/2016
	*	computes cubic deflection functions from end deflections
	*	and end rotations.  Saves deflected shapes to a file.
	*	These bent shapes are exact for mode-shapes, and for frames
	*	loaded at their nodes.
	*/
	void GnuPltCubicBentBeam(
		vector<point> &beam,
		VX &D,			/* calculated deformation */
		int dual_i, DualGraph *ptr_dualgraph, WireFrame *ptr_frame, 
		double exagg
		);

	/*
	* WriteInputData - write input data to a .3dd file			Mar/20/2016
	*/
	void WriteInputData(
		const char *fpath, 
		DualGraph *ptr_dualgraph, 
		FiberPrintPARM *ptr_parm, 
		int verbose
		);

	/*
	* SaveUpperMatrix - save a symmetric matrix of dimension [1..n][1..n]	Nov/26/2015
	* to the named file, use only upper-triangular part
	*/
	void SaveUpperMatrix(char filename[], const MX &A, int n);
	
	/*
	* SaveDeformVector - save displacement vector of dimemsion [1...6*N]	Nov/26/2015
	* to the named file
	*/		;
	void SaveDisplaceVector(char filename[], const VX &D, int n, DualGraph *ptr_dual_graph);
	
public:
	void	dots(FILE *fp, int n)
	{
		int i;
		for (i = 1; i <= n; i++)	fprintf(fp, ".");
	}

	const char *TempDir()
	{
		char *tmp;
		tmp = getenv("TEMP");
		if (tmp == NULL) {
			fprintf(stderr,
				"ERROR: Environment Variables %%TEMP%% and %%FRAME3DD_OUTDIR%% are not set.  "
				"At least one of these variables must be set so that FrameFab knows where to "
				"write its temporary files.  Set one of these variable, then re-run FrameFab.");
			exit(15);
		}
		return tmp;
	}

private:
	CoordTrans		trsf_;
	StiffnessSolver	solver_;	// solver_: LU decomposition for cubic bent beam computaion
};
#endif
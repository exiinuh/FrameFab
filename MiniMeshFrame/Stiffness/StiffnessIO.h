/*
* ==========================================================================
*
*       class: StiffnessIO
*
*    Description:  create mesh data of deformed and undeformed mesh for GnuPlot.
*
*	 Version:  1.0
*	 Created:  Nov/25/2015
*
*	 Author:   Yijiang Huang, Xin Hu, Guoxian Song
*	 Company:  GCL@USTC
*	 Note:	   This file is modified from frame3dd_io.c, which is a part of Frame3dd.
*			You can get original C file of Frame3dd from http://frame3dd.sourceforge.net/.
*			You can download Gnuplot at	http://sourceforge.net/projects/gnuplot/
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

#include "GCommon.h"
#include "GUtil.h"
#include "StiffnessSolver.h"

#include "FiberPrint\FiberPrintPARM.h"
#include "FiberPrint\DualGraph.h"

/* maximum number of load cases */
#define _NL_ 32
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

	int			StiffnessGetline(FILE *fp, char *s, int lim);
	void		GetlineNoComment(FILE *fp, char *s, int lim);
	void		ParseInput(FILE *fp, const char *tpath);
	void		OutputPath(const char *fname, char fullpath[], const int len, char *default_outdir);

	void ReadRunData(
		char OUT_file[],	 /**< output data file name							*/
		char meshpath[],	 /**< file name for mesh data output				*/
		char plotpath[],	 /**< file name for Gnuplot script					*/
		int  debug
		);

	/* GnuPlot file output */
	/*
	* GnuPltStaticMesh - create mesh data of deformed and undeformed mesh, use gnuplot	 Nov/25/2015
	*/
	void GnuPltStaticMesh(
		char OUT_file[],
		char meshpath[], char plotpath[],
		char *title, int nN, int nE, int nL, int lc, int DoF,
		vec3 &xyz, VX &L,
		VXi &N1, VXi &N2, VX &p, VX &D,
		double exagg_static, int D3_flag, int anlyz, float scale
		);

	/*
	* GnuPltCubicBentBeam -											Nov/25/2015
	*	computes cubic deflection functions from end deflections
	*	and end rotations.  Saves deflected shapes to a file.
	*	These bent shapes are exact for mode-shapes, and for frames
	*	loaded at their nodes.
	*/
	void GnuPltCubicBentBeam(
		FILE *fpm,		/**< deformed mesh data file pointer		*/
		int n1, int n2,	/**< node 1 and node 2 of the frame element */
		vec3   &xyz,	/**< node coordinates						*/
		double L,		/**< frame element lengths					*/
		float  p,		/**< frame element local rotations			*/
		VX	   &D,		/**< node displacements						*/
		double exagg	/**< mesh exaggeration factor				*/
		);

	/*
	* WriteInputData - write input data to a .3dd file			Nov/26/2015
	*/
	void WriteInputData(DualGraph *ptr_dualgraph, FiberPrintPARM *ptr_parm);

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
	
	void Debug(int verbose);		// 1 : copious screenpaly, 0 : none 

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
				"At least one of these variables must be set so that FiberPrint knows where to "
				"write its temporary files.  Set one of these variable, then re-run FiberPrint.");
			exit(15);
		}
		return tmp;
	}
private:
	CoordTrans		trsf_;
	StiffnessSolver	solver_;
};
#endif
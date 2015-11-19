/*
	@file :
	Frame input/output functions (for FRAME#DD native format .3dd)
	Author : Yijiang H.
	Time   : Nov 2015
	
	Note : 
	Modifying from fram3dd_io.h, I introduce Eigen Library to simplify vector and matrix manipulation.
	you can reach Eigen by 
	http://eigen.tuxfamily.org/dox/group__TutorialMatrixClass.html
	you can get original Frame3dd from
	http://frame3dd.sourceforge.net/
*/
#pragma once
#include <stdio.h>
#include <vector>
#include <Eigen/dense>

#include "CoordTrans.h"
#include "Statistics.h"

#include "GCommon.h"
#include "GUtil.h"

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
	void		OutPath(const char *fname, char fullpath[], const char *default_outdir);

	// Stiffness Matrix Data Input
	void ReadNodeData(
		FILE *fp,				/**< input data file pointer		*/
		int nN, 				/**< number of nodes				*/
		vec3 &xyz,				/**< XYZ coordinates of each node	*/
		VX &rj					/**< rigid radius of each node		*/
		);
	
	void ReadFrameElementData(
		FILE *fp,					/**< input data file pointer					*/
		int &nN,					/**< number of nodes							*/
		int &nE,					/**< number of frame elements					*/
		std::vector<VX> xyz,		/**< XYZ coordinates of each node				*/
		VX &rj,						/**< rigid radius of each node					*/
		VX &L, VX &Le,				/**< length of each frame element, effective	*/
		VXi &N1, VXi &N2, 			/**< node connectivity							*/
		VX &Ax, VX &Asy, VX &Asz,	/**< section areas								*/
		VX &Jx, VX &Iy, VX &Iz,		/**< section inertias							*/
		VX &E, VX &G,				/**< elastic moduli and shear moduli			*/
		VX &p,						/**< roll angle of each frame element (radians)	*/
		VX &d						/**< mass density of each frame element			*/
		);

	void ReadRunData(
		FILE *fp,			 /**< input data file pointer						*/
		char OUT_file[],	 /**< output data file name							*/
		int &shear,			 /**< 1: include shear deformations, 0: don't		*/
		int &geom,		   	 /**< 1: include geometric stiffness, 0: don't		*/
		char meshpath[],	 /**< file name for mesh data output				*/
		char plotpath[],	 /**< file name for Gnuplot script					*/
		char infcpath[],	 /**< file name for internal force data				*/
		double &exagg_static,/**< factor for static displ. exaggeration			*/
		double &exagg_flag,	 /**< static exagg. command-line over-ride			*/
		float &scale,		 /**< zoom scale for 3D plotting in gnuplot			*/
		float &dx,			 /**< frame element increment for internal forces	*/
		int &anlyz,			 /**< 1: perform elastic analysis, 0: don't			*/
		int debug			 /**< print debugging information					*/
		);

	void ReadAndAssembleLoads(
		FILE *fp,	/**< input data file pointer			*/
		int nN,		/**< number of nodes				*/
		int nE,		/**< number of frame elements			*/
		int nL,		/**< number of load cases			*/
		int DoF,	/**< number of degrees of freedom		*/
		vec3 *xyz,	/**< XYZ coordinates of each node		*/
		double *L, double *Le,	/**< length of each frame element, effective */
		int *N1, int *N2, 	/**< node connectivity			*/
		float *Ax, float *Asy, float *Asz,	/**< section areas	*/
		float *Iy, float *Iz,	/**< section inertias			*/
		float *E, float *G,	/**< elastic moduli and shear moduli	*/
		float *p,	/**< roll angle of each frame element (radians)	*/
		float *d,  /**< mass density of each frame element		*/
		float *gX, /**< gravitational acceleration in global X each load case */
		float *gY, /**< gravitational acceleration in global Y each load case */
		float *gZ, /**< gravitational acceleration in global Z each load case */
		int *r,		/**< r[i]=1: DoF i is fixed, r[i]=0: DoF i is free */
		int shear,	/**< 1: include shear deformations, 0: don't	*/
		int *nF, 		/**< number of concentrated node loads */
		int *nU, 		/**< number of uniformly distributed loads */
		int *nW,		/**< number of trapezoidaly distributed loads */
		int *nP, 		/**< number of concentrated point loads	*/
		int *nT, 		/**< number of temperature loads	*/
		int *nD,		/**< number of prescribed displacements */
		double **Q,		/**< frame element end forces, every beam */
		double **F_temp, 	/**< thermal loads			*/
		double **F_mech, 	/**< mechanical loads			*/
		double *Fo,	 	/**< thermal loads + mechanical loads	*/
		float ***U,		/**< uniformally distributed loads	*/
		float ***W,		/**< trapezoidally distributed loads	*/
		float ***P,		/**< concentrated point loads		*/
		float ***T,	 	/**< temperature loads			*/
		float **Dp,		/**< prescribed displacements at rctns	*/
		double ***eqF_mech,	/**< equiv. end forces for mech. loads	*/
		double ***eqF_temp,	/**< equiv. end forces for temp. loads	*/
		int verbose		/**< 1: copious output to screen, 0: none */
		);

	void ReadReactionData(
		FILE *fp,	/**< input data file pointer			*/
		int DoF,	/**< number of degrees of freedom		*/
		int nN,		/**< number of nodes				*/
		int *nR,	/**< number of nodes with reactions		*/
		int *q,		/**< q[i]=0: DoF i is fixed, q[i]=1: DoF i is free */
		int *r,		/**< r[i]=1: DoF i is fixed, r[i]=0: DoF i is free */
		int *sumR,	/**< sum of vector R				*/
		int verbose	/**< 1: copious screen output; 0: none		*/
		);

	void ReadMassData(
		FILE *fp,	/**< input data file pointer			*/
		char *OUT_file,	/**< input output data file name 		*/
		int nN, int nE,	/**< number of nodes, number of frame elements */
		int *nI,	/**< number of nodes with extra inertia	*/
		int *nX,	/**< number of elements with extra mass		*/
		float *d, float *EMs, /**< density, extra frame element mass	*/
		float *NMs, float *NMx, float *NMy, float *NMz, /**< node inertia*/
		double *L,	/**< length of each frame element		*/
		float *Ax, 	/**< cross section area of each frame element	*/
		double *total_mass,	/**< total mass of structure and extra mass */
		double *struct_mass, 	/**< mass of structural elements	*/
		int *nM,	/**< number of modes to find			*/
		int *Mmethod, 	/**< modal analysis method			*/
		int modal_flag, /**< command-line over-ride			*/
		int *lump,	/**< 1: use lumped mass matrix, 0: consistent mass */
		int lump_flag,	/**< command-line over-ride			*/
		double *tol,	/**< convergence tolerance for mode shapes	*/
		double tol_flag, /**< command-line over-ride			*/
		double *shift,	/**< frequency shift for unrestrained frames	*/
		double shift_flag, /**< command-line over-ride			*/
		double *exagg_modal, /**< exaggerate modal displacements	*/
		char modepath[], /**< filename for mode shape data for plotting	*/
		int *anim,	/**< list of modes to be graphically animated	*/
		float *pan,	/**< 1: pan viewpoint during animation, 0: don't */
		float pan_flag, /**< command-line over-ride			*/
		int verbose,	/**< 1: copious output to screen, 0: none	*/
		int debug	/**< 1: debugging output to screen, 0: none	*/
		);

	// GnuPlot file output
	void GnuPltStaticMesh(
		char OUT_file[],
		char infcpath[], char meshpath[], char plotpath[],
		char *title, int nN, int nE, int nL, int lc, int DoF,
		vec3 *xyz, double *L,
		int *N1, int *N2, float *p, double *D,
		double exagg_static, int D3_flag, int anlyz, float dx, float scale
		);

	void GnuPltCubicBentBeam(
		FILE *fpm,	/**< deformed mesh data file pointer	*/
		int n1, int n2,	/**< node 1 and node 2 of the frame element */
		vec3 *xyz,	/**< node coordinates			*/
		double L,	/**< frame element lengths		*/
		float p,	/**< frame element local rotations	*/
		double *D,	/**< node displacements		*/
		double exagg	/**< mesh exaggeration factor		*/
		);

private:
	void	dots(FILE *fp, int n)
	{
		int i;
		for (i = 1; i <= n; i++)	fprintf(fp, ".");
	}

	int		GetFileExt(char *filename, char *ext)
	{
		int	i = 0, full_len = 0, len = 0;

		while (filename[len++] != '\0') /* the length of file filename */;
		full_len = len;
		while (filename[len--] != '.' && len > 0) /* the last '.' in filename */;
		if (len == 0)	len = full_len;
		++len;

		for (i = 0; len < full_len; i++, len++) ext[i] = tolower(filename[len]);

		if (!strcmp(ext, ".csv")) return (1);
		if (!strcmp(ext, ".fmm")) return (2);
		return(0);
	}
private:
	CoordTrans		trsf_;
};
/*
* ==========================================================================
*
*       class: StiffnessSolver  
*
*    Description:  Matrix math function for stiffness linear system solving
*
*	 Version:  1.0
*	 Created:  Nov/24/2015
*
*	 Author:   Yijiang Huang, Xin Hu, Guoxian Song
*	 Company:  GCL@USTC
*	 Note:	   This file is modified from HPGmatrix.c, which is a part of Frame3dd.
*			You can get original C file of Frame3dd from http://frame3dd.sourceforge.net/.
*			Related matrix computation issue raised in matrix structural analysis can be 
*			found at
*			1. Partitioned matrix equation : http://people.duke.edu/~hpgavin/cee421/SolveMatrixEquation.pdf.
* ==========================================================================
*/
#ifndef STIFFNESS_SOLVER_H
#define STIFFNESS_SOLVER_H

#include <iostream>
#include <Eigen/dense>
#include <Eigen/sparse>

#define sind(x) (sin(fmod((x),360) * M_PI / 180))
#define cosd(x) (cos(fmod((x),360) * M_PI / 180))

using namespace std;

class StiffnessSolver
{
public:
	typedef Eigen::SparseMatrix<double> SpMat;
	typedef Eigen::MatrixXd				MX;
	typedef Eigen::Matrix3d				M3;
	typedef Eigen::VectorXd				VX;
	typedef Eigen::Vector3d				V3;
	typedef Eigen::VectorXi				VXi;
	typedef	Eigen::MatrixXi				MXi;
	
public:
	StiffnessSolver(){};
	~StiffnessSolver(){};

public:
	/* solver I/O*/
	
	/*
	* SolveSystem  -  solve {F} =   [K]{D} via L D L' decomposition        24/nov/2015
	* @param K   : stiffness matrix for the restrained frame
	* @param D   : displacement vector to be solved
	* @param F   : mechenical force(external load vector)
	* @param R   : unknown reaction force for restrained nodes
	* @param DoF : Degree of Freedom
	* @param q	 : state indicator, mark node with known external force but unknown displacement
	*			 i.e. q[i] = 1 for free node, q[i] = 0 for restrained node
	* @param r   : state indicator, mark node with known displacement but unknown reaction force
	*			 i.e, r[i] = 0 for free node, r[i] = 1 for restrained nodes
	* @param verbose	:  =1 for copious screenplay
	* @param info		: <0 : not stiffness matrix positive definite
	* @param rms_resid  : the RMS error of the solution residual 
	*/
	void SolveSystem(		
		SpMat &K, VX &D, VX &F, VX &R, 
		int Dof, VXi &q, VXi &r, 
		int verbose, int &info, double &rms_resid
		);

	void Debug();

public:
	/*
	* LDLDecompPM  -  Solves partitioned matrix equations		Nov/24/2015
	*
	* @param A : a symmetric diagonally-dominant matrix of dimension [1..n][1..n].
	* @param n : matrix dimension
	* @param d : diagonal of D in decomp, stored as a n-vector
	* @param b : a r.h.s. vector of dimension [1..n].
	* @param x : solution vector
	* @param c : solution vector on the rhs, returned as a vector of [1..n] with {c_q}=0.
	* @param q : a vector of the indexes of known values {b_q}(free nodes)
	* @param r : a vector of the indexes of known values {x_r}(restrained nodes)
	* @param reduce : 1: do a forward reduction of A; 0: don't
	* @param solve  ; 1: do a back substituition for solving {x}, 0: don't
	* @param info	: 1: definite matrix and successful L D L^t decomp
	*
	* usage: MX A, VX d, VX b, VX x;
	*		 int n, reduce, solve, info;
	*	ldl_dcmp_pm ( A, n, d, b, x, c, q, r, reduce, solve, pd );
	*/
	void LDLDecompPM(MX &A, const int &n, VX &d, VX &b, VX &x, VX &c, 
		const VXi &q, const VXi &r, const int &reduce, const int &solve, int &info);
	
	/*
	* LDLImprovePM - Improves a solution vector x[1..n] of the partitioned set of linear equations	Nov/24/2015
	*
	* @param A : The matrix [A_qq] is the L D L^t decomposition of 
	*			 the original system matrix, as returned by ldl_dcmp_pm().
	* @param n : The dimension of the matrix
	* @param d : diagonal of D in the L D L^t
	* @param b : the right hand side vector (unchanged)
	* @param x : solution vector					(modified to an improved set of values)
	* @param c : the part of the solution in the rhs(modified to an improved set of values)
	* @param q : q[j] = 1 if b[j] is known; q[j] = 0 otherwise
	* @param r : r[j] = 1 if x[j] is known; r[j] = 0 otherwise
	* @param rms_resid : root-mean-square of residual error
	* @param info	   : 1: >10% reduction in rms_resid; 0: don't
	* usage: double **A, *d, *b, *x, rms_resid;
	* 	int   n, ok, *q, *r;
	*	ldl_mprove_pm ( A, n, d, b, x, q, r, rms_resid, info );
	*/
	void LDLImprovePM(MX &A, int n, VX &d, const VX &b, VX &x, VX &c,
		const VXi &q, const VXi &r, double &rms_resid, int &info);

	/*
	* LUDecomp - Solves [A]{x} = {b}, simply and efficiently, by performing an		Nov/25/2015
	*			 LU-decomposition of matrix [A]. No pivoting is performed.
	* @param A is a diagonally dominant matrix of dimension [1..n][1..n].
	* @param b is a r.h.s. vector of dimension [1..n].
	*
	* {b} is updated using [LU] and then back-substitution is done to obtain {x}.
	* {b} is replaced by {x} and [A] is replaced by the LU-reduction of itself.
	*  usage:  MX A, VX b;
	*		   int   n, reduce, solve, info;
	*		   lu_dcmp ( A, n, b, reduce, solve, info);
	*/
	void LUDecomp(
		MX  &A,		/**< the system matrix, and its LU-reduction			 */
		int n,      /**< the dimension of the matrix						 */
		VX  &b,		/**< the right hand side vector, and the solution vector */
		int reduce, /**< 1: do a forward reduction; 0: don't				 */
		int solve,  /**< 1: do a back substitution for {x};  0: don't		 */
		int &info   /**< 1: positive diagonal  and  successful LU decomp'n   */
		);
};

#endif
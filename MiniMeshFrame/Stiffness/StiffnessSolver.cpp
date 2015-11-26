#include "StiffnessSolver.h"

void StiffnessSolver::SolveSystem(
		SpMat &K, VX &D, VX &F, VX &R, 
		int DoF, VXi &q, VXi &r, 
		int verbose, int &info, double &rms_resid)
{
	VX	diag;		/* diagonal vector of the L D L' decomp. */

	diag.resize(DoF);

	//MX K_comp = K;
	int row = K.rows(), col = K.cols();
	MX K_comp(row, col);
	K_comp.setZero();

	for (int k = 0; k < K.outerSize(); ++k)
	{
		for (SpMat::InnerIterator it(K, k); it; ++it)
		{
			int		r = it.row();
			int		c = it.col();
			double	v = it.value();
			K_comp(r, c) = v;
		}
	}

	/*  L D L' decomposition of K[q,q] into lower triangle of K[q,q] and diag[q] */
	/*  vectors F and D are unchanged */
	/*  not solving at this moment*/
	LDLDecompPM(K_comp, DoF, diag, F, D, R, q, r, 1, 0, info);
	if (info < 0)
	{
		fprintf(stderr, " Stiffness Matrix is not positive definite");
		fprintf(stderr, " The stucture may have mechanism and thus not stable in general");
		fprintf(stderr, " Please Make sure that all six");
		fprintf(stderr, " rigid body translations are restrained!\n");
		/* exit(31); */
	}
	else
	{
		/* LDL'  back-substitution for D[q] and R[r] */
		LDLDecompPM(K_comp, DoF, diag, F, D, R, q, r, 0, 1, info);
		if (verbose) { fprintf(stdout, "    LDL' RMS residual:"); }
		rms_resid = info = 1;

		do {
			/* improve solution for D[q] and R[r] */
			LDLImprovePM(K_comp, DoF, diag, F, D, R, q, r, rms_resid, info);
			if (verbose) { fprintf(stdout, "%9.2e", rms_resid); }
		} while (info);

		if (verbose) fprintf(stdout, "LDL^t Solving completed\n");
	}

}

/*
* LDLDecompPM - Solves partitioned matrix equations		Nov / 24 / 2015
* Function:
*			Suppose the nodes[1, ..., n] are un - restrained and
*			the displacement of coordinates[n + 1, ..., 2N] are restrained nodes
*			the vector forces b(external forces), reaction forces c(only for restrained nodes)
*           and the vector displacement d(at all coord) are related by the following :
*					[A_qq, A_qr][x_q] = [b_q]
*					[A_rq, A_rr][x_r] = [b_r + c_r]
*			where q represents free node with known force but unknown displacements
*				  r represents restrained node with unknown reaction force but known displacments
*
*					[A_qq]{x_q} +[A_qr]{x_r} = { b_q }
*					[A_rq]{x_q} +[A_rr]{x_r} = { b_r }+{c_r}
*			where {b_q}, { b_r }, and{ x_r } are known and
*           where {x_q} and{ c_r } are unknown
*
*			we use LDLt decomposition(without pivoting) to solve and get x_q = inv(A_qq)(b_q - [A_qr][x_r])
*			and then simply plug in x_q to get c_r = [A_rq][x_q] + [A_rr][x_r] - [b_r]
*
* Detail : LDLt is implemented without pivoting and
*		   the lower triangle of[A_qq] is replaced by the lower triangle L of its
*		   L D L' reduction.  The diagonal of D is returned in the vector {d}
*		   {b} is updated using L D L' and then back-substitution is done to obtain {x}
*		   {b_q} and{ b_r }  are returned unchanged.
*/
void StiffnessSolver::LDLDecompPM(
	MX &A, const int &n, VX &d, VX &b, VX &x, VX &c,
	const VXi &q, const VXi &r,
	const int &reduce, const int &solve, int &info)
{
	int	i, j, k, m;
	info = 0;	/* number of negative elements on the diagonal of D */

	if (reduce)
	{		
		/* forward column-wise reduction of [A]	*/
		for (j = 0; j < n; j++)
		{
			d[j] = 0.0;

			if (q[j])
			{
				/* reduce column j, except where q[i]==1() */
				for (m = 0, i = 0; i < j; i++)
				{
					// find one entry in the col that is not zero
					if (A(i,j) == 0.0)
						++m;
					else
						break;
				}

				for (i = m; i < j; i++)
				{
					if (q[i])
					{
						A(j,i) = A(i,j);
						for (k = m; k < i; k++)
						{
							if (q[k])
							{
								A(j,i) -= A(j,k) * A(i,k);
							}
						}
					}
				}

				d[j] = A(j,j);
				for (i = m; i < j; i++)
				{
					if (q[i])
					{
						d[j] -= A(j,i)* A(j,i) / d[i];
					}
				}
				for (i = m; i < j; i++)
				{
					if (q[i])
					{
						A(j, i) /= d[i];
					}
				}

				if (d[j] == 0.0)
				{
					fprintf(stderr, " ldl_dcmp_pm(): zero found on diagonal ...\n");
					fprintf(stderr, " d[%d] = %11.4e\n", j, d[j]);
					return;
				}

				if (d[j] < 0.0) { info--; }
			}
		}

	}		/* the forward reduction of [A] is now complete	*/

	if (solve)
	{
		/* back substitution to solve for {x}   */
		for (i = 0; i < n; i++)
		{
			if (q[i])
			{
				x[i] = b[i];
				for (j = 0; j < n; j++)
				{
					if (r[j])
					{
						x[i] -= A(i,j) * x[j];
					}
				}
			}
		}

		/* {x} is run through the same forward reduction as was [A] */
		for (i = 0; i < n; i++)
		{
			if (q[i])
			{
				for (j = 0; j < i; j++)
				{
					if (q[j])
					{
						x[i] -= A(i, j) * x[j];
					}
				}
			}
		}

		for (i = 0; i < n; i++)
		{
			if (q[i])
			{
				x[i] /= d[i];
			}
		}

		/* now back substitution is conducted on {x};  [A] is preserved */
		for (i = n - 1; i > 0; i--)
		{
			if (q[i])
			{
				for (j = 0; j < i; j++)
				{
					if (q[j])
					{
						x[j] -= A(i,j) * x[i];
					}
				}
			}
		}

		/* finally, evaluate c_r	*/
		for (i = 0; i < n; i++)
		{
			c[i] = 0.0;
			if (r[i])
			{
				c[i] = -b[i];
				for (j = 0; j < n; j++)
				{
					c[i] += A(i,j) * x[j];
				}
			}
		}
	}// end if(solve)
	return;
}

/*
* LDLImprovePM
* Improves a solution vector x[1..n] of the partitioned set of linear equations
*           [A_qq]{x_q} + [A_qr]{x_r} = {b_q}
*           [A_rq]{x_q} + [A_rr]{x_r} = {b_r}+{c_r}
*           where {b_q}, {b_r}, and {x_r} are known and
*           where {x_q} and {c_r} are unknown
* by reducing the residual r_q
*           A_qq r_q = {b_q} - [A_qq]{x_q+r_q} + [A_qr]{x_r}
* The matrix A[1..n][1..n], and the vectors b[1..n] and x[1..n]
* are input, as is the dimension n.   The matrix [A] is the L D L'
* decomposition of the original system matrix, as returned by ldl_dcmp_pm().
* Also input is the diagonal vector, {d} of [D] of the L D L' decompositon.
* On output, only {x} is modified to an improved set of values.
* The calculations in ldl_mprove_pm do not involve b_r.
*/
void StiffnessSolver::LDLImprovePM(
	MX &A, int n, VX &d, const VX &b, VX &x, VX &c, const VXi &q,
	const VXi &r, double &rms_resid, int &info)
{
	double  sdp;		// accumulate the r.h.s. in double precision
	VX		dx,		// the residual error
		dc;		// update to partial r.h.s. vector, c
	double 	rms_resid_new = 0.0; // the RMS error of the mprvd solution
	
	int	j, i, pd;

	dx.resize(n);
	dc.resize(n);

	for (i = 0; i < n; i++)
	{
		dx[i] = 0.0;
	}

	// original equation : [A_qq]{x_q} +[A_qr]{x_r} = { b_q }
	// calculate the r.h.s. of ...
	// [A_qq]{dx_q} = {b_q} - [A_qq]*{x_q} - [A_qr]*{x_r}      
	// {dx_r} is left unchanged at 0.0
	for (i = 0; i < n; i++)
	{
		if (q[i]) 
		{
			sdp = b[i];
			for (j = 0; j < n; j++) 
			{
				if (q[j]) 
				{	
					// A_qq in upper triangle only
					if (i <= j)
					{
						sdp -= A(i,j) * x[j];
					}
					else
					{
						sdp -= A(j,i) * x[j];
					}
				}
			}
			for (j = 0; j < n; j++)
			{
				if (r[j])
				{
					sdp -= A(i,j) * x[j];
				}
			}
				dx[i] = sdp;
		} // else dx[i] = 0.0; // x[i];
	}

	// solve for the residual error term, A is already factored
	// ldl_dcmp_pm(A, n, d, dx, dx, dc, q, r, 0, 1, &pd);
	LDLDecompPM(A, n, d, dx, dx, dc, q, r, 0, 1, info);

	for (i = 0; i < n; i++)
	{
		if (q[i])
		{
			rms_resid_new += dx[i] * dx[i];
		}
	}

	rms_resid_new = sqrt(rms_resid_new / (double)n);

	info = 0;
	if (rms_resid_new / rms_resid < 0.90) 
	{
		/*  enough improvement    */
		for (i = 0; i < n; i++) 
		{	
			if (q[i])
			{
				x[i] += dx[i];
			}
			if (r[i])
			{
				c[i] += dc[i];
			}
		}
		rms_resid = rms_resid_new;	/* return the new residual   */
		info = 1;					/* the solution has improved */
	}

	return;
}

/*
* LUDecomp - Solves [A]{x} = {b}, simply and efficiently, by performing an		Nov/25/2015
*			 LU-decomposition of matrix [A]. No pivoting is performed.
*
* {b} is updated using [LU] and then back-substitution is done to obtain {x}.
* {b} is replaced by {x} and [A] is replaced by the LU-reduction of itself.
*/
void StiffnessSolver::LUDecomp(
	MX  &A,		/**< the system matrix, and its LU-reduction			 */
	int n,      /**< the dimension of the matrix						 */
	VX  &b,		/**< the right hand side vector, and the solution vector */
	int reduce, /**< 1: do a forward reduction; 0: don't				 */
	int solve,  /**< 1: do a back substitution for {x};  0: don't		 */
	int &info   /**< 1: positive diagonal  and  successful LU decomp'n   */
	)
{
	double	pivot;		/* a diagonal element of [A]		*/
	int	i, j, k;

	info = 1;
	if (reduce) 
	{			
		/* forward reduction of [A]	*/

		for (k = 0; k < n; k++) 
		{
			if (0.0 == (pivot = A(k,k))) 
			{
				fprintf(stderr, " lu_dcmp: zero found on the diagonal\n");
				fprintf(stderr, " A[%d][%d] = %11.4e\n", k, k, A(k,k));
				info = 0;
				return;
			}
			for (i = k + 1; i < n; i++) 
			{
				A(i,k) /= pivot;
				for (j = k + 1; j < n; j++)
				{
					A(i,j) -= A(i,k) * A(k,j);
				}
			}
		}
	}		/* the forward reduction of [A] is now complete	*/

	if (solve) 
	{		
		/* back substitution to solve for {x}	*/

		/* {b} is run through the same forward reduction as was [A]	*/

		for (k = 0; k < n; k++)
		{
			for (i = k + 1; i < n; i++)	
			{
				b[i] -= A(i,k) * b(k);
			}
		}
		/* now back substitution is conducted on {b};  [A] is preserved */

		for (j = n - 1; j >= 1; j--)
		{
			for (i = 0; i <= j - 1; i++)	
			{
				b[i] -= b[j] * A(i,j) / A(j,j);
			}
		}

		/* finally we solve for the {x} vector			*/

		for (i = 0; i < n; i++)
		{
			b[i] /= A(i,i);
		}
	}

	/* {b} is now {x} and is ready to be returned	*/

	return;
}

void StiffnessSolver::Debug()
{
}
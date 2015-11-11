#include "LPMosek.h"
#include "Loader.h"

#include <iomanip>      // std::setprecision

#define MOSEK_EXISTS

#ifdef MOSEK_EXISTS
#include <mosek.h>
#endif

#define MYOUT std::cout
#define MYERR std::cerr

LPMosek::LPMosek() :LinearP(), fVal_(std::numeric_limits<double>::infinity())
{
#ifdef MOSEK_EXISTS
	MSKrescodee   r;
	r = MSK_makeenv(&env_, NULL);
	assert(r == MSK_RES_OK);
#endif
}

LPMosek::~LPMosek()
{
#ifdef MOSEK_EXISTS
	MSK_deleteenv(&env_);
#endif
}

#ifdef MOSEK_EXISTS
/* This function prints log output from MOSEK to the terminal. */
void MSKAPI LP_printstr(void *handle,
	MSKCONST char str[])
{
	(void)handle;
	printf("%s", str);
} /* printstr */
#endif

bool LPMosek::solve(const VX& c,
	const MX& A, const VX& lc, const VX& uc,
	const VX& lb, const VX& ub,
	VX& _x, bool _debug, bool is_integer_opt)
{
#ifdef MOSEK_EXISTS
	fVal_ = MYINF;

	tSetup.start();

	bool success = false;

	//number of variables
	MSKint32t numvar = c.size();

	//number of constraints
	MSKint32t numcon = A.rows();

	MSKint32t     i, j;

	MSKenv_t      env = env_;
	MSKtask_t     task = NULL;
	MSKrescodee   r = MSK_RES_OK;

	if (r == MSK_RES_OK)
	{
		/* Create the optimization task. */
		r = MSK_maketask(env, numcon, numvar, &task);

		if (r == MSK_RES_OK)
		{
			if (_debug)
			{
				r = MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, LP_printstr);
			}

			// Append 'numcon' empty constraints. 
			// The constraints will initially have no bounds.
			if (r == MSK_RES_OK)
				r = MSK_appendcons(task, numcon);

			// Append 'NUMVAR' variables.
			// The variables will initially be fixed at zero (x=0).
			if (r == MSK_RES_OK)
				r = MSK_appendvars(task, numvar);

			// Set constraint matrix A to mosek
			if (r == MSK_RES_OK)
			{
				if (_debug)
				{
					MYOUT << "LP_constraint matrix A: " << std::endl;
				}

				//for (int k = 0; k < A.outerSize() && r == MSK_RES_OK; ++k)
				//{
				//	for (SpMat::InnerIterator it(A, k); it; ++it)
				//	{
				//		MSKint32t r = it.row();
				//		MSKint32t c = it.col();
				//		double	v = it.value();

				//		if (_debug)
				//		{
				//			MYOUT << "(" << r << "," << c << ") " << /*std::setprecision(16) <<*/ v << std::endl;
				//		}

				//		r = MSK_putaij(task, r, c, v);
				//	}
				//}

				for (int i = 0; i < A.rows(); i++)
				{
					for (int j = 0; j < A.cols(); j++)
					{
						MSKint32t r = i;
						MSKint32t c = j;
						double	v = A(i,j);

						//if (_debug)
						//{
						//	MYOUT << "(" << r << "," << c << ") " << /*std::setprecision(16) <<*/ v << std::endl;
						//}

						r = MSK_putaij(task, r, c, v);
					}
				}
			}

			for (j = 0; j < numvar && r == MSK_RES_OK; ++j)
			{

				// Set the linear term c_j in the objective.
				if (c.size()>0)
				{
					if (r == MSK_RES_OK)
					{
						r = MSK_putcj(task, j, c[j]);
					}
				}

				// Set the bounds on variable j.
				// blx[j] <= x_j <= bux[j]

				if (r == MSK_RES_OK)
				{
					MSKboundkeye bkx = MSK_BK_FR;
					double lj = lb[j];
					double uj = ub[j];

					if (lj != -MYINF && uj == MYINF){ bkx = MSK_BK_LO; }
					if (lj == -MYINF && uj != MYINF){ bkx = MSK_BK_UP; }
					if (lj == uj){ bkx = MSK_BK_FX; assert(std::isfinite(lj)); }
					if (lj == -MYINF && uj == MYINF){ bkx = MSK_BK_FR; }
					if (lj != -MYINF && uj != MYINF){ bkx = MSK_BK_RA; }

					r = MSK_putvarbound(task,
						j,        // Index of variable.
						bkx,      // Bound key.
						lj,       // Numerical value of lower bound.
						uj);      // Numerical value of upper bound.
				}
			}

			// Set the bounds on constraints.
			// for i=1, ...,NUMCON : blc[i] <= constraint i <= buc[i]
			for (i = 0; i < numcon && r == MSK_RES_OK; ++i)
			{
				double lbi = lc[i];
				double ubi = uc[i];
				MSKboundkeye bkc = MSK_BK_FR;

				if (lbi != -MYINF && ubi == MYINF){ bkc = MSK_BK_LO; }
				if (lbi == -MYINF && ubi != MYINF){ bkc = MSK_BK_UP; }
				if (lbi == ubi){ bkc = MSK_BK_FX; assert(std::isfinite(lbi)); }
				if (lbi == -MYINF && ubi == MYINF){ bkc = MSK_BK_FR; }
				if (lbi != -MYINF && ubi != MYINF){ bkc = MSK_BK_RA; }

				r = MSK_putconbound(task,
					i,							/* Index of constraint.*/
					bkc,	/* Bound key.*/
					lbi,			/* Numerical value of lower bound.*/
					ubi);			/* Numerical value of upper bound.*/
			}
		}

		if (_debug){
			MSK_analyzeproblem(task, MSK_STREAM_MSG);
			MSK_writedata(task, "Mosek_taskdump.opf");
		}

		tSetup.stop();
		if (storeVariables_){
			std::string fileName;
			if (!Loader::uniqueFilename(storePath_ + "/QPMosekDump", ".task", fileName))
			{
				MYERR << __FUNCTION__ << ": No unique filename for QpMosek dump found. Not storing." << std::endl;
			}
			else
			{
				MSK_writedata(task, fileName.c_str());
			}
		}

		// integer optimization or not
		if (is_integer_opt == true)
		{
			for (j = 0; j < numvar && r == MSK_RES_OK; ++j)
			{
				r = MSK_putvartype(task, j, MSK_VAR_TYPE_INT);
			}
		}


		// Minimize objective function.
		if (r == MSK_RES_OK)
		{
			r = MSK_putobjsense(task, MSK_OBJECTIVE_SENSE_MINIMIZE);
		}

		tSolve.start();

		if (r == MSK_RES_OK)
		{
			MSKrescodee trmcode;

			// Run optimizer
			r = MSK_optimizetrm(task, &trmcode);

			// Print a summary containing information
			// about the solution for debugging purposes
			if (_debug)
			{
				MSK_solutionsummary(task, MSK_STREAM_MSG);
			}

			if (r == MSK_RES_OK)
			{
				// solution status
				MSKsolstae solsta;

				// We keep the basic solution (MSK_SOL_BAS)
				if (!is_integer_opt)
				{
					MSK_getsolsta(task, MSK_SOL_BAS, &solsta);
				}
				else
				{
					MSK_getsolsta(task, MSK_SOL_ITG, &solsta);
				}

				xFlag_ = solsta;

				switch (solsta)
				{
				case MSK_SOL_STA_OPTIMAL:
				case MSK_SOL_STA_NEAR_OPTIMAL:
					_x.resize(numvar);
					MSK_getxx(task,
						MSK_SOL_BAS,    // Request the basic solution.
						_x.data());

					MSK_getprimalobj(task, MSK_SOL_ITR, &fVal_);
					assert(std::isfinite(fVal_));

					success = true;
					printf("Mosek Lp Optimal primal solution\n");
					for (j = 0; j < numvar; ++j)
						printf("x[%d]: %e\n", j, _x[j]);

					break;
				case MSK_SOL_STA_DUAL_INFEAS_CER:
				case MSK_SOL_STA_PRIM_INFEAS_CER:
				case MSK_SOL_STA_NEAR_DUAL_INFEAS_CER:
				case MSK_SOL_STA_NEAR_PRIM_INFEAS_CER:
					printf("Primal or dual infeasibility certificate found.\n");
					break;

				case MSK_SOL_STA_UNKNOWN:
				{
											char symname[MSK_MAX_STR_LEN];
											char desc[MSK_MAX_STR_LEN];

											// If the solutions status is unknown, print the termination code
											// indicating why the optimizer terminated prematurely.

											MSK_getcodedesc(trmcode,
												symname,
												desc);

											printf("The status of the solution could not be determined.\n");
											printf("The optimizer terminitated with code: %s\n", symname);
											break;
				}
				case MSK_SOL_STA_NEAR_INTEGER_OPTIMAL:
				case MSK_SOL_STA_INTEGER_OPTIMAL:
					_x.resize(numvar);
					MSK_getxx(task,
						MSK_SOL_ITG,    // Request the basic solution.
						_x.data());

					MSK_getprimalobj(task, MSK_SOL_ITG, &fVal_);
					assert(std::isfinite(fVal_));

					success = true;
					printf("Mosek Lp Optimal integer solution\n");
					/*for (j = 0; j < numvar; ++j)
						printf("x[%d]: %e\n", j, _x[j]);*/

					break;
				default:
					printf("Other solution status.");
					break;
				}
			} // solution status
			else
			{
				printf("Error while optimizing.\n");
			}
		} // MSK_optimizetrm

		if (r != MSK_RES_OK)
		{
			/* In case of an error print error code and description. */
			char symname[MSK_MAX_STR_LEN];
			char desc[MSK_MAX_STR_LEN];

			printf("An error occurred while optimizing.\n");
			MSK_getcodedesc(r,
				symname,
				desc);
			printf("Error %s - '%s'\n", symname, desc);
		}

		MSK_deletetask(&task);
	}

	tSolve.stop();
	
	return success;

#else
	MYOUT << __FUNCTION__ << ": Not compiled with Mosek. Cant optimize." << std::endl;
	return false;
#endif		  
}

std::string LPMosek::report() const
{
	std::stringstream ss;
	ss << "LP using Mosek. exitFlag: " << xFlag_ << " ( " << exitFlagToString(xFlag_) << " ) " << std::endl;
	ss << "Timing: " << tSetup << " setup, " << tSolve << " solve." << std::endl;
	return ss.str();
}

bool LPMosek::test() const
{
	const MSKint32t numvar = 4,
		numcon = 3;

	double       c[] = { 3.0, 1.0, 5.0, 1.0 };
	/* Below is the sparse representation of the A
	matrix stored by column. */
	MSKint32t    aptrb[] = { 0, 2, 5, 7 },
		aptre[] = { 2, 5, 7, 9 },
		asub[] = { 0, 1,
		0, 1, 2,
		0, 1,
		1, 2 };
	double       aval[] = { 3.0, 2.0,
		1.0, 1.0, 2.0,
		2.0, 3.0,
		1.0, 3.0 };

	/* Bounds on constraints. */
	MSKboundkeye bkc[] = { MSK_BK_FX, MSK_BK_LO, MSK_BK_UP };
	double       blc[] = { 30.0, 15.0, -MSK_INFINITY };
	double       buc[] = { 30.0, +MSK_INFINITY, 25.0 };
	/* Bounds on variables. */
	MSKboundkeye bkx[] = { MSK_BK_LO, MSK_BK_RA, MSK_BK_LO, MSK_BK_LO };
	double       blx[] = { 0.0, 0.0, 0.0, 0.0 };
	double       bux[] = { +MSK_INFINITY, 10.0, +MSK_INFINITY, +MSK_INFINITY };
	MSKenv_t     env = NULL;
	MSKtask_t    task = NULL;
	MSKrescodee  r;
	MSKint32t    i, j;

	/* Create the mosek environment. */
	r = MSK_makeenv(&env, NULL);

	if (r == MSK_RES_OK)
	{
		/* Create the optimization task. */
		r = MSK_maketask(env, numcon, numvar, &task);

		/* Directs the log task stream to the 'printstr' function. */
		if (r == MSK_RES_OK)
			r = MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, LP_printstr);

		/* Append 'numcon' empty constraints.
		The constraints will initially have no bounds. */
		if (r == MSK_RES_OK)
			r = MSK_appendcons(task, numcon);

		/* Append 'numvar' variables.
		The variables will initially be fixed at zero (x=0). */
		if (r == MSK_RES_OK)
			r = MSK_appendvars(task, numvar);

		for (j = 0; j<numvar && r == MSK_RES_OK; ++j)
		{
			/* Set the linear term c_j in the objective.*/
			if (r == MSK_RES_OK)
				r = MSK_putcj(task, j, c[j]);

			/* Set the bounds on variable j.
			blx[j] <= x_j <= bux[j] */
			if (r == MSK_RES_OK)
				r = MSK_putvarbound(task,
				j,           /* Index of variable.*/
				bkx[j],      /* Bound key.*/
				blx[j],      /* Numerical value of lower bound.*/
				bux[j]);     /* Numerical value of upper bound.*/

			/* Input column j of A */
			if (r == MSK_RES_OK)
				r = MSK_putacol(task,
				j,                 /* Variable (column) index.*/
				aptre[j] - aptrb[j], /* Number of non-zeros in column j.*/
				asub + aptrb[j],     /* Pointer to row indexes of column j.*/
				aval + aptrb[j]);    /* Pointer to Values of column j.*/
		}

		/* Set the bounds on constraints.
		for i=1, ...,numcon : blc[i] <= constraint i <= buc[i] */
		for (i = 0; i<numcon && r == MSK_RES_OK; ++i)
			r = MSK_putconbound(task,
			i,           /* Index of constraint.*/
			bkc[i],      /* Bound key.*/
			blc[i],      /* Numerical value of lower bound.*/
			buc[i]);     /* Numerical value of upper bound.*/

		/* Maximize objective function. */
		if (r == MSK_RES_OK)
			r = MSK_putobjsense(task, MSK_OBJECTIVE_SENSE_MAXIMIZE);

		if (r == MSK_RES_OK)
		{
			MSKrescodee trmcode;

			/* Run optimizer */
			r = MSK_optimizetrm(task, &trmcode);

			/* Print a summary containing information
			about the solution for debugging purposes. */
			MSK_solutionsummary(task, MSK_STREAM_LOG);

			if (r == MSK_RES_OK)
			{
				MSKsolstae solsta;

				if (r == MSK_RES_OK)
					r = MSK_getsolsta(task,
					MSK_SOL_BAS,
					&solsta);
				switch (solsta)
				{
				case MSK_SOL_STA_OPTIMAL:
				case MSK_SOL_STA_NEAR_OPTIMAL:
				{
												 double *xx = (double*)calloc(numvar, sizeof(double));
												 if (xx)
												 {
													 MSK_getxx(task,
														 MSK_SOL_BAS,    /* Request the basic solution. */
														 xx);

													 printf("Optimal primal solution\n");
													 for (j = 0; j<numvar; ++j)
														 printf("x[%d]: %e\n", j, xx[j]);

													 free(xx);
												 }
												 else
													 r = MSK_RES_ERR_SPACE;

												 break;
				}
				case MSK_SOL_STA_DUAL_INFEAS_CER:
				case MSK_SOL_STA_PRIM_INFEAS_CER:
				case MSK_SOL_STA_NEAR_DUAL_INFEAS_CER:
				case MSK_SOL_STA_NEAR_PRIM_INFEAS_CER:
					printf("Primal or dual infeasibility certificate found.\n");
					break;
				case MSK_SOL_STA_UNKNOWN:
				{
											char symname[MSK_MAX_STR_LEN];
											char desc[MSK_MAX_STR_LEN];

											/* If the solutions status is unknown, print the termination code
											indicating why the optimizer terminated prematurely. */

											MSK_getcodedesc(trmcode,
												symname,
												desc);

											printf("The solution status is unknown.\n");
											printf("The optimizer terminitated with code: %s\n", symname);
											break;
				}
				default:
					printf("Other solution status.\n");
					break;
				}
			}
		}

		if (r != MSK_RES_OK)
		{
			/* In case of an error print error code and description. */
			char symname[MSK_MAX_STR_LEN];
			char desc[MSK_MAX_STR_LEN];

			printf("An error occurred while optimizing.\n");
			MSK_getcodedesc(r,
				symname,
				desc);
			printf("Error %s - '%s'\n", symname, desc);
		}

		/* Delete the task and the associated data. */
		MSK_deletetask(&task);
	}

	/* Delete the environment and the associated data. */
	MSK_deleteenv(&env);

	return (r == MSK_RES_OK);

}

std::string LPMosek::exitFlagToString(int _xflag) const
{
#ifdef MOSEK_EXISTS
	switch (_xflag){
	case MSK_SOL_STA_UNKNOWN:	return "MSK_SOL_STA_UNKNOWN";
	case MSK_SOL_STA_OPTIMAL:	return "MSK_SOL_STA_OPTIMAL";
	case MSK_SOL_STA_PRIM_FEAS:	return "MSK_SOL_STA_PRIM_FEAS";
	case MSK_SOL_STA_DUAL_FEAS:	return "MSK_SOL_STA_DUAL_FEAS";
	case MSK_SOL_STA_PRIM_AND_DUAL_FEAS:	return "MSK_SOL_STA_PRIM_AND_DUAL_FEAS";
	case MSK_SOL_STA_PRIM_INFEAS_CER:	return "MSK_SOL_STA_PRIM_INFEAS_CER";
	case MSK_SOL_STA_DUAL_INFEAS_CER:	return "MSK_SOL_STA_DUAL_INFEAS_CER";
	case MSK_SOL_STA_NEAR_OPTIMAL:	return "MSK_SOL_STA_NEAR_OPTIMAL";
	case MSK_SOL_STA_NEAR_PRIM_FEAS:	return "MSK_SOL_STA_NEAR_PRIM_FEAS";
	case MSK_SOL_STA_NEAR_DUAL_FEAS:	return "MSK_SOL_STA_NEAR_DUAL_FEAS";
	case MSK_SOL_STA_NEAR_PRIM_AND_DUAL_FEAS:	return "MSK_SOL_STA_NEAR_PRIM_AND_DUAL_FEAS";
	case MSK_SOL_STA_NEAR_PRIM_INFEAS_CER:	return "MSK_SOL_STA_NEAR_PRIM_INFEAS_CER";
	case MSK_SOL_STA_NEAR_DUAL_INFEAS_CER:	return "MSK_SOL_STA_NEAR_DUAL_INFEAS_CER";
	case MSK_SOL_STA_INTEGER_OPTIMAL:	return "MSK_SOL_STA_INTEGER_OPTIMAL";
	case MSK_SOL_STA_NEAR_INTEGER_OPTIMAL:	return "MSK_SOL_STA_NEAR_INTEGER_OPTIMAL";
	}

	return "MSK_SOL_STA_UNKNOWN";
#else
	return std::string();
#endif
}
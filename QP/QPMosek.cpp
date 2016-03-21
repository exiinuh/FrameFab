#include "QPMosek.h"
#include "loader.h"

#include <iomanip>      // std::setprecision

#define MOSEK_EXISTS

#ifdef MOSEK_EXISTS
#include <mosek.h>
#endif


#define MYOUT std::cout
#define MYERR std::cerr

QPMosek::QPMosek() :QP(), fVal_(std::numeric_limits<double>::infinity())
{
#ifdef MOSEK_EXISTS
	MSKrescodee   r;
	r = MSK_makeenv(&env_, NULL);
	assert(r == MSK_RES_OK);
#endif
	nTasks_ = -1;
	mP_ = 10e-8;
}

QPMosek::~QPMosek()
{
#ifdef MOSEK_EXISTS
	MSK_deleteenv(&env_);
#endif
}

#ifdef MOSEK_EXISTS
/* This function prints log output from MOSEK to the terminal. */
void MSKAPI printstr(void *handle,
	MSKCONST char str[])
{
	(void)handle;
	printf("%s", str);
} /* printstr */
#endif

bool QPMosek::solve(const S& H, const V& f,
	const S& A, const V& b,
	const S& C, const V& d,
	const V& lb, const V& ub,
	V &_x, const V *_x0/*=NULL*/,
	const Cones* cones/*=NULL*/,
	bool _debug)
{
#ifdef MOSEK_EXISTS
	fVal_ = MYINF;
	tSetup.start();

	bool success = false;

	//To cancel warnings about unused
	(void)_x0;

	//number of variables
	MSKint32t numvar = H.rows();
	bool linprog = (numvar == 0);
	if (linprog){ numvar = f.size(); }

	//number of constraints
	MSKint32t numcon = b.size() + d.size();

	MSKint32t     i, j;

	MSKenv_t      env = env_;
	MSKtask_t     task = NULL;
	MSKrescodee   r = MSK_RES_OK;

	if (r == MSK_RES_OK)
	{
		/* Create the optimization task. */
		r = MSK_maketask(env, numcon, numvar, &task);

		if (nTasks_>0)
		{
			MSK_putintparam(task, MSK_IPAR_NUM_THREADS, nTasks_);
		}


		//set precision: see http://docs.mosek.com/7.0/capi/The_optimizers_for_continuous_problems.html#sec-solve-conic
		MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_PFEAS, mP_);		//Controls primal feasibility, default 10e-8
		MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_DFEAS, mP_);		//Controls dual feasibility, default 10e-8
		MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_REL_GAP, mP_);	//Controls relative gap, default 10e-7
		MSK_putdouparam(task, MSK_DPAR_INTPNT_TOL_INFEAS, mP_);		//Controls when the problem is declared infeasible, default 10e-10
		MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_MU_RED, mP_);	//Controls when the complementarity is reduced enough, default 10e-8

		if (r == MSK_RES_OK)
		{
			if (_debug){ r = MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, printstr); }

			//Set mosek to use simplex optimizer. Doc says this method should be more suited for hot-start
			// if ( r == MSK_RES_OK )
			//	  r = MSK_putintparam(task,MSK_IPAR_OPTIMIZER,MSK_OPTIMIZER_FREE_SIMPLEX);

			/* Append 'NUMCON' empty constraints.
			The constraints will initially have no bounds. */
			if (r == MSK_RES_OK)
				r = MSK_appendcons(task, numcon);

			/* Append 'NUMVAR' variables.
			The variables will initially be fixed at zero (x=0). */
			if (r == MSK_RES_OK)
				r = MSK_appendvars(task, numvar);

			/* Optionally add a constant term to the objective. */
			if (r == MSK_RES_OK)
				r = MSK_putcfix(task, 0.0);

			/* Copy matrix [A;C] to mosek */
			if (r == MSK_RES_OK)
			{
				if (_debug){ MYOUT << "A: " << std::endl; }

				for (int k = 0; k<A.outerSize() && r == MSK_RES_OK; ++k)
				{
					for (S::InnerIterator it(A, k); it; ++it)
					{
						MSKint32t r = it.row();
						MSKint32t c = it.col();
						double	v = it.value();
						//if (_debug){ MYOUT << "(" << r << "," << c << ") " << /*std::setprecision(16) <<*/ v << std::endl; }
						r = MSK_putaij(task, r, c, v);
					}
				}

				if (_debug) { MYOUT << "C: " << std::endl; }

				for (int k = 0; k<C.outerSize() && r == MSK_RES_OK; ++k)
				{
					for (S::InnerIterator it(C, k); it; ++it)
					{
						MSKint32t r = it.row() + A.rows();
						MSKint32t c = it.col();
						double	v = it.value();
						//if (_debug){ MYOUT << "(" << r << "," << c << ") " << /*std::setprecision(16) <<*/ v << std::endl; }
						r = MSK_putaij(task, r, c, v);
					}
				}

				if (r == MSK_RES_OK)
				{
					if (cones != NULL)
					{
						for (unsigned int ci = 0; ci<cones->size(); ++ci)
						{
							r = MSK_appendcone(task, MSK_CT_QUAD, 0.0, cones->at(ci).idxs.size(), &(cones->at(ci).idxs[0]));
							//r=MSK_appendcone(task,MSK_CT_QUAD,0.0,cones->at(ci).size(),&(cones->at(ci)[0]));
							std::cout << ci << std::endl;
						}
					}
				}


				/*
				r = MSK_putacolslice(task,
				0,
				A.cols(),
				A.outerIndexPtr(),
				A.outerIndexPtr()+1,
				A.innerIndexPtr(),
				A.valuePtr());

				r = MSK_putacolslice(task,
				A.cols(),
				A.cols()+C.cols(),
				C.outerIndexPtr(),
				C.outerIndexPtr()+1,
				C.innerIndexPtr(),
				C.valuePtr());
				*/
			}


			for (j = 0; j<numvar && r == MSK_RES_OK; ++j)
			{

				/* Set the linear term c_j in the objective.*/
				if (f.size()>0){
					if (r == MSK_RES_OK)
						r = MSK_putcj(task, j, f[j]);
				}
				/* Set the bounds on variable j.
				blx[j] <= x_j <= bux[j] */
				if (r == MSK_RES_OK){
					MSKboundkeye bkx = MSK_BK_FR;
					double lj = lb[j];
					double uj = ub[j];

					if (lj != -MYINF && uj == MYINF){ bkx = MSK_BK_LO; }
					if (lj == -MYINF && uj != MYINF){ bkx = MSK_BK_UP; }
					if (lj == uj){ bkx = MSK_BK_FX; assert(std::isfinite(lj)); }
					if (lj == -MYINF && uj == MYINF){ bkx = MSK_BK_FR; }
					if (lj != -MYINF && uj != MYINF){ bkx = MSK_BK_RA; }

					r = MSK_putvarbound(task,
						j,           /* Index of variable.*/
						bkx,      /* Bound key.*/
						lj,      /* Numerical value of lower bound.*/
						uj);     /* Numerical value of upper bound.*/
				}

				//Done before as a whole
				/* Input column j of A */
				//		        if(r == MSK_RES_OK)
				//		          r = MSK_putacol(task,
				//		                          j,                 /* Variable (column) index.*/
				//		                          aptre[j]-aptrb[j], /* Number of non-zeros in column j.*/
				//		                          asub+aptrb[j],     /* Pointer to row indexes of column j.*/
				//		                          aval+aptrb[j]);    /* Pointer to Values of column j.*/

			}

			/* Set the bounds on constraints.
			for i=1, ...,NUMCON : blc[i] <= constraint i <= buc[i] */
			for (i = 0; i<numcon && r == MSK_RES_OK; ++i)
			{
				bool equ = (i >= b.size());
				int ie = (equ ? (i - b.size()) : i);

				double lbi;
				double ubi;
				if (!equ){
					lbi = -MYINF;
					ubi = b[i];
				}
				else{
					lbi = d[ie];	// beyond b.size() quality constraints
					ubi = d[ie];
				}

				r = MSK_putconbound(task,
					i,							/* Index of constraint.*/
					equ ? MSK_BK_FX : MSK_BK_UP,	/* Bound key.*/
					lbi,			/* Numerical value of lower bound.*/
					ubi);			/* Numerical value of upper bound.*/
			}

			if (r == MSK_RES_OK && !linprog)
			{
				/*
				* The lower triangular part of the Q
				* matrix in the objective is specified.
				*/
				unsigned int nnz = H.nonZeros();
				std::vector<int> hi(nnz + 1), hj(nnz + 1);
				std::vector<double> hv(nnz);


				if (_debug){ MYOUT << "H:" << std::endl; }

				/* Hessian Matrix */
				int c = 0;
				for (int k = 0; k<H.outerSize(); ++k){
					for (S::InnerIterator it(H, k); it; ++it)
					{
						if (it.row() >= it.col())
						{ 
							//only lower triangular part
							hv.at(c) = it.value();
							hi.at(c) = it.row();
							hj.at(c) = it.col();
							//if (_debug){ MYOUT << "(" << it.row() << "," << it.col() << ") " << std::setprecision(16) << it.value() << std::endl; }
							++c;
						}
					}
				}

				/* Input the Q for the objective. */
				r = MSK_putqobj(task, nnz, &hi[0], &hj[0], &hv[0]);
			}

			if (_debug){
				MSK_analyzeproblem(task, MSK_STREAM_MSG);
				MSK_writedata(task, "taskdump.opf");
			}

			tSetup.stop();
			if (storeVariables_){
				std::string fileName;
				if (!Loader::uniqueFilename(storePath_ + "/QPMosekDump", ".task", fileName)){
					MYERR << __FUNCTION__ << ": No unique filename for QpMosek dump found. Not storing." << std::endl;
				}
				else{
					MSK_writedata(task, fileName.c_str());
				}
			}
			tSolve.start();

			if (r == MSK_RES_OK)
			{
				MSKrescodee trmcode;

				/* Run optimizer */
				r = MSK_optimizetrm(task, &trmcode);

				/* Print a summary containing information
				about the solution for debugging purposes*/
				if (_debug){ MSK_solutionsummary(task, MSK_STREAM_MSG); }

				if (r == MSK_RES_OK)
				{
					MSKsolstae solsta;
					MSK_getsolsta(task, MSK_SOL_ITR, &solsta);
					xFlag_ = solsta;

					switch (solsta)
					{
					case MSK_SOL_STA_OPTIMAL:
					case MSK_SOL_STA_NEAR_OPTIMAL:
						_x.resize(numvar);
						MSK_getxx(task,
							MSK_SOL_ITR,    /* Request the interior solution. */
							_x.data());

						MSK_getprimalobj(task, MSK_SOL_ITR, &fVal_);
						assert(std::isfinite(fVal_));

						success = true;
						//printf("Optimal primal solution\n");
						//for(j=0; j<numvar; ++j)
						//printf("x[%d]: %e\n",j,xx[j]);

						break;
					case MSK_SOL_STA_DUAL_INFEAS_CER:
					case MSK_SOL_STA_PRIM_INFEAS_CER:
					case MSK_SOL_STA_NEAR_DUAL_INFEAS_CER:
					case MSK_SOL_STA_NEAR_PRIM_INFEAS_CER:
						//printf("Primal or dual infeasibility certificate found.\n");
						break;

					case MSK_SOL_STA_UNKNOWN:
						printf("The status of the solution could not be determined.\n");
						break;
					default:
						//printf("Other solution status.");
						break;
					}
				}
				else
				{
					printf("Error while optimizing.\n");
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

std::string QPMosek::report() const
{
	std::stringstream ss;
	ss << "QP using Mosek. exitFlag: " << xFlag_ << " ( " << exitFlagToString(xFlag_) << " ) " << std::endl;
	ss << "Timing: " << tSetup << " setup, " << tSolve << " solve." << std::endl;
	return ss.str();
}

bool QPMosek::test() const
{
#ifdef MOSEK_EXISTS
	const MSKint32t numvar = 4,
		numcon = 3;

	double       c[] = { 3.0, 1.0, 5.0, 1.0 };	// the linear part in objective function
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
			//  r = MSK_linkfunctotaskstream(task,MSK_STREAM_LOG,NULL,printstr);

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

				if (r == MSK_RES_OK){
					MSKsolstae solsta;
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
	//MSK_deleteenv(&env);

	return (r == MSK_RES_OK);

#else
	return false;
#endif
}

bool QPMosek::solve(const S& H, const V& f, V &_x, const double& d_tol, const double& rot_tol, bool _debug)
{
	bool success = false;

	//number of variables
	MSKint32t numvar = H.rows();
	bool linprog = (numvar == 0);
	if (linprog){ numvar = f.size(); }

	//number of constraints
	MSKint32t numcon = numvar / 3;

	MSKint32t     i, j;

	MSKenv_t      env = env_;
	MSKtask_t     task = NULL;
	MSKrescodee   r = MSK_RES_OK;

	if (r == MSK_RES_OK)
	{
		/* Create the optimization task. */
		r = MSK_maketask(env, numcon, numvar, &task);

		if (nTasks_>0)
		{
			MSK_putintparam(task, MSK_IPAR_NUM_THREADS, nTasks_);
		}


		//set precision: see http://docs.mosek.com/7.0/capi/The_optimizers_for_continuous_problems.html#sec-solve-conic
		MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_PFEAS, mP_);		//Controls primal feasibility, default 10e-8
		MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_DFEAS, mP_);		//Controls dual feasibility, default 10e-8
		MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_REL_GAP, mP_);	//Controls relative gap, default 10e-7
		MSK_putdouparam(task, MSK_DPAR_INTPNT_TOL_INFEAS, mP_);		//Controls when the problem is declared infeasible, default 10e-10
		MSK_putdouparam(task, MSK_DPAR_INTPNT_CO_TOL_MU_RED, mP_);	//Controls when the complementarity is reduced enough, default 10e-8

		if (r == MSK_RES_OK)
		{
			if (_debug){ r = MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, printstr); }

			//Set mosek to use simplex optimizer. Doc says this method should be more suited for hot-start
			// if ( r == MSK_RES_OK )
			//	  r = MSK_putintparam(task,MSK_IPAR_OPTIMIZER,MSK_OPTIMIZER_FREE_SIMPLEX);

			/* Append 'NUMCON' empty constraints.
			The constraints will initially have no bounds. */
			if (r == MSK_RES_OK)
				r = MSK_appendcons(task, numcon);

			/* Append 'NUMVAR' variables.
			The variables will initially be fixed at zero (x=0). */
			if (r == MSK_RES_OK)
				r = MSK_appendvars(task, numvar);

			/* Optionally add a constant term to the objective. */
			if (r == MSK_RES_OK)
				r = MSK_putcfix(task, 0.0);

			if (r == MSK_RES_OK)
			{
				if (_debug) { MYOUT << "Q: " << std::endl; }
				for (int i = 0; i < numvar/6; i++)
				{
					MSKint32t qsubi[] = { 6 * i, 6 * i + 1, 6 * i + 2 };
					MSKint32t qsubj[] = { 6 * i, 6 * i + 1, 6 * i + 2 };
					double	  qval[] = { 2, 2, 2 };
					r = MSK_putqconk(task, 2 * i, 3, qsubi, qsubj, qval);

					MSKint32t qsubri[] = { 6 * i + 3, 6 * i + 4, 6 * i + 5 };
					MSKint32t qsubrj[] = { 6 * i + 3, 6 * i + 4, 6 * i + 5 };
					double	  qrval[] = { 2, 2, 2 };
					r = MSK_putqconk(task, 2 * i + 1, 3, qsubri, qsubrj, qrval);
				}
				
			}


			for (j = 0; j<numvar && r == MSK_RES_OK; ++j)
			{

				/* Set the linear term c_j in the objective.*/
				if (f.size()>0){
					if (r == MSK_RES_OK)
						r = MSK_putcj(task, j, f[j]);
				}
				/* Set the bounds on variable j.
				blx[j] <= x_j <= bux[j] */
				if (r == MSK_RES_OK){

					r = MSK_putvarbound(task,
						j,           /* Index of variable.*/
						MSK_BK_FR,      /* Bound key.*/
						-MYINF,      /* Numerical value of lower bound.*/
						MYINF);     /* Numerical value of upper bound.*/
				}
			}

			/* Set the bounds on constraints.
			for i=1, ...,NUMCON : blc[i] <= constraint i <= buc[i] */
			for (i = 0; i<numcon && r == MSK_RES_OK; ++i)
			{
				if (0 == i % 2)
				{
					r = MSK_putconbound(task,
						i,							/* Index of constraint.*/
						MSK_BK_UP,	/* Bound key.*/
						-MYINF,			/* Numerical value of lower bound.*/
						d_tol);			/* Numerical value of upper bound.*/
				}
				else
				{
					r = MSK_putconbound(task,
						i,							/* Index of constraint.*/
						MSK_BK_UP,	/* Bound key.*/
						-MYINF,			/* Numerical value of lower bound.*/
						rot_tol);			/* Numerical value of upper bound.*/
				}

			}

			if (r == MSK_RES_OK && !linprog)
			{
				/*
				* The lower triangular part of the Q
				* matrix in the objective is specified.
				*/
				unsigned int nnz = H.nonZeros();
				std::vector<int> hi(nnz + 1), hj(nnz + 1);
				std::vector<double> hv(nnz);


				if (_debug){ MYOUT << "H:" << std::endl; }

				/* Hessian Matrix */
				int c = 0;
				for (int k = 0; k<H.outerSize(); ++k){
					for (S::InnerIterator it(H, k); it; ++it)
					{
						if (it.row() >= it.col())
						{
							//only lower triangular part
							hv.at(c) = it.value();
							hi.at(c) = it.row();
							hj.at(c) = it.col();
							//if (_debug){ MYOUT << "(" << it.row() << "," << it.col() << ") " << std::setprecision(16) << it.value() << std::endl; }
							++c;
						}
					}
				}

				/* Input the Q for the objective. */
				r = MSK_putqobj(task, nnz, &hi[0], &hj[0], &hv[0]);
			}

			if (_debug)
			{
				MSK_analyzeproblem(task, MSK_STREAM_MSG);
				MSK_writedata(task, "taskdump.opf");
			}

			if (storeVariables_)
			{
				std::string fileName;
				if (!Loader::uniqueFilename(storePath_ + "/QPMosekDump", ".task", fileName)){
					MYERR << __FUNCTION__ << ": No unique filename for QpMosek dump found. Not storing." << std::endl;
				}
				else{
					MSK_writedata(task, fileName.c_str());
				}
			}

			if (r == MSK_RES_OK)
			{
				MSKrescodee trmcode;

				/* Run optimizer */
				r = MSK_optimizetrm(task, &trmcode);

				/* Print a summary containing information
				about the solution for debugging purposes*/
				if (_debug){ MSK_solutionsummary(task, MSK_STREAM_MSG); }

				if (r == MSK_RES_OK)
				{
					MSKsolstae solsta;
					MSK_getsolsta(task, MSK_SOL_ITR, &solsta);
					xFlag_ = solsta;

					switch (solsta)
					{
					case MSK_SOL_STA_OPTIMAL:
					case MSK_SOL_STA_NEAR_OPTIMAL:
						_x.resize(numvar);
						MSK_getxx(task,
							MSK_SOL_ITR,    /* Request the interior solution. */
							_x.data());

						MSK_getprimalobj(task, MSK_SOL_ITR, &fVal_);
						assert(std::isfinite(fVal_));

						success = true;
						//printf("Optimal primal solution\n");
						//for(j=0; j<numvar; ++j)
						//printf("x[%d]: %e\n",j,xx[j]);

						break;
					case MSK_SOL_STA_DUAL_INFEAS_CER:
					case MSK_SOL_STA_PRIM_INFEAS_CER:
					case MSK_SOL_STA_NEAR_DUAL_INFEAS_CER:
					case MSK_SOL_STA_NEAR_PRIM_INFEAS_CER:
						//printf("Primal or dual infeasibility certificate found.\n");
						break;

					case MSK_SOL_STA_UNKNOWN:
						printf("The status of the solution could not be determined.\n");
						break;
					default:
						//printf("Other solution status.");
						break;
					}
				}
				else
				{
					printf("Error while optimizing.\n");
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
		}
		MSK_deletetask(&task);
	}
	return success;
}


std::string QPMosek::exitFlagToString(int _xflag) const
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

#include "TSPSolver.h"

#define MAXCUTROUNDS    2
#define MAXADDPERROUNDS 1000

// Convert 2-dim matrix index to 1-dim vector index
#define IJ(i,j,n) (n*i + j)

TSPSolver::TSPSolver() :maxrounds_(MAXCUTROUNDS), maxcuts_(MAXADDPERROUNDS)
{
}

TSPSolver::TSPSolver(MX *_cost) : maxrounds_(MAXCUTROUNDS), maxcuts_(MAXADDPERROUNDS)
{
	SetCostMatrix(*_cost);
}

TSPSolver::TSPSolver(SpMat *_cost) : maxrounds_(MAXCUTROUNDS), maxcuts_(MAXADDPERROUNDS)
{
	SetCostMatrix(*_cost);
}

TSPSolver::~TSPSolver()
{
	MSK_deletetask(&task_);
	MSK_deleteenv(&env_);
}

#ifdef MOSEK_EXISTS
/* This function prints log output from MOSEK to the terminal. */
void MSKAPI TSP_printstr(void *handle,
	MSKCONST char str[])
{
	(void)handle;
	printf("%s", str);
} /* printstr */
#endif

void TSPSolver::SetCostMatrix(SpMat &_costM)
{
	N_ = _costM.rows();
	cost_.resize(N_*N_);

	for (int i = 0; i < N_; i++)
	{
		for (int j = 0; j < N_; j++)
		{
			cost_[i*N_ + j] = _costM.coeff(i, j);
		}
	}
}

void TSPSolver::SetCostMatrix(MX  &_costM)
{
	N_ = _costM.rows();
	cost_.resize(N_*N_);

	for (int i = 0; i < N_; i++)
	{
		for (int j = 0; j < N_; j++)
		{
			cost_[i*N_ + j] = _costM(i, j);
		}
	}
}

void TSPSolver::AddVars()
{
	int ij;
	int n2 = N_*N_;

	r_ = MSK_appendvars(task_, n2);
	assert(r_ == MSK_RES_OK);

	for (ij = 0; ij < n2; ++ij)
	{
		// MSK_ACC_VAR : access data by columns (variable oriented)
		r_ = MSK_putbound(task_, MSK_ACC_VAR, ij, MSK_BK_RA, 0, 1);
		assert(r_ == MSK_RES_OK);
		r_ = MSK_putvartype(task_, ij, MSK_VAR_TYPE_INT);
		assert(r_ == MSK_RES_OK);
	}

	for (ij = 0; ij < N_; ij++)
	{
		// fix all the x_ii to zero
		r_ = MSK_putbound(task_, MSK_ACC_VAR, IJ(ij, ij, N_), MSK_BK_FX, 0, 0);
		assert(r_ == MSK_RES_OK);
	}
}

void TSPSolver::AddObjFunctions()
{
	int ij;
	int n2 = N_ ^ 2;
	r_ = MSK_putcfix(task_, 0.0);
	assert(r_ == MSK_RES_OK);

	for (ij = 0; ij < n2; ++ij)
	{
		r_ = MSK_putcj(task_, ij, cost_[ij]);
		assert(r_ == MSK_RES_OK);
	}
}

void TSPSolver::AddAssignCons()
{
	int i, j;
	double *aval;	    // Number of non-zeros in row i of A.
	int	   *asub;		// Row indexes of non - zero values in row i of A.

	aval = (double*)malloc(N_*sizeof(double)); 
	assert(aval);
	
	asub = (int*)malloc(N_*sizeof(int)); 
	assert(asub);

	for (i = 0; i < N_; i++)
	{
		aval[i] = 1;
	}

	// Apeend 'n*2' empty constraints. The constraints will initially have no bounds
	r_ = MSK_appendcons(task_, N_ * 2);
	assert(r_ == MSK_RES_OK);

	/* Constraint 0--(n-1) is \sum_j x_{ij} = 1 */
	for (i = 0; i < N_; i++)
	{
		r_ = MSK_putbound(task_, MSK_ACC_CON, i, MSK_BK_FX, 1, 1);
		assert(r_ == MSK_RES_OK);

		for (j = 0; j < N_; j++)
		{
			asub[j] = IJ(i, j, N_);
		}

		r_= MSK_putarow(task_, i, N_, asub, aval);
		assert(r_ == MSK_RES_OK);
	}

	/* Constraint n--(2n-1) is \sum_i x_{ij} = 1 */
	for (j = 0; j < N_; j++)
	{
		r_ = MSK_putbound(task_, MSK_ACC_CON, j + N_, MSK_BK_FX, 1, 1);
		assert(r_ == MSK_RES_OK);
		for (i = 0; i < N_; i++)
		{
			asub[i] = IJ(i, j, N_);
		}

		r_ = MSK_putarow(task_, j + N_, N_, asub, aval);
		assert(r_ == MSK_RES_OK);
	}
	free(aval);
	free(asub);
}

void TSPSolver::AddMTZCons()
{
	int varidx, conidx, i, j;

	r_ = MSK_getnumvar(task_, &varidx); 
	assert(r_ == MSK_RES_OK);
	
	r_ = MSK_getnumcon(task_, &conidx); 
	assert(r_ == MSK_RES_OK);

	// add the vars u_k for k = 1 ... (n-1) getting index
	// from varidx to varidx + n - 2
	r_ = MSK_appendvars(task_, N_ - 1);
	assert(r_ == MSK_RES_OK);

	for (i = varidx; i < varidx + N_ - 1; ++i)
	{
		// set bound: 2 <= u_k <= n, k=1..(n-1)
		r_ = MSK_putbound(task_, MSK_ACC_VAR, i, MSK_BK_RA, 2, N_);
		assert(r_ == MSK_RES_OK);
	}
	
	// add the (n-1)^2 constraints:
    // u_i - u_j + 1 <= (n - 1)(1 - x_ij) or equivalently
	// u_i - u_j + (n - 1)x_ij <= n - 2, for i,j != 0
	r_ = MSK_appendcons(task_, (N_ - 1)*(N_ - 1)); 
	assert(r_ == MSK_RES_OK);

	for (i = 1; i < N_; i++)
	{
		for (j = 1; j < N_; j++)
		{
			double aval[3];
			int	   asub[3];

			aval[0] = 1; 
			aval[1] = -1; 
			aval[2] = N_ - 1;
			
			asub[0] = varidx + i - 1; /* u_i */
			asub[1] = varidx + j - 1; /* u_j */
			asub[2] = IJ(i, j, N_);   /* x_ij */
			
			r_ = MSK_putbound(task_, MSK_ACC_CON, conidx, MSK_BK_UP, -MSK_INFINITY, N_ - 2);
			assert(r_ == MSK_RES_OK);

			r_ = MSK_putacol(task_, conidx, 3, asub, aval);
			assert(r_ == MSK_RES_OK);
			
			conidx++;
		}
	}
}

int* TSPSolver::SubtoursToList(int nextnode[], int subtour[], int chosen[], int k, int *size)
{
	int ncities, i, j;		// ncities : number of cities in current subtour
	int *cities;			// index of cities in this subtour

	cities = (int*)malloc(N_*sizeof(int));
	assert(cities);

	ncities = 0;
	
	for (i = 0; i < k; i++)
	{
		int subtourstart = subtour[chosen[i]];
		j = subtourstart;
		do
		{
			cities[ncities] = j;
			ncities++;
			j = nextnode[j];
		} while (j != subtourstart);
	}

	*size = ncities;
	return cities;
}

void TSPSolver::AddCut(int citylist[], int size)
{
	int i, j, asubidx, conidx;
	double *aval;
	int	   *asub;

	int size2 = size * size;

	aval = (double*)malloc(size2*sizeof(double)); 
	assert(aval);
	
	asub = (int*)malloc(size2*sizeof(int)); 
	assert(asub);
	
	for (i = 0; i < size2; i++)
	{
		aval[i] = 1;
	}

	r_ = MSK_getnumcon(task_, &conidx); 
	assert(r_ == MSK_RES_OK);
	
	r_ = MSK_appendcons(task_, 1); 
	assert(r_ == MSK_RES_OK);

	// Adds the subtour constraint given by the list cities S:
	// \sum_{i,j \in S} x_{ij} \leq |S|-1 
	r_ = MSK_putbound(task_, MSK_ACC_CON, conidx, MSK_BK_UP, -MSK_INFINITY, size - 1);
	assert(r_ == MSK_RES_OK);

	asubidx = 0;
	for (i = 0; i < size; i++)
	{
		for (j = 0; j < size; j++)
		{
			asub[asubidx] = IJ(citylist[i], citylist[j], N_);
			asubidx++;
		}
	}

	r_ = MSK_putarow(task_, conidx, size2, asub, aval);
	assert(r_ == MSK_RES_OK);
	
	free(aval);
	free(asub);
}

void TSPSolver::AddCuts()
{
	int i, j, k;
	int n2 = N_*N_;

	double *xx;
	int *nextnode, *visited, *subtour, *chosen;
	int nsubt = 0;

	xx		 = (double*)malloc(n2*sizeof(double));
	nextnode = (int*)malloc(N_*sizeof(int));
	assert(xx);
	assert(nextnode);

	r_ = MSK_getsolutionslice(task_, MSK_SOL_ITG, MSK_SOL_ITEM_XX, 0, n2, xx);
	assert(r_ == MSK_RES_OK);

	// Convert matrix representation of graph (xx) to
	// adjacency(-list) (nextnode)
	for (i = 0; i < N_; i++)
	{
		for (j = 0; j<N_; j++)
		{
			if (xx[IJ(i, j, N_)]>0.5)
			{				
				// i.e. x_ij = 1
				nextnode[i] = j;
			}
		}
	}
	free(xx); 
	xx = NULL;

	visited = (int*)calloc(N_,  sizeof(int)); // visited is initialized to 0
	subtour = (int*)malloc(N_ * sizeof(int));
	assert(visited);
	assert(subtour);

	/* identify subtours; keep count in nsubt, save starting
	* pointers in subtour[0..(nsubt-1)] */
	for (i = 0; i < N_; i++)
	{
		if (!visited[i]) 
		//find an unvisited node this starts a new subtour
		{
			subtour[nsubt] = i;	// startpoint index
			nsubt++;
			j = i;
			do
			{
				// traverse all the cities in this tour
				assert(!visited[j]);
				visited[j] = 1;
				j = nextnode[j];
			} while (j != i);
		}
	}
	free(visited); visited = NULL;
	
	nsubtours_ = nsubt;		// Number of subtour
	ncuts_ = 0;				// Number of cuts
	chosen = (int*)malloc(nsubt*sizeof(int)); /* list of chosen subtours */

	for (k = 1; k <= nsubt; k++) /* choose k of nsubt subtours */
	{
		int nchosen = 1;
		chosen[0] = nsubt - 1;
		while (ncuts_ < maxcuts_)
		{
			if (nchosen == k)
			{
				int *citylist;
				int size;
				citylist = SubtoursToList(nextnode, subtour,
					chosen, k, &size);
				if (size <= N_ / 2) 
				// add only subtour constraints of size n/2 or less
				{
					AddCut(citylist, size);
					ncuts_++;
				}
				free(citylist);
				j = 0;
				while (j < k && chosen[k - 1 - j] == j)
				{
					j++;
				}
				if (k == j) break; /* all k-size subsets done */
				nchosen = k - j;
				chosen[nchosen - 1]--;
			}
			else /* 0 < nchosen < k */
			{
				chosen[nchosen] = chosen[nchosen - 1] - 1;
				nchosen++;
			}
		}
	}
	free(nextnode);
	free(subtour);
	free(chosen);
}

bool TSPSolver::Solve(VX &_x, bool _debug)
{

	// Create the mosek environment and empty task
	// Assocaite

	r_ = MSK_makeenv(&env_, NULL);
	assert(r_ == MSK_RES_OK);
	
	r_ = MSK_initenv(env_);                      
	assert(r_ == MSK_RES_OK);
	
	r_ = MSK_makeemptytask(env_, &task_);          
	assert(r_ == MSK_RES_OK);

	MSK_linkfunctotaskstream(task_, MSK_STREAM_LOG, NULL, TSP_printstr);

	AddVars();
	AddObjFunctions();
	AddAssignCons();

	double cut_time = 0;
	nsubtours_ = 2;

	double t;
	for (int k = 0; k < maxrounds_; k++)
	{
		r_ = MSK_optimize(task_);
		assert(r_ == MSK_RES_OK);

		r_ = MSK_getprimalobj(task_, MSK_SOL_ITG, &ObjVal_);
		assert(r_ == MSK_RES_OK);

		MSK_getdouinf(task_, MSK_DINF_OPTIMIZER_TIME, &t);
		cut_time += t;

		AddCuts();
		printf("----------------------------"
			"TSPSolver\n"
			"Round : %d\n"
			"ObjValue : %e\n"
			"Number of subtours : %d\n"
			"Number of cuts added : %d\n\n", k + 1, ObjVal_, nsubtours_, ncuts_);
		if (nsubtours_ == 1)
		{
			break; // problem solved!
		}
	}

	t = 0;
	if (nsubtours_ > 1)
	{
		printf("TSPSolver: Adding MTZ arc constraints\n");
		AddMTZCons();
		r_ = MSK_optimize(task_);                         
		assert(r_ == MSK_RES_OK);

		r_ = MSK_getprimalobj(task_, MSK_SOL_ITG, &ObjVal_);
		assert(r_ == MSK_RES_OK);
		
		MSK_getdouinf(task_, MSK_DINF_OPTIMIZER_TIME, &t);
	}

	if (_debug)
	{
		MSK_solutionsummary(task_, MSK_STREAM_MSG);
	}

	// information stream
	if (r_ == MSK_RES_OK)
	{
		// solution status
		MSKsolstae solsta;
		MSK_getsolsta(task_, MSK_SOL_ITG, &solsta);

		switch (solsta)
		{
		case MSK_SOL_STA_OPTIMAL:
		case MSK_SOL_STA_NEAR_OPTIMAL:
			_x.resize(N_ * N_);
			MSK_getxx(task_,
				MSK_SOL_BAS,    // Request the basic solution.
				_x.data());

			MSK_getprimalobj(task_, MSK_SOL_ITR, &ObjVal_);
			assert(std::isfinite(ObjVal_));

			printf("Mosek Lp Optimal primal solution\n");
			/*for (int j = 0; j < N_ * N_; ++j)
			{
				printf("x[%d]: %e\n", j, _x[j]);
			}*/

			break;
		case MSK_SOL_STA_DUAL_INFEAS_CER:
		case MSK_SOL_STA_PRIM_INFEAS_CER:
		case MSK_SOL_STA_NEAR_DUAL_INFEAS_CER:
		case MSK_SOL_STA_NEAR_PRIM_INFEAS_CER:
			printf("Primal or dual infeasibility certificate found.\n");
			break;

		case MSK_SOL_STA_UNKNOWN:
		{
			printf("The status of the solution could not be determined.\n");
			break;
		}

		case MSK_SOL_STA_NEAR_INTEGER_OPTIMAL:
		case MSK_SOL_STA_INTEGER_OPTIMAL:
			_x.resize(N_ * N_);
			MSK_getxx(task_,
				MSK_SOL_ITG,    // Request the basic solution.
				_x.data());
			
			MSK_getprimalobj(task_, MSK_SOL_ITG, &ObjVal_);
			assert(std::isfinite(ObjVal_));

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

	printf("\n"
		"Done solving.\n"
		"Time spent cutting: %.2f\n"
		"Total time spent: %.2f\n"
		"ObjValue: %e\n", cut_time, cut_time + t, ObjVal_);

	return true;
}
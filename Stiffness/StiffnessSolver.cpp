#include "StiffnessSolver.h"

/* Eigen solver */
bool StiffnessSolver::SolveSystem(SpMat &K, VX &D, VX &F, int verbose, int &info)
{
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    solver.compute(K);
    info = 0;
    
    if (solver.info() != Eigen::Success)
    {
        fprintf(stderr, "SolverSystem(LDLT): Error in Decomposition!\n");
        return false;
    }

    VX Diag = solver.vectorD();
        
    for (int i = 0; i < Diag.size(); i++)
    {
        if (Diag[i] == 0.0)
        {
            fprintf(stderr, " SolveSystem(LDLT): zero found on diagonal ...\n");
            fprintf(stderr, " d[%d] = %11.4e\n", i, Diag[i]);
			return false;
        }

        if (Diag[i] < 0.0)
        {
            fprintf(stderr, " SolveSystem(LDLT): negative number found on diagonal ...\n");
            fprintf(stderr, " d[%d] = %11.4e\n", i, Diag[i]);
            info--;
			return false;
        }
    }

    if (info < 0)
    {
		fprintf(stderr, "Stiffness Matrix is not positive definite: %d negative elements\n", info);
		fprintf(stderr, "found on decomp diagonal of K.\n");
		fprintf(stderr, "The stucture may have mechanism and thus not stable in general\n");
		fprintf(stderr, "Please Make sure that all six\n");
		fprintf(stderr, "rigid body translations are restrained!\n");

		return false;
    }

    D = solver.solve(F);
    if (solver.info() != Eigen::Success)
    {
        fprintf(stderr, "SolverSystem(LDLT): Error in Solving!\n");
		return false;
    }

	return true;
}


bool StiffnessSolver::SolveSystem(SpMat &K, VX &D, VX &F, VX &D0, int verbose, int &info)
{
	Eigen::ConjugateGradient<Eigen::SparseMatrix<double>> solver;
	solver.compute(K);
	info = 0;
	
	if (solver.info() != Eigen::Success)
	{
		fprintf(stderr, "SolverSystem(ConjugateGradient): Error in Decomposition!\n");
		return false;
	}
	
	solver.setMaxIterations(3000);
	D = solver.solve(F);

	if (solver.info() != Eigen::Success)
	{
		fprintf(stderr, "SolverSystem(ConjugateGradient): Error in Solving!\n");
		return false;
	}

	return true;
}

bool StiffnessSolver::LUDecomp(
	MX &A,
	VX &x,
	VX &b	)
{
	x = A.fullPivLu().solve(b);

	if ((A*x).isApprox(b))
	{
		return true;
	}
	else
	{
		return false;
	}
}

void StiffnessSolver::Debug()
{
}
#ifndef TSP_SOLVER_H
#define TSP_SOLVER_H

#include <Eigen/Sparse>
#include <iostream>

#define MOSEK_EXISTS

#ifdef MOSEK_EXISTS
#include <mosek.h>
#endif

// solves min c^t * x subject to lc < A * x < uc, x >= lb, x <= ub
// Formulate TSP using "Integer Programming Formulation of Traveling Salesman Problems" http://dl.acm.org/citation.cfm?id=321046
// i,e, using MTZ constraints to keep subtour clear.

class TSPSolver
{
public:
	typedef Eigen::MatrixXd				MX;
	typedef Eigen::SparseMatrix<double> SpMat;
	typedef Eigen::VectorXd				VX;
public:
	TSPSolver();
	TSPSolver(const SpMat  &_cost);
	TSPSolver(const MX	   &_cost);
	~TSPSolver();

	// I/O
	void SetCostMatrix(const MX &_costM);
	void SetCostMatrix(const SpMat &_costM);

	bool Solve(VX &_x, bool _debug);

	// Sub-procedure in solving process
	void AddVars();
	void AddObjFunctions();
	void AddAssignCons();
	
	void AddMTZCons();										// Add the Miller-Tucker-Zemlin arc constraints
	int *SubtoursToList(int nextnode[], 
		int subtour[], int chosen[], int k, int *size);		// Construct the list of cities in the chosen subtours
	void AddCut(int citylist[], int size);					// Adds the subtour constraint given by the list cities S:
															// sum_{i,j \in S} x_{ij} <= |S|-1 
	void AddCuts();											// Identifies subtours and adds a number of violated cuts
private:
	VX		    cost_;
	int			N_;
	double		ObjVal_;
	VX			lb_, ub_;			// Lower variable bound and Upper variable bound
	VX			lc_, uc_;			// lc <= A * x <= uc, constraints upper and lower bound 
	MX			A_;

	int			maxrounds_;
	int			maxcuts_;
	int			nsubtours_, ncuts_;

	MSKrescodee		r_;				// Mosek return code
	MSKenv_t		env_;			// Mosek enviroment
	MSKtask_t       task_;			// Mosek task
};
#endif
#pragma once

class FiberPrintPARM
{
public:
	FiberPrintPARM();
	FiberPrintPARM(double radius, double density, double g, double youngs_modulus, double shear_modulus,
					double penalty, double D_tol, double pri_tol, double dual_tol, double alpha, double beta, double gamma);
	~FiberPrintPARM();

public:
	// stiffness
	double		radius_;
	double		density_;
	double		g_;
	double		youngs_modulus_;
	double		shear_modulus_;

	// ADMM
	double		penalty_;		// penalty  : penalty factor used in ADMM  
	double		D_tol_;			// D_tol    : tolerance in D-Qp problem constraints
	double		pri_tol_;		// pri_tol  : primal residual tolerance for ADMM termination criterion
	double		dual_tol_;		// dual_tol : dual   residual tolerance for ADMM termination criterion

	// TSP
	double		alpha_;
	double		beta_;
	double		gamma_;
};


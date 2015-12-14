#pragma once

class FiberPrintPARM
{
public:
	FiberPrintPARM();
	FiberPrintPARM(double radius, double density, double g,
					double youngs_modulus, double shear_modulus,
					double D_tol, double penalty, double pri_tol,
					double dual_tol, double gamma, double Wl, double Wp);
	~FiberPrintPARM();

public:
	// material & environment
	double		radius_;
	double		density_;
	double		g_;
	double		youngs_modulus_;
	double		shear_modulus_;
	double		poisson_ratio_;

	// stiffness
	double		D_tol_;			// D_tol    : tolerance in D-Qp problem constraints

	// ADMM
	double		penalty_;		// penalty  : penalty factor used in ADMM  
	double		pri_tol_;		// pri_tol  : primal residual tolerance for ADMM termination criterion
	double		dual_tol_;		// dual_tol : dual   residual tolerance for ADMM termination criterion

	// Sequence Analyzer
	double		gamma_;
	double		Wl_;
	double		Wp_;
};


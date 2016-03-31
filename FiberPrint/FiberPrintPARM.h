#pragma once

#include "GlobalFunctions\GCommon.h"

class FiberPrintPARM
{
public:
	FiberPrintPARM();
	FiberPrintPARM(double Wl, double Wp, double Wa);
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
	double		Dt_tol_;		// Dt_tol   : tolerance of offset in stiffness
	double		Dr_tol_;		// Dr_tol   : tolerance of rotation in stiffness

	// ADMM
	double		penalty_;		// penalty  : penalty factor used in ADMM  
	double		pri_tol_;		// pri_tol  : primal residual tolerance for ADMM termination criterion
	double		dual_tol_;		// dual_tol : dual   residual tolerance for ADMM termination criterion

	// Sequence Analyzer
	double		Wl_;
	double		Wp_;
	double		Wa_;
};


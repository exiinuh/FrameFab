#pragma once

#include "GlobalFunctions\GCommon.h"

class FiberPrintPARM
{
public:
	FiberPrintPARM(
		double Wp = 1.0,
		double Wa = 1.0,
		double Wi = 5.0,
		double seq_D_tol = 2.0,
		double ADMM_D_tol = 1.0,
		double penalty = 1e2,
		double pri_tol = 1e-2,
		double dual_tol = 1e-2,
		double radius = 0.75,
		double density = 1210 * 1e-12,
		double g = -9806.3,
		double youngs_modulus = 3457,
		double shear_modulus = 1294,
		double poisson_ratio = 0.335
		);

	~FiberPrintPARM();

public:
	// material & environment
	double		radius_;
	double		density_;
	double		g_;
	double		youngs_modulus_;
	double		shear_modulus_;
	double		poisson_ratio_;

	// ADMM
	double		ADMM_D_tol_;	// ADMM_D_tol_	: tolerance of offset in stiffness for ADMMCut 
	double		penalty_;		// penalty		: penalty factor used in ADMM  
	double		pri_tol_;		// pri_tol		: primal residual tolerance for ADMM termination criterion
	double		dual_tol_;		// dual_tol		: dual   residual tolerance for ADMM termination criterion

	// Sequence Analyzer
	double		Wp_;
	double		Wa_;
	double		Wi_;
	double		seq_D_tol_;		// seq_D_tol_   : tolerance of offset in stiffness for SeqAnalyzer 
};


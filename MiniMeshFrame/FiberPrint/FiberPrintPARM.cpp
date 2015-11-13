#include "FiberPrintPARM.h"


FiberPrintPARM::FiberPrintPARM()
			   :radius_(0.0015), density_(0.001), g_(9.80), youngs_modulus_(1586), shear_modulus_(1387),
			   penalty_(10e2), D_tol_(0.1), pri_tol_(10e-3), dual_tol_(10e-3), alpha_(1.0), beta_(10000.0), gamma_(100.0)
{
}


FiberPrintPARM::FiberPrintPARM(double radius, double density, double g, double youngs_modulus, double shear_modulus,
	double penalty, double D_tol, double pri_tol, double dual_tol, double alpha, double beta, double gamma)
			   :radius_(radius), density_(density), g_(g), youngs_modulus_(youngs_modulus), shear_modulus_(shear_modulus),
			   penalty_(penalty), D_tol_(D_tol), pri_tol_(pri_tol), dual_tol_(dual_tol), alpha_(alpha), beta_(beta), gamma_(gamma)
{
}


FiberPrintPARM::~FiberPrintPARM()
{
}

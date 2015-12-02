#include "FiberPrintPARM.h"


FiberPrintPARM::FiberPrintPARM()
			   :penalty_(10e2), D_tol_(0.1), pri_tol_(1e-4), dual_tol_(1e-4), alpha_(1.0), beta_(10000.0), gamma_(100.0)
{
	/*
	*	For Your Inference; Unit Transfer
	*	Gpa = 10^3Mpa = 10^9pa
	*   1 kg/m^3 = 1e-3 g/cm^3 = 1e-12 Ton/mm^3
	*/
	
	/* 
	* Test case 1: Acrylonitrile Butadiene Styrene (ABS)
	* Data Source : http://www.grantadesign.com/education/datasheets/ABS.htm
	* Density ;						1210 * 1e-12 Ton/mm^3
	* Elastic(Young's) Modulus :    1100 Mpa
	* Shear Modulus :				1032 Mpa
	* radius of the element :		0.4	 mm
	* poisson ratio :				0.39
	*/

	/*
	* Gravity acceleration along Z axis:   gZ = -9806.33 mm/s^2
	*/

	radius_ = 0.4;
	//density_ = 1210 * 1e-12;
	density_ = 12600 * 1e-12;
	g_ = -9806.33;
	youngs_modulus_ = 1100;
	shear_modulus_ = 1032;
	poisson_ratio_ = 0.39;
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

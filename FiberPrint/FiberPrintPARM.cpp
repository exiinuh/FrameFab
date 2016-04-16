#include "FiberPrintPARM.h"


FiberPrintPARM::FiberPrintPARM()
{
	/*
	*	For Your Inference; Unit Transfer
	*	Gpa = 10^3Mpa = 10^9pa
	*   1 kg/m^3 = 1e-3 g/cm^3 = 1e-12 Ton/mm^3
	*	1 Pa = 1 N/mm^2
	*	1 N  = kg/(m * s^2)
	*	1 Giga = 10^3 Mega = 10^6
	*/
	
	/* 
	* Test case 1: Acrylonitrile Butadiene Styrene (ABS)
	* Data Source : http://www.grantadesign.com/education/datasheets/ABS.htm
	* Density ;						1210 * 1e-12 Ton/mm^3
	* Elastic(Young's) Modulus :    1100 Mpa
	* Shear Modulus :				1032 Mpa
	* radius of the element :		0.6	 mm
	* poisson ratio :				0.39
	*/

	/*
	* Gravity acceleration along Z axis:   gZ = -9806.33 mm/s^2
	*/

	radius_ = 0.6;
	density_ = 1210 * 1e-12;
	g_ = -9806.33;
	youngs_modulus_ = 1100;
	shear_modulus_ = 1032;
	poisson_ratio_ = 0.39;
	Dt_tol_ = 5.0;
	Dr_tol_ = 10 * F_PI / 180;
	penalty_ = 1e3;
	pri_tol_ = 1e-3;
	dual_tol_ = 1e-3;

	Wp_ = 1.0;
	Wa_ = 1.0;
	Wi_ = 5.0;
}


FiberPrintPARM::FiberPrintPARM(double Wp, double Wa, double Wi)
{
	radius_ = 0.6;
	density_ = 1210 * 1e-12;
	g_ = -9806.33;
	youngs_modulus_ = 1100;
	shear_modulus_ = 1032;
	poisson_ratio_ = 0.39;
	Dt_tol_ = 5.0;
	Dr_tol_ = 10 * F_PI / 180;
	penalty_ = 1e3;
	pri_tol_ = 1e-3;
	dual_tol_ = 1e-3;

	Wp_ = Wp;
	Wa_ = Wa;
	Wi_ = Wi;
}


FiberPrintPARM::~FiberPrintPARM()
{
}

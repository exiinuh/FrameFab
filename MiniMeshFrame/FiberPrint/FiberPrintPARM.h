#pragma once

class FiberPrintPARM
{
public:
	FiberPrintPARM();
	~FiberPrintPARM();

public:
	inline double Radius(){ return radius_; }
	inline double Density(){ return density_; }
	inline double G(){ return g_; }
	inline double YoungsModulus(){ return youngs_modulus_; }
	inline double ShearModulus(){ return shear_modulus_; }

private:
	double	radius_;
	double	density_;
	double	g_;
	double	youngs_modulus_;
	double	shear_modulus_;
};


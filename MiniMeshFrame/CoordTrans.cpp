#include "CoordTrans.h"

void CoordTrans::CreateTransMatrix(
	std::vector<V3> xyz,
	double L,			// length of the element(edge)
	int n1, int n2,		// index fo endpoint of the element
	double &t1, double &t2, double &t3, double &t4, double &t5,
	double &t6, double &t7, double &t8, double &t9,
	float p)
{
	//  CoordTrans - calculate the 9 elements of the block - diagonal 12 - by - 12
	//	coordinate transformation matrix, t1, t2, ..., t9.

	//	These coordinate transformation factors are used to :
	//  transform frame element end forces from the element(local) coordinate system
	//	to the structral(global) coordinate system.

	//	Element matrix coordinate transformations are carried out by function ATMA

	double	Cx, Cy, Cz, den,		/* direction cosines	*/
		Cp, Sp;			/* cosine and sine of roll angle */

	Cx = (xyz[n2][0] - xyz[n1][0]) / L;
	Cy = (xyz[n2][1] - xyz[n1][1]) / L;
	Cz = (xyz[n2][2] - xyz[n1][2]) / L;

	t1 = t2 = t3 = t4 = t5 = t6 = t7 = t8 = t9 = 0.0;

	Cp = cos(p);
	Sp = sin(p);

	if (fabs(Cz) == 1.0)
	{
		t3 = Cz;
		t4 = -Cz*Sp;
		t5 = Cp;
		t7 = -Cz*Cp;
		t8 = -Sp;
	}
	else
	{

		den = sqrt(1.0 - Cz*Cz);

		t1 = Cx;
		t2 = Cy;
		t3 = Cz;

		t4 = (-Cx*Cz*Sp - Cy*Cp) / den;
		t5 = (-Cy*Cz*Sp + Cx*Cp) / den;
		t6 = Sp*den;

		t7 = (-Cx*Cz*Cp + Cy*Sp) / den;
		t8 = (-Cy*Cz*Cp - Cx*Sp) / den;
		t9 = Cp*den;
	}

	return;
}

void CoordTrans::TransLocToGlob(double t1, double t2, double t3,
	double t4, double t5, double t6,
	double t7, double t8, double t9,
	MX	   &m, float r1, float r2)
{
	int     i, j, k;

	MX t(12, 12);
	MX mt(12, 12);

	t.setZero();
	mt.setZero();

	for (i = 0; i <= 3; i++) {
		t(3 * i, 3 * i) = t1;
		t(3 * i, 3 * i + 1) = t2;
		t(3 * i, 3 * i + 2) = t3;
		t(3 * i + 1, 3 * i) = t4;
		t(3 * i + 1, 3 * i + 1) = t5;
		t(3 * i + 1, 3 * i + 2) = t6;
		t(3 * i + 2, 3 * i) = t7;
		t(3 * i + 2, 3 * i + 1) = t8;
		t(3 * i + 2, 3 * i + 2) = t9;
	}

	/*  effect of finite node radius on coordinate transformation  is not supported now */

	mt = t.transpose() * m * t;
	m = mt;
}
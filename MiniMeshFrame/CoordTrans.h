/*
	@file : 
	Coordinate transformations for the Fiberprint Structural Analysis SubModule

	Author : Yijiang H.
	Date   : 18 Nov 2015
	Note   : This file modify part of the file coordtrans.h, a part of FRAME3DD
	You can get it at http://frame3dd.sourceforge.net/.
*/

#pragma once
#include <vector>
#include <Eigen\dense>

/**
CoordTrans -  evaluate the 3D coordinate transformation coefficients
Default order of coordinate rotations...  typical for Y as the vertical axis
1. rotate about the global Z axis
2. rotate about the global Y axis
3. rotate about the local  x axis --- element 'roll'

If Zvert is defined as 1, then the order of coordinate rotations is typical
for Z as the vertical axis
1. rotate about the global Y axis
2. rotate about the global Z axis
3. rotate about the local  x axis --- element 'roll'

Q=TF;   U=TD;   T'T=I;   Q=kU;   TF=kTD;   T'TF=T'kTD;   T'kT = K;   F=KD
*/
class CoordTrans{
public:
	typedef		Eigen::MatrixXd MX;
	typedef		Eigen::Matrix3d M3;
	typedef		Eigen::VectorXd VX;
	typedef		Eigen::Vector3d V3;
	typedef		Eigen::VectorXi VXi;
	typedef		Eigen::MatrixXi MXi;
public:
	CoordTrans(){};
	~CoordTrans(){};

	void CreateTransMatrix(
		std::vector<V3>	xyz,
		double L,			// length of the element(edge)
		int n1, int n2,		// index fo endpoint of the element
		double &t1, double &t2, double &t3, double &t4, double &t5,
		double &t6, double &t7, double &t8, double &t9,
		float p);

	void TransLocToGlob(double t1, double t2, double t3,
		double t4, double t5, double t6,
		double t7, double t8, double t9,
		MX	   &m, float r1, float r2);
};
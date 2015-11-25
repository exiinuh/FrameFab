#pragma once

#include <iostream>
#include <assert.h>

#include <Eigen/dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCore>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseLU>
#include <Eigen/SparseQR>
#include <Eigen/Core>
#include <Eigen/OrderingMethods>
#include <Eigen/IterativeLinearSolvers>

#include "WireFrame\WireFrame.h"
#include "FiberPrint\DualGraph.h"
#include "FiberPrint\FiberPrintPARM.h"
#include "CoordTrans.h"

using namespace std;
using namespace Eigen;


class Stiffness
{
public:
	typedef Eigen::SparseMatrix<double> SpMat;
	typedef Eigen::MatrixXd MX;
	typedef Eigen::VectorXd VX;

	Stiffness();
	Stiffness(DualGraph *ptr_dualgraph);
	Stiffness(DualGraph *ptr_dualgraph, FiberPrintPARM *ptr_parm);
	~Stiffness();

public:
	void		Init();
	void		CreateFe();
	void		CreateF(const VectorXd *ptr_x);
	void		CreateElasticK();
	void		CreateGlobalK(const VectorXd *ptr_x);

	// Socket to GraphCut
	void		CalculateD(VectorXd *ptr_D);
	void		CalculateD(VectorXd *ptr_D, const VectorXd *ptr_x);

	// Data I/O
	SpMat		*WeightedK(){ assert(&K_); return &K_; }
	VX			*WeightedF(){ assert(&F_); return &F_; }

	MX			eKe(int ei);			// ei: orig e id
	MX			eKv(int ei);			// ei: orig e id
	VX			Fe(int ei);				// ei: orig e id

	void		Debug();

public:
	//private:
	DualGraph	*ptr_dualgraph_;

	SpMat		K_;						// x-Weighted global stiffness matrix, 6n*6n
	vector<MX>	eK_;					// elastic K, indexed by dual id
	VX			F_;
	vector<VX>	Fe_;

	double		r_;						// radius of frame
	double		nr_;					// radius of node
	double		density_;
	double		g_;
	double		G_;						// shear modulus
	double		E_;						// young's modulus;
	double		v_;						// possion ratio
};

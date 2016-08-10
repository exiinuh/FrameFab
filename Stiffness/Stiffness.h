#ifndef FIBERPRINT_STIFFNESS_H
#define FIBERPRINT_STIFFNESS_H

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
#include "GlobalFunctions\GCommon.h"
#include "StiffnessIO.h"
#include "StiffnessSolver.h"
#include "IllCondDetector.h"

using namespace std;
using namespace Eigen;

class Stiffness
{
public:
	typedef Eigen::SparseMatrix<double> SpMat;
	typedef Eigen::MatrixXd				MX;
	typedef Eigen::VectorXd				VX;
	typedef Eigen::VectorXi				VXi;
	typedef	Eigen::MatrixXi				MXi;
	typedef trimesh::point				point;

public:
	Stiffness();
	Stiffness(DualGraph *ptr_dualgraph);
	Stiffness(DualGraph *ptr_dualgraph, FiberPrintPARM *ptr_parm, char *ptr_path);
	~Stiffness();

public:
	void		Init();
	void		CreateFe();
	void		CreateF(VX *ptr_x = NULL);
	void		CreateElasticK();
	void		CreateGlobalK(VX *ptr_x = NULL);

	/* calculate D using LDLT */
	bool CalculateD(
		VX &D,
		VX *ptr_x = NULL,
		bool verbose = true, bool cond_num = false, bool write_3dd = false,
		int file_id = 0, string file_name = ""
		);

	/* calculate D using ConjugateGradient by Eigen */
	bool CalculateD(
		VX &D, 
		VX &D0,						// D0 is the last result
		VX *ptr_x = NULL,
		bool verbose = true, bool cond_num = false, bool write_3dd = false,
		int file_id = 0, string file_name = ""
		);

	/* Check condition number */
	bool CheckIllCondition(
		IllCondDetector &stiff_inspector, 
		bool verbose = true
		);
	bool CheckError(
		IllCondDetector &stiff_inspector,
		VX &D, 
		bool verbose = true
		);

	/* Write to file */
	void WriteData(
		VectorXd &D, 
		int id = 0, 
		string fname = "stiff_data",
		bool verbose = true
		);

	/* Data I/O */
	SpMat		*WeightedK(){ assert(&K_); return &K_; }
	VX			*WeightedF(){ assert(&F_); return &F_; }

	MX			eKe(int ei);			// ei: orig e id
	MX			eKv(int ei);			// ei: orig e id
	VX			Fe(int ei);				// ei: orig e id

	void		PrintOutTimer();
	
public:
	DualGraph		*ptr_dualgraph_;
	FiberPrintPARM	*ptr_parm_;
	char			*ptr_path_;

	StiffnessIO		stiff_io_;
	StiffnessSolver	stiff_solver_;

	CoordTrans		transf_;

	SpMat			K_;						// x-Weighted global stiffness matrix, 6n*6n
	vector<MX>		eK_;					// elastic K, indexed by dual id
	VX				F_;
	vector<VX>		Fe_;

	int				Ns_;

	double			r_;						// radius of frame
	double			nr_;					// radius of node
	double			density_;
	double			g_;
	double			G_;						// shear modulus
	double			E_;						// young's modulus;
	double			v_;						// possion ratio

	bool			shear_;					// 1 : shear deformation taken into consideration; 0 : not

	Timer			create_fe_;
	Timer			create_f_;
	Timer			create_ek_;
	Timer			create_k_;
	Timer			check_ill_;
	Timer			check_error_;
};
#endif
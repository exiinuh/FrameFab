#ifndef STATISTICS_H
#define STATISTICS_H

#include <iostream>
#include <fstream>
#include <iomanip>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include <vector>
#include <string>

using namespace std;
using namespace Eigen;

#define PATH "C:/Users/DELL/Desktop/result/"

class Statistics{
public:
	Statistics(){}
	Statistics(std::string _name, Eigen::VectorXd _VX, int _iteration) :name_(_name),iteration_(_iteration)
	{
		Vx_ = _VX;
		store_path_ = PATH;
	}
	Statistics(std::string _name, Eigen::VectorXd _VX) :name_(_name),iteration_(-1)
	{
		Vx_ = _VX;
		store_path_ = PATH;
	}

	Statistics(std::string _name, Eigen::SparseMatrix<double> SpMat) :name_(_name), iteration_(-1)
	{ SpMat_ = SpMat; 
	store_path_ = PATH;
	}
	
	Statistics(std::string _name, Eigen::MatrixXd Mat): name_(_name)
	{
		denMat_ = Mat; 
		store_path_ = PATH;
	}

	Statistics(std::string _name, std::vector<double> _stdvec) : name_(_name), iteration_(-1)
	{
		stdVec_ = _stdvec;
		store_path_ = PATH;
	}
	~Statistics(){}

	void StreamVectorOutPut();
	void StreamSpMatrixOutput();
	void StreamDenMatrixOutput();;

	void GenerateVectorFile();
	void GenerateMatrixFile();
	void GenerateSpFile();
	void GenerateStdVecFile();
	bool UniqueFileName();

	bool IsSpSymmetry();
	bool IsDenSymmetry();

	std::string						name_;
	int								iteration_;
	std::string						lastDir_;
	std::string						filename_;
	std::string						store_path_;
	Eigen::SparseMatrix<double>		SpMat_;
	Eigen::MatrixXd					denMat_;
	Eigen::VectorXd					Vx_;
	std::vector<double>				stdVec_;
};

#endif // STATISTICS_H

#ifndef FIBER_PRINT_LP_H
#define FIBER_PRINT_LP_H

#include <Eigen/Sparse>
#include <ostream>

#define MYNAN std::numeric_limits<double>::quiet_NaN()
#define MYINF std::numeric_limits<double>::infinity()

// solves min c^t * x subject to lc < A * x < uc, x >= lb, x <= ub
class LinearP
{
public:
	typedef Eigen::SparseMatrix<double> SpMat;
	typedef Eigen::MatrixXd				MX;
	typedef Eigen::VectorXd				VX;

	LinearP() :storeVariables_(false), storePath_(std::string("F:\\FiberPrintProject\\ResultData\\Lp_data\\")){ ; }
	virtual ~LinearP(){ ; }

	//trying to solve LP described above. Returns true if succeeded, false otherwise.
	virtual bool solve(const VX& c, 
					   const MX& A, const VX& lc, const VX& uc,
				  	   const VX&   lb, const VX& ub,
					   VX& _x,	bool _debug = false, bool _integer_opt = false) = 0;

	virtual std::string report() const = 0;
	virtual double functionValue() const = 0;
	virtual int exitFlag() const = 0;

	void setStoreVariables(bool b){ storeVariables_ = b; }
	bool storeVariables() const { return storeVariables_; }
	bool setStorePath(const std::string& sp = std::string()){ storePath_ = sp; }

	double GetSetupTime() const { return setup_time_; }
	double GetSolveTime() const { return solve_time_; }

	friend std::ostream& operator<<(std::ostream& out, const LinearP& lp);

protected:
	bool			storeVariables_;
	std::string		storePath_;
	double			setup_time_, solve_time_;
};
#endif
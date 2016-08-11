#pragma once

#ifndef ptr_qp_H
#define ptr_qp_H

#include <Eigen/Sparse>
#include <ostream>


#define MYNAN std::numeric_limits<double>::quiet_NaN()
#define MYINF std::numeric_limits<double>::infinity()

	/** Solves the quadratic programming problem:
	min 0.5* xt*H*x + ft*x subject to A*x <= b, C*x = d, x >= lb, x <= ub
	*/

	class Cone{
		// A scaled cone; s * x(idxs(0)) >= sqrt( sum_{0<i<idxs.size()} x(idxs(i))^2)
	public:
		Cone(){ s = 1.; }
		Cone(const std::vector<int>& _i, double _s = 1.){ idxs = _i; s = _s; }

		std::vector<int> idxs;
		double s;
	};

	class QP{
	public:
		typedef Eigen::SparseMatrix<double> S;
		typedef Eigen::VectorXd V;
		typedef std::vector<Cone> Cones;

		// A scaled cone with varibles x_1, ..., x_n and scale s represents quadratic cone constraint
		// x_1 * s >= \sqrt{ x_2^2 + ... = x_n^2}
		class ScaledCone{
		public:
			ScaledCone(const Cone &cone_, double scale_ = 1.0) :
				cone(cone_), scale(scale_){}

			Cone cone;
			double scale;

		};

		typedef std::vector<ScaledCone> ScaledCones;

		QP():storeVariables_(false), storePath_(std::string("./")){ ; }
		virtual ~QP(){ ; }

		virtual bool solve(
			const S& H, const V& f,
			const S& C, const V& d,
			V& _x,
			bool _debug = false) = 0;

		virtual bool solve(
			const S& H, const V& f, 
			V &_x,
			const V &_y,
			const double& d_tol,
			bool _debug) = 0;

		virtual std::string report() const = 0;
		virtual double functionValue() const = 0;
		virtual int exitFlag() const = 0;

		void setStoreVariables(bool b){ storeVariables_ = b; }
		bool storeVariables() const { return storeVariables_; }
		bool setStorePath(const std::string& sp = std::string()){ storePath_ = sp; }

		friend std::ostream& operator<<(std::ostream& out, const QP& qp);

	protected:
		bool storeVariables_;		// option, store variable or not.
		std::string storePath_;
	};


#endif // ptr_qp_H

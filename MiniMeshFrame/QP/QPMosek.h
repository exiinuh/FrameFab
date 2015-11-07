#pragma once

#ifndef QPMOSEK_H
#define QPMOSEK_H


#include "QP.h"
#include "Timer.h"


class QPMosek : public QP
{
public:
	QPMosek();
	virtual ~QPMosek();

	virtual bool solve(const S& H, const V& f,
		const S& A, const V& b,
		const S& C, const V& d,
		const V& lb, const V& ub,
		V& _x, const V* _x0 = NULL,
		const Cones* cones = NULL,
		bool _debug = false);

	virtual std::string report() const;
	virtual double functionValue() const { return fVal_; }
	virtual int exitFlag() const { return xFlag_; }

	bool test() const;

	void setNTasks(int n){ nTasks_ = n; };
	int nTasks() const { return nTasks_; }

	void setThreshold(double t){ mP_ = t; }
	double threshold(){ return mP_; }

	std::string exitFlagToString(int _xFlag) const;

private:
	Timer tSetup, tSolve;
	void* env_;		// environment in Mosek
	double fVal_;	// objective fdunction value
	int xFlag_;		// solution status
	int nTasks_;	// for multi task number
	double mP_;		// threshold
};
#endif // QPMOSEK_H
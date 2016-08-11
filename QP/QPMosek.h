#pragma once

#ifndef QPMOSEK_H
#define QPMOSEK_H


#include "QP.h"
#include "GlobalFunctions\Timer.h"


class QPMosek : public QP
{
public:
	QPMosek();
	virtual ~QPMosek();

	/* for FrameFab graphcut: CalculateX*/
	virtual bool solve(
		const S& H, const V& f,
		const S& C, const V& d,
		V& _x,
		bool _debug = false);
	
	/* for FrameFab graphcut: CalculateD*/
	virtual bool solve(
		const S& H, const V& f, 
		V &_d,
		const V& _x,
		const double& d_tol,
		bool _debug);

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
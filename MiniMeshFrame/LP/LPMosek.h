#ifndef LPMOSEK_H
#define LPMOSEK_H

#include "LinearP.h"
#include "Timer.h"

// solves min c^t * x subject to lc < A * x < uc, x >= lb, x <= ub

// Please note the constraints here only deals with the situation that lc and uc limited.
// If unlimited case is encountered, please change constraints boundkey accordingly.

class LPMosek : public LinearP
{
public:
	LPMosek();
	virtual ~LPMosek();

	virtual bool solve(const VX& c,
		const MX& A, const VX& lc, const VX& uc,
		const VX&   lb, const VX& ub,
		VX& _x, bool _debug = false, bool _integer_opt = false);

	virtual std::string report() const;
	virtual double functionValue() const { return fVal_; }
	virtual int exitFlag() const { return xFlag_; }

	bool test() const;
	std::string exitFlagToString(int _xflag) const;

private:
	Timer			tSetup, tSolve;
	void*			env_;			// environment in Mosek
	double			fVal_;			// objective fdunction value
	int				xFlag_;			// solution status
};
#endif // LPMOSEK_H
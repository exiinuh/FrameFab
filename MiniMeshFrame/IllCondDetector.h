#pragma once

#ifndef ILLCONDDECTOR_H
#define ILLCONDDECTOR_H

#include "assert.h"

#include "f2c.h"
#include "clapack.h"
#include <Eigen/Sparse>

class IllCondDetector{
public:
	typedef Eigen::SparseMatrix<double> EigenSp;

public:
	IllCondDetector(){};
	IllCondDetector(EigenSp *K);
	~IllCondDetector(){};

public:
	// I\O
	void SetParm(int _ns, int _nl, int _condthres, int _gap)
	{ 
		assert(_ns > 0 || _nl > 0 || _condthres > 1 || _gap > 0); 
		ns_ = _ns;
		nl_ = _nl;
		cond_thres_ = _condthres;
		gap_ = _gap;
	}

private:
	int ns_;			// ns_ : The number of smallest eigenpairs
	int nl_;			// nl_ : The number of largest  eigenpairs
	int cond_thres_;	// cond_thres_ : The condition number threshold for triggering analysis
	int gap_;			// gap_ : The order of the gap between a cluster of smallest eigenvalues
						// and the next largest eigen values
};

#endif
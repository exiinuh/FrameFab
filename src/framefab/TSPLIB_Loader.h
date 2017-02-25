// this loader class is specified for TSP loader to dataset : 
//	http://www.iwr.uni-heidelberg.de/groups/comopt/software/TSPLIB95/
// Relative document of TSP MTZ formulation can be found at :
//  https://lost-contact.mit.edu/afs/md.kth.se/pkg/mosek/4/tools/doc/html/tools/node15.html

#ifndef TSP_LOADER_H
#define TSP_LOADER_H

#include "iostream"
#include <iostream>
#include <fstream>
#include "framefab\Statistics.h"

class TSPLIB_Loader
{
public:
	typedef Eigen::SparseMatrix<double> SpMat;

public:
	TSPLIB_Loader(){ ; }
	virtual ~TSPLIB_Loader(){ ; }

public:
	virtual bool loadFromFile(char *filename, int &N, SpMat *CostMatrix);		// N : number of nodes in graph 
};


#endif

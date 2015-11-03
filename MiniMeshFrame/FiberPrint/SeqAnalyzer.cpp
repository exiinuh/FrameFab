#include "SeqAnalyzer.h"

#define IJ(i,j,n) (n*i + j)
#define MAXCUTROUNDS 2
#define MAXADDPERROUND 2
SeqAnalyzer::SeqAnalyzer()
{
}


SeqAnalyzer::SeqAnalyzer(GraphCut *ptr_graphcut)
{
	ptr_graphcut_ = ptr_graphcut;
}

SeqAnalyzer::~SeqAnalyzer()
{
}

void SeqAnalyzer::DetectCollision()
{

}

void SeqAnalyzer::TSPSolver()
{
	// solves min c^t * x subject to lc < A * x < uc, x >= lb, x <= ub
	// Formulate TSP using "Integer Programming Formulation of Traveling Salesman Problems" http://dl.acm.org/citation.cfm?id=321046
	// i,e, using MTZ constraints to keep subtour clear.
}


void SeqAnalyzer::debug()
{
	//lp = LPFactory::make(static_cast<LPFactory::LPType>(1));

	LPMosek *lpm = new LPMosek();

	// test lp with official example
	lpm->test();
	
}
#pragma once

#include "SeqAnalyzer.h"
#include "GraphCut.h"
#include "Collision\Collision.h"

#include "TSPSolver.h"
#include "TSPLIB_Loader.h"

class SeqAnalyzer
{
	typedef Eigen::MatrixXd MX;
	typedef Eigen::VectorXd VX;

public:
	SeqAnalyzer();
	SeqAnalyzer(GraphCut *ptr_graphcut);
	~SeqAnalyzer();

public:
	void		LayerPrint();
	void		Debug();

private:
	GraphCut		*ptr_graphcut_;
	Collision		*ptr_collision_;

	double			alpha_;					
	double			beta_;
	double			gamma_;
};


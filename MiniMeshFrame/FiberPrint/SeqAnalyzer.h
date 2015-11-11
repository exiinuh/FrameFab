#pragma once

#include "SeqAnalyzer.h"
#include "GraphCut.h"
#include "Collision\Collision.h"
#include "TSPSolver.h"


class SeqAnalyzer
{
	typedef Eigen::MatrixXd MX;
	typedef Eigen::VectorXd VX;

public:
	SeqAnalyzer();
	SeqAnalyzer(GraphCut *ptr_graphcut);
	~SeqAnalyzer();

public:
	void				LayerPrint();

	vector<int>			*GetQueue()			{ return queue_; }
	vector<vector<int>>	*GetRangeState()	{ return ptr_collision_->GetRangeState(); }
	vector<Bulk*>		*GetBulk()			{ return ptr_collision_->GetBulk(); }

private:
	GraphCut		*ptr_graphcut_;
	Collision		*ptr_collision_;

	double			alpha_;					
	double			beta_;
	double			gamma_;

	vector<int>		*queue_;
};


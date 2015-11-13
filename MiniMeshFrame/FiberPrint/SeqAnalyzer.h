#pragma once

#include "SeqAnalyzer.h"
#include "GraphCut.h"
#include "Collision\Collision.h"
#include "TSP\TSPSolver.h"


enum Orientation
{
	SEQUENCE,
	REVERSE,
};


class SeqAnalyzer
{
	typedef Eigen::MatrixXd MX;
	typedef Eigen::VectorXd VX;

public:
	SeqAnalyzer();
	SeqAnalyzer(GraphCut *ptr_graphcut);
	~SeqAnalyzer();

public:
	void			LayerPrint();

	void			SetStartEdge(int id);
	void			GenerateQueue();

	void			ChangeOrientation();
	Orientation		GetOrientation();

	vector<int>			*GetQueue()			{ return queue_; }
	vector<vector<int>>	*GetRangeState()	{ return ptr_collision_->GetRangeState(); }
	vector<Bulk*>		*GetBulk()			{ return ptr_collision_->GetBulk(); }

private:
	GraphCut		*ptr_graphcut_;
	Collision		*ptr_collision_;

	VX				tsp_x_;

	double			alpha_;					
	double			beta_;
	double			gamma_;

	int				start_edge_;
	Orientation		orientation_;
	vector<int>		*queue_;
};


/*
* ==========================================================================
*
*       class: SequenceAnalyzer
*
*    Description:  perform tool path searching algorithm to generate
*				   a collision-free, structurally stable 
*
*	 Version:  1.0
*	 Created:  Oct/20/2015
*    Update :  Dec/08/2015
*
*	 Author:   Xin Hu, Guoxian Song, Yijiang Huang
*	 Company:  GCL@USTC
*	 Note:
*			   Ver1.0: Backtracking Greedy Approach:
*					   At every decision state, a trail solution is performed,
*				unvisited current layer edges that are connected to already printed
*				structure and calculate their adjacency,collision and stiffness weight.
*				The total printing cost is weighted sum of the three: wp_ * P + wl_ * L + ws_ * S
*				P: adjacency cost
*
*				L: collision cost	- range(dual_i, dual_j) is the prohibited angle range for printing edge i
*				because of the existence of edge j. Larger prohibited angle means that edge i is difficult to print
*				due to edge j's blocking, thus we give it larger cost to discourage greedy approach from choosing it.
*
*				S: stiffness cost
* ==========================================================================
*/
#pragma once
#include <cmath>

#include "SeqAnalyzer.h"
#include "GraphCut.h"
#include "Collision\Collision.h"

typedef struct Set
{
	double min;
	double max;
};

enum Orientation
{
	SEQUENCE,
	REVERSE,
};

typedef struct QueueInfo
{
	int		layer_;
	int		layer_id_;
	int		dual_id_;
};

class SeqAnalyzer
{
public:
	typedef Eigen::MatrixXd MX;
	typedef Eigen::Matrix3d M3;
	typedef Eigen::VectorXd VX;
	typedef Eigen::Vector3d V3;

public:
	SeqAnalyzer();
	SeqAnalyzer(GraphCut *ptr_graphcut);
	SeqAnalyzer(GraphCut *ptr_graphcut, FiberPrintPARM *ptr_parm);
	~SeqAnalyzer();

public:
	bool			LayerPrint();
	
	bool			GenerateSeq(int l, int h, int t);

	vector<QueueInfo>		*GetQueue()			{ return layer_queue_; }
	vector<vector<int>>		*GetRangeState()	{ return ptr_collision_->GetRangeState(); }
	vector<BaseBulk*>		*GetBulk()			{ return ptr_collision_->GetBulk(); }
	ExtruderCone			GetExtru(int i)		{ return (extrulist_)[i]; }

	void			Print(Set *a);
	void			Print(vector<Set*> *a);
	void			Debug();

private:
	GraphCut		*ptr_graphcut_;
	Collision		*ptr_collision_;
	DualGraph		*ptr_subgraph_;

	double			alpha_;					
	double			beta_;
	double			gamma_;
	double			stiff_tol_;
	double			Wl_;
	double			Wp_;
	double			height_differ_;

	vector<vector<int>>		layers_;			// store dual_node's id for each layers
	vector<QueueInfo>		*layer_queue_;

	ExtruderCone			extruder_;
	vector<point>			base_;
	vector<double>			anglelist_;
	vector<ExtruderCone>	extrulist_;
};


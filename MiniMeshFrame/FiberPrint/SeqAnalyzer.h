#pragma once

#include <cmath>

#include "SeqAnalyzer.h"
#include "GraphCut.h"
#include "Collision\Collision.h"
//#include "TSP\TSPSolver.h"

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
	void			LayerPrint();

	vector<QueueInfo>		*GetQueue()			{ return layer_queue_; }
	vector<vector<int>>		*GetRangeState()	{ return ptr_collision_->GetRangeState(); }
	vector<BaseBulk*>		*GetBulk()			{ return ptr_collision_->GetBulk(); }

	ExtruderCone	GetExtru(int i){ return (extrulist_)[i]; }

	vector<double>  AngleList(vector<QueueInfo>*layer_queue);
    double			Divergence( Range *r);

	// @PARM : start_id is for vector
	double			AngleValue(int start_id, vector<QueueInfo>*layer_queue);

	void			Print(Set *a);
	void			Print(vector<Set*> *a);

	// AngleDecide : find the suiable angle in those set
	double			AngleDecide(vector<Set*> *temp);

	void			Debug();

private:
	GraphCut		*ptr_graphcut_;
	Collision		*ptr_collision_;

	double			alpha_;					
	double			beta_;
	double			gamma_;
	double			stiff_tol_;

	int				start_edge_;

	vector<vector<int>>		layers_;
	vector<QueueInfo>		*layer_queue_;

	ExtruderCone			extruder_;
	vector<point>			base_;
	vector<double>			anglelist_;
	vector<ExtruderCone>	extrulist_;
};


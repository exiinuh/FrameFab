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
*    Update :  Dec/09/2015
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
* ==========================================================================
*/
#pragma once
#include <cmath>

#include "ADMMCut.h"
#include "NormalCut.h"
#include "Collision\Collision.h"
#include "Collision\ResolveAngle.h"

typedef struct Set
{
	double min;
	double max;
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
	SeqAnalyzer(GraphCut *ptr_graphcut, FiberPrintPARM *ptr_parm, char *path);
	~SeqAnalyzer();

public:
	virtual bool	LayerPrint();

	void			GetQueue(vector<int> &layer_queue);

	Vec3f			GetNormal(int i)	{ return extruder_list_[i].Normal(); }
	ExtruderCone	GetExtru(int i)		{ return (extruder_list_)[i]; }
	int				GetSupport()		{ return support_; }
	double			GetWave(int id)		{ return wave_[id]; }

public:
	DualGraph		*ptr_dualgraph_;
	DualGraph		*ptr_subgraph_;
	WireFrame		*ptr_frame_;
	Collision		*ptr_collision_;
	char			*path_;

	vector<vector<int>>		layers_;			// store dual_node's id for each layers
	vector<QueueInfo>		layer_queue_;
	vector<lld>				angle_state_;

	/* Printing Orientation Related Data */
	int				support_;
	bool			extru_;

	vector<ExtruderCone>	extruder_list_;
	vector<double>			wave_;				// wave_: orientation range data for each printing edge, 
												// index computed by seq analyzer, output data
	bool			debug_;
	bool			fileout_;
};


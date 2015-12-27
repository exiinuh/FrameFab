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

#include "SeqAnalyzer.h"
#include "GraphCut.h"
#include "Collision\Collision.h"
#include "Collision\ResolveAngle.h"

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
	SeqAnalyzer(GraphCut *ptr_graphcut, FiberPrintPARM *ptr_parm, char *path);
	~SeqAnalyzer();

public:
	bool			LayerPrint();
	bool			GenerateSeq(int l, int h, int t);
	double			GenerateCost(int l, int j);

	void			GetQueue(vector<int> &layer_queue);
	void			WriteLayerQueue();


	//vector<vector<int>>		*GetRangeState()	{ return ptr_collision_->GetRangeState(); }
	//vector<BaseBulk*>		*GetBulk()			{ return ptr_collision_->GetBulk(); }
	Vec3f					GetNormal(int i)	{ return extruder_list_[i].Normal(); }
	ExtruderCone			GetExtru(int i)		{ return (extruder_list_)[i]; }
	int						GetSupport()		{ return support_; }

	bool     extru_;
	double					GetWave(int id)		{ return wave_[id]; }

	

	///* Feasible Orientation Generation 		Nov/09/2015*/	
	///* Collision Ver2.0, angle list data interface */
	//vector<GeoV3>		AngleList(vector<QueueInfo> *layer_queue);
	///* Collision Ver2.0, angle decision interface  
	//*  
	//*/
	//GeoV3				AngleDec(int id);

	///* IsColVec - Judge if the target vector in the prohibited range*/
	//bool				IsColVec(GeoV3 start, GeoV3 end, GeoV3 target);

	///* Misc function */
	//void			Print(Set *a);
	//void			Print(vector<Set*> *a);
	//void			Debug();

public:
	GraphCut		*ptr_graphcut_;
	DualGraph		*ptr_subgraph_;
	char			*path_;

	void AngleDec();
private:
	double		gamma_;							// gamma_	 : amplifier factor for adjacency cost
	double		Dt_tol_;						// Dt_tol   : tolerance of offset in stiffness
	double		Dr_tol_;						// Dr_tol   : tolerance of rotation in stiffness
	double		Wl_;							// Wl_		 : tradeoff weight for printing cost
	double		Wp_;							// Wp_		 : tradeoff weight for printing cost

	vector<vector<int>>		layers_;			// store dual_node's id for each layers
	vector<QueueInfo>		layer_queue_;

	/* Printing Orientation Related Data */
	int			support_;
	point		start_;
	point		end_;

	ExtruderCone			extruder_;
	vector<double>			wave_;				// wave_: orientation range data for each printing edge, 
												// index computed by seq analyzer, output data

	vector<ExtruderCone>	extruder_list_;
	vector<GeoV3>			angle_list_;
	gte::Plane3<float>		table_;

	bool					debug_;





};


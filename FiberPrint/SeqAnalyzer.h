/*
* ==========================================================================
*
*		class:	SequenceAnalyzer
*
* Description:  perform tool path searching algorithm to generate
*				a collision-free, structurally stable path.
*
*	  Version:  1.2
*	  Created:  Oct/20/2015
*     Update :  Mar/30/2016
*
*	   Author:  Xin Hu, Guoxian Song, Yijiang Huang
*	  Company:  GCL@USTC
*
*	Successor:	FFAnalyzer - FrameFab sequence analyzer
*				BFAnalyzer - Brute Force sequence analyzer
*
* ==========================================================================
*/
#pragma once
#include <cmath>

#include "ADMMCut.h"
#include "NormalCut.h"
#include "Collision\QuadricCollision.h"
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
	SeqAnalyzer(GraphCut *ptr_graphcut, FiberPrintPARM *ptr_parm, char *ptr_path);
	~SeqAnalyzer();

public:
	virtual bool	SeqPrint();

public:
	void			GetPrintOrder();
	void			InputPrintOrder(vector<int> &print_queue);
	void			OutputPrintOrder(vector<int> &print_queue);

protected:
	void			UpdateStructure(WF_edge *e);
	void			RecoverStructure(WF_edge *e);

	void			UpdateStateMap(int dual_i, vector<vector<lld>> &state_map);
	void			RecoverStateMap(int dual_i, vector<vector<lld>> &state_map);

	bool			TestifyStiffness();

public:
	DualGraph			*ptr_dualgraph_;
	WireFrame			*ptr_frame_;

protected:
	QuadricCollision	*ptr_collision_;
	char				*ptr_path_;

	/* output */
	vector<int>			print_order_; 

	/* maintaining for sequence */
	DualGraph			*ptr_subgraph_;
	VX					D0_;
	vector<QueueInfo>	print_queue_;
	vector<vector<lld>> angle_state_;
	vector<vector<lld>*>colli_map_;
	vector<vector<int>>	layers_;					// store dual_node's id for each layers

	/* parameters */
	FiberPrintPARM		*ptr_parm_;
	double				gamma_;						// gamma_	: amplifier factor for adjacency cost
	double				Dt_tol_;					// Dt_tol	: tolerance of offset in stiffness
	double				Dr_tol_;					// Dr_tol   : tolerance of rotation in stiffness
	double				Wl_;						// Wl_		: tradeoff weight for printing cost
	double				Wp_;						// Wp_		: tradeoff weight for printing cost
	double				Wa_;						// Wa_		: tradeoff weight for printing cost

	bool				debug_;
	bool				fileout_;
};


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
	SeqAnalyzer(
		DualGraph			*ptr_dualgraph,
		QuadricCollision	*ptr_collision,
		Stiffness			*ptr_stiffness,
		FiberPrintPARM		*ptr_parm,
		char				*ptr_path
		);
	virtual ~SeqAnalyzer();

public:
	virtual bool	SeqPrint();
	virtual void	PrintOutTimer();
	virtual void	WriteRenderPath(int min_layer, int max_layer, char *ptr_path);

public:
	void			Init();

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
	WireFrame			*ptr_frame_;
	DualGraph			*ptr_dualgraph_;
	Stiffness			*ptr_stiffness_;
	QuadricCollision	*ptr_collision_;
	char				*ptr_path_;

protected:
	/* output */
	vector<int>			print_order_; 

	/* maintaining for sequence */
	DualGraph			*ptr_wholegraph_;
	VX					D0_;
	vector<QueueInfo>	print_queue_;
	vector<vector<lld>> angle_state_;
	vector<vector<int>>	layers_;					// store dual_node's id for each layers

	/* parameters */
	double				gamma_;						// gamma_	: amplifier factor for adjacency cost
	double				Dt_tol_;					// Dt_tol	: tolerance of offset in stiffness
	double				Dr_tol_;					// Dr_tol   : tolerance of rotation in stiffness
	double				Wp_;						// Wp_		: stablity weight for printing cost
	double				Wa_;						// Wa_		: adjacent weight for printing cost
	double				Wi_;						// Wl_		: influence weight for printing cost

	bool				debug_;
	bool				fileout_;

	Timer				upd_struct_;
	Timer				rec_struct_;
	Timer				upd_map_;
	Timer				upd_map_collision_;
	Timer				rec_map_;
	Timer				test_stiff_;
};


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
*     Update :  Mar/25/2015
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
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 3cef735a95cf86715af239aaec2119119cb169f4
	virtual bool	SeqPrint();

public:
	void			GetQueue(vector<int> &layer_queue);
<<<<<<< HEAD
=======
	bool			LayerPrint();
	bool			GenerateSeq(int l, int h, int t);
	double			GenerateCost(int l, int j);
	void			DetectAngle();

	void			GetQueue(vector<int> &layer_queue);
	void			WriteLayerQueue();
>>>>>>> 2c719846f6006b0658c93a4bf28bf6ac0236a416
=======
>>>>>>> 3cef735a95cf86715af239aaec2119119cb169f4

	Vec3f			GetNormal(int i)	{ return extruder_list_[i].Normal(); }
	ExtruderCone	GetExtru(int i)		{ return (extruder_list_)[i]; }
	int				GetSupport()		{ return support_; }
	double			GetWave(int id)		{ return wave_[id]; }

protected:
	void			UpdateStateMap(int dual_i, vector<vector<lld>> &state_map);
	void			RecoverStateMap(int dual_i, vector<vector<lld>> &state_map);

	bool			TestifyStiffness();

public:
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 3cef735a95cf86715af239aaec2119119cb169f4
	DualGraph			*ptr_dualgraph_;
	WireFrame			*ptr_frame_;

protected:
	DualGraph			*ptr_subgraph_;
	QuadricCollision	*ptr_collision_;
	char				*ptr_path_;

	vector<QueueInfo>	print_queue_;
	vector<vector<lld>> angle_state_;
	vector<vector<int>>	layers_;				// store dual_node's id for each layers

	FiberPrintPARM		*ptr_parm_;
	double				gamma_;						// gamma_	: amplifier factor for adjacency cost
	double				Dt_tol_;					// Dt_tol	: tolerance of offset in stiffness
	double				Dr_tol_;					// Dr_tol   : tolerance of rotation in stiffness
	double				Wl_;						// Wl_		: tradeoff weight for printing cost
	double				Wp_;						// Wp_		: tradeoff weight for printing cost
	double				Wi_;						// Wi_		: tradeoff weight for printing cost

<<<<<<< HEAD
=======
	GraphCut		*ptr_graphcut_;
	DualGraph		*ptr_subgraph_;
	char			*path_;

private:
	double			gamma_;						// gamma_	: amplifier factor for adjacency cost
	double			Dt_tol_;					// Dt_tol	: tolerance of offset in stiffness
	double			Dr_tol_;					// Dr_tol   : tolerance of rotation in stiffness
	double			Wl_;						// Wl_		: tradeoff weight for printing cost
	double			Wp_;						// Wp_		: tradeoff weight for printing cost
	double			Wi_;						// Wi_		: tradeoff weight for printing cost

	vector<vector<int>>		layers_;			// store dual_node's id for each layers
	vector<QueueInfo>		layer_queue_;

	Collision		*ptr_collision_;
	vector<lld>		angle_state_;

	double			min_z_;
	double			max_z_;
>>>>>>> 2c719846f6006b0658c93a4bf28bf6ac0236a416
=======
>>>>>>> 3cef735a95cf86715af239aaec2119119cb169f4
	/* Printing Orientation Related Data */
	int					support_;
	bool				extru_;

	vector<ExtruderCone>	extruder_list_;
	vector<double>			wave_;				// wave_: orientation range data for each printing edge, 
												// index computed by seq analyzer, output data
	bool				debug_;
	bool				fileout_;
};


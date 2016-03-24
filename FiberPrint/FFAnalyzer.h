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


class FFAnalyzer : public SeqAnalyzer
{
public:
	typedef Eigen::MatrixXd MX;
	typedef Eigen::Matrix3d M3;
	typedef Eigen::VectorXd VX;
	typedef Eigen::Vector3d V3;

public:
	FFAnalyzer();
	FFAnalyzer(GraphCut *ptr_graphcut);
	FFAnalyzer(GraphCut *ptr_graphcut, FiberPrintPARM *ptr_parm, char *path);
	~FFAnalyzer();

public:
	bool			LayerPrint();
	bool			GenerateSeq(int l, int h, int t);
	double			GenerateCost(int l, int j);
	void			DetectBulk();
	bool			GenerateSeq(int h, int t);

	void			WriteLayerQueue();
	void			WritePathRender();

private:
	double			gamma_;						// gamma_	: amplifier factor for adjacency cost
	double			Dt_tol_;					// Dt_tol	: tolerance of offset in stiffness
	double			Dr_tol_;					// Dr_tol   : tolerance of rotation in stiffness
	double			Wl_;						// Wl_		: tradeoff weight for printing cost
	double			Wp_;						// Wp_		: tradeoff weight for printing cost
	double			Wi_;						// Wi_		: tradeoff weight for printing cost

	double			min_z_;
	double			max_z_;

};


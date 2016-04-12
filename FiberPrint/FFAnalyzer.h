/*
* ==========================================================================
*
*       class:	FFAnalyzer
*
* Description:  perform the algorithm in FrameFab to generate a collision-free,
*				structurally-stable path.
*
*	  Version:  1.1
*	  Created:  Mar/23/2016
*     Update :  Mar/25/2016
*
*	   Author:  Xin Hu, Guoxian Song, Yijiang Huang
*	  Company:  GCL@USTC
*	     Note:	Backtracking Greedy Approach:
*				At every decision state, a trail solution is performed,
*				unvisited current layer edges that are connected to already printed
*				structure and calculate their adjacency,collision and stiffness weight.
*				
*				The total printing cost is weighted sum of the three: 
*					
*					wp_ * P + wl_ * L + ws_ * S
*				
*					P: adjacency cost
*					L: collision cost	
*
* ==========================================================================
*/

#pragma once
#include <cmath>
#include <cstring>

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
	FFAnalyzer(WireFrame *ptr_frame, char *ptr_path)
		:SeqAnalyzer(ptr_frame, ptr_path){}
	FFAnalyzer(GraphCut *ptr_graphcut, FiberPrintPARM *ptr_parm, char *ptr_path)
		:SeqAnalyzer(ptr_graphcut, ptr_parm, ptr_path){}
	~FFAnalyzer();

public:
	bool			SeqPrint();

private:
	bool			GenerateSeq(int l, int h, int t);
	double			GenerateCost(int l, int j, WF_edge *ei);

public:
	void			PrintOutTimer();
	void			WriteRenderPath(int min_layer, int max_layer, char *ptr_path);

private:
	double			min_z_;
	double			max_z_;

};


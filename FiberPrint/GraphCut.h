/*
* ==========================================================================
*
*		class:	GraphCut
*
* Description:	Cut input model into layers by different means.
*
*	  Version:  1.1
*	  Created:  Oct/10/2015
*     Updated:  Mar/25/2016
*
*	   Author:  Xin Hu, Yijiang Huang, Guoxian Song
*	  Company:  GCL@USTC
*
*	Successor:	ADMMCut - FrameFab ADMM cut
*				NormalCut - cut layers by sweeping from bottom to top.
*				NoneCut - no layers
* ==========================================================================
*/

#pragma once

#include <iostream>

#include "WireFrame\WireFrame.h"
#include "Stiffness\Stiffness.h"
#include "Collision\QuadricCollision.h"
#include "FiberPrintPARM.h"

using namespace std;


class GraphCut
{
public:
	GraphCut();
	GraphCut(WireFrame *ptr_frame, char *ptr_path);
	virtual ~GraphCut();

public:
	virtual void	MakeLayers();	
	virtual void	PrintOutTimer();

public:
	vector<DualVertex*>		*GetDualVertList()		{ return ptr_dualgraph_->GetVertList(); }
	vector<DualEdge*>		*GetDualEdgeList()		{ return ptr_dualgraph_->GetEdgeList(); }
	vector<DualFace*>		*GetDualFaceList()		{ return ptr_dualgraph_->GetFaceList(); }

public:
	WireFrame			*ptr_frame_;
	DualGraph			*ptr_dualgraph_;
	Stiffness			*ptr_stiffness_;	
	QuadricCollision	*ptr_collision_;
	char				*ptr_path_;


	vector<int>		cutting_edge_;
	vector<int>		vert_depth_;
	vector<int>		edge_depth_;
	int				max_vert_dep_;


protected:
	/* for debuging */
	bool				debug_;

};


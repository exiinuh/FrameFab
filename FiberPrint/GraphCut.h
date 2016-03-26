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
#include "FiberPrintPARM.h"

using namespace std;


class GraphCut
{
public:
	GraphCut();
	~GraphCut();

public:
	virtual void	MakeLayers();						

public:
	vector<DualVertex*>		*GetDualVertList()		{ return ptr_dualgraph_->GetVertList(); }
	vector<DualEdge*>		*GetDualEdgeList()		{ return ptr_dualgraph_->GetEdgeList(); }
	vector<DualFace*>		*GetDualFaceList()		{ return ptr_dualgraph_->GetFaceList(); }

public:
//private:
	WireFrame		*ptr_frame_;
	DualGraph		*ptr_dualgraph_;
	Stiffness		*ptr_stiff_;	// Store 3*3 stiffness and caluculate weighted global stiffness matrix
	char			*path_;

	bool			debug_;
};


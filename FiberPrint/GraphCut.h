/*
* ==========================================================================
*
*       class: GraphCut
*
*    Description:  This file is a part of implementation fo paper "WirePrint : a fast&stable way to fabricate wireframe"
*				   The GraphCut submodule takes charge of dividing the wireframe into several structually-stable sections, 
*				   scaling the problem down, enabling further tool path searching part numerically tractable.
*
*	 Version:  1.0
*	 Created:  Oct/10/2015
*    Updated:  Nov/02/2015
*
*	 Author:   Xin Hu, Yijiang Huang, Guoxian Song
*	 Company:  GCL@USTC
*
*    WARNING:  DO NOT insert node and edge after you dualize the graph, 
*    as we append all project fixed point at the end of dual face.
*    Further inserting will cause stiffness matrix and force creation error.
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
};


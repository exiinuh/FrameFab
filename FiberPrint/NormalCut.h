/*
* ==========================================================================
*
*		class:	NormalCut
*
* Description:	Cut layers by sweeping from bottom to top.
*
*	  Version:  1.0
*	  Created:  Mar/25/2016
*     Updated:  Mar/26/2016
*
*	   Author:  Xin Hu, Yijiang Huang, Guoxian Song
*	  Company:  GCL@USTC
*
* ==========================================================================
*/

#pragma once

#include <map>

#include "GraphCut.h"

using namespace std;


class NormalCut : public GraphCut
{
public:
	NormalCut();
	NormalCut(WireFrame *ptr_frame, FiberPrintPARM *ptr_parm, char *path);
	~NormalCut();

public:
	void	MakeLayers();

private:
	multimap<double, WF_edge*>	sweep_queue_;
};


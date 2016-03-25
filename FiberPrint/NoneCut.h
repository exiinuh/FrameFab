/*
* ==========================================================================
*
*		class:	GraphCut
*
* Description:	No layer-cut.
*
*	  Version:  1.0
*	  Created:  Mar/25/2016
*     Updated:  Mar/25/2016
*
*	   Author:  Xin Hu, Yijiang Huang, Guoxian Song
*	  Company:  GCL@USTC
*
* ==========================================================================
*/

#pragma once

#include "GraphCut.h"

class NoneCut : public GraphCut
{
public:
	NoneCut();
	NoneCut(WireFrame *ptr_frame, FiberPrintPARM *ptr_parm, char *path);
	~NoneCut();
};


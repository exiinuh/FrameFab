/*
* ==========================================================================
*
*       class:	BFAnalyzer
*
* Description:  perform Brute Force algorithm to generate a collision-free, 
*				structurally-stable path.
*
*	  Version:  1.0
*	  Created:  Mar/23/2016
*     Update :  Mar/25/2016
*
*	   Author:  Xin Hu, Guoxian Song, Yijiang Huang
*	  Company:  GCL@USTC
*	     Note:	Generate every possible tool path and remove paths that are
*				not collision-free or structurally-stable.
*
* ==========================================================================
*/

#pragma once

#include "SeqAnalyzer.h"

class BFAnalyzer : public SeqAnalyzer
{
public:
	BFAnalyzer();
	BFAnalyzer(
		DualGraph			*ptr_dualgraph,
		QuadricCollision	*ptr_collision,
		Stiffness			*ptr_stiffness,
		FiberPrintPARM		*ptr_parm,
		char				*ptr_path
		)
		:SeqAnalyzer(ptr_dualgraph, ptr_collision, ptr_stiffness,
		ptr_parm, ptr_path){}
	~BFAnalyzer();

public:
	bool	SeqPrint();

private:
	bool	GenerateSeq(int h, int t);
	void	PrintOutQueue(int N);	
};


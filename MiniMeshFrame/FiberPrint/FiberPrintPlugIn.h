#pragma once

#ifndef FIBERPRINTPLUGIN_H
#define FIBERPRINTPLUGIN_H

#include <vector>
#include "SeqAnalyzer.h"
//#include "Stiffness.h"

#include "StiffnessIO.h"
#include "GraphCut.h"

class FiberPrintPlugIn
{
public:
	FiberPrintPlugIn();
	FiberPrintPlugIn(WireFrame *ptr_frame);
	FiberPrintPlugIn(WireFrame *ptr_frame, FiberPrintPARM *ptr_parm);
	~FiberPrintPlugIn();

public:
	void			Print();

	void			SetStartEdge(int id);
	void			ChangeOrientation();

	vector<DualVertex*>	*GetDualVertList()	{ return ptr_graphcut_->GetDualVertList(); }
	vector<int>			*GetLabel()			{ return ptr_graphcut_->GetLabel(); }
	vector<int>			*GetCut()			{ return ptr_graphcut_->GetCut(); }

	vector<int>			*GetQueue()			{ return ptr_seqanalyzer_->GetQueue(); }
	vector<vector<int>>	*GetRangeState()	{ return ptr_seqanalyzer_->GetRangeState(); }
	vector<Bulk*>		*GetBulk()			{ return ptr_seqanalyzer_->GetBulk(); }

	void				Debug();		// return value: edge index in mesh, for cut rendering

public:
	WireFrame		*ptr_frame_;
	GraphCut		*ptr_graphcut_;
	SeqAnalyzer		*ptr_seqanalyzer_;
};

#endif // FIBERPRINTPLUGIN_H

#pragma once

#ifndef FIBERPRINTPLUGIN_H
#define FIBERPRINTPLUGIN_H

#include <vector>
#include "ADMMCut.h"
#include "NormalCut.h"
#include "FFAnalyzer.h"
#include "BFAnalyzer.h"
#include "ProcessAnalyzer.h"


class FiberPrintPlugIn
{
public:
	FiberPrintPlugIn();
	FiberPrintPlugIn(WireFrame *ptr_frame);
	FiberPrintPlugIn(WireFrame *ptr_frame, FiberPrintPARM *ptr_parm,
						char *path);
	~FiberPrintPlugIn();

public:
	void			Print();

	vector<DualVertex*>	*GetDualVertList()				{ return ptr_graphcut_->GetDualVertList(); }
	void				GetQueue(vector<int> &queue)	{ ptr_seqanalyzer_->GetQueue(queue); }
	//vector<BaseBulk*>	*GetBulk()			{ return ptr_seqanalyzer_->GetBulk(); }

	void				Debug();		// return value: edge index in mesh, for cut rendering

public:
	WireFrame		*ptr_frame_;
	GraphCut		*ptr_graphcut_;
	SeqAnalyzer		*ptr_seqanalyzer_;
	ProcessAnalyzer	*ptr_procanalyzer_;
};

#endif // FIBERPRINTPLUGIN_H

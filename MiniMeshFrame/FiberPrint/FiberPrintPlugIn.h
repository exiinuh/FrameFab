#pragma once

#ifndef FIBERPRINTPLUGIN_H
#define FIBERPRINTPLUGIN_H

#include <vector>
#include "GraphCut.h"
#include "SeqAnalyzer.h"
#include "Stiffness.h"


class FiberPrintPlugIn
{
public:
	FiberPrintPlugIn();
	FiberPrintPlugIn(WireFrame *ptr_frame);
	~FiberPrintPlugIn();

public:
	void				Print();
	vector<DualVertex*>	*GetDualVertList();
	VectorXi			*GetLabel();
	void				Debug();		// return value: edge index in mesh, for cut rendering

private:
	WireFrame		*ptr_frame_;
	GraphCut		*ptr_graphcut_;
	SeqAnalyzer		*ptr_seqanalyzer_;
};

#endif // FIBERPRINTPLUGIN_H

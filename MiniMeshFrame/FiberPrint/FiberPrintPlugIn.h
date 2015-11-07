#pragma once

#ifndef FIBERPRINTPLUGIN_H
#define FIBERPRINTPLUGIN_H

#include <vector>
#include "GraphCut.h"
#include "SeqAnalyzer.h"
#include "Stiffness.h"

#include "TSPSolver.h"
#include "TSPLIB_Loader.h"

class FiberPrintPlugIn
{
public:
	FiberPrintPlugIn();
	FiberPrintPlugIn(WireFrame *ptr_frame);
	~FiberPrintPlugIn();

public:
	void				Print();
	vector<DualVertex*>	*GetDualVertexList();
	VectorXi			*GetLabel();
	void				Debug();		// return value: edge index in mesh, for cut rendering

private:
	WireFrame		*ptr_frame_;
	GraphCut		*ptr_graphcut_;
	SeqAnalyzer		*ptr_seqanalyzer_;
	Stiffness		*ptr_stiffness_;
};

#endif // FIBERPRINTPLUGIN_H

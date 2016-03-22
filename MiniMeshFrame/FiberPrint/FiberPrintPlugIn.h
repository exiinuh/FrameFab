#pragma once

#ifndef FIBERPRINTPLUGIN_H
#define FIBERPRINTPLUGIN_H

#include <stdio.h>

#include <vector>
#include "GraphCut.h"
#include "SeqAnalyzer.h"
#include "ProcessAnalyzer.h"

#include "DualGraph.h"
#include "Stiffness\Stiffness.h"
#include "Stiffness\StiffnessIO.h"
#include "Stiffness\IllCondDetector.h"

class FiberPrintPlugIn
{
public:
	typedef Eigen::MatrixXd MX;
	typedef Eigen::Matrix3d M3;
	typedef Eigen::VectorXd VX;
	typedef Eigen::Vector3d V3;

public:
	FiberPrintPlugIn();
	FiberPrintPlugIn(WireFrame *ptr_frame);
	FiberPrintPlugIn(WireFrame *ptr_frame, FiberPrintPARM *ptr_parm,
						char *path);
	~FiberPrintPlugIn();

public:
	void			Print();

	vector<DualVertex*>	*GetDualVertList()				{ return ptr_graphcut_->GetDualVertList(); }
	vector<int>			*GetCut()						{ return ptr_graphcut_->GetCut(); }
	void				GetQueue(vector<int> &queue)	{ ptr_seqanalyzer_->GetQueue(queue); }
	//vector<BaseBulk*>	*GetBulk()			{ return ptr_seqanalyzer_->GetBulk(); }

	void				GetDeformation();  // Use stiffness matrix to solve deformation
	void				Debug();		   // return value: edge index in mesh, for cut rendering

public:
	WireFrame		*ptr_frame_;
	GraphCut		*ptr_graphcut_;
	SeqAnalyzer		*ptr_seqanalyzer_;
	ProcessAnalyzer	*ptr_procanalyzer_;
	FiberPrintPARM  *ptr_parm_;
};

#endif // FIBERPRINTPLUGIN_H

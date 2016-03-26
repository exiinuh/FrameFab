#pragma once

#ifndef FIBERPRINTPLUGIN_H
#define FIBERPRINTPLUGIN_H

#include <stdio.h>

#include <vector>
#include "ADMMCut.h"
#include "NormalCut.h"
#include "NoneCut.h"
#include "FFAnalyzer.h"
#include "BFAnalyzer.h"
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
	void			FrameFabPrint();
	void			BruteForcePrint();
	void			SweepingPrint();

	vector<DualVertex*>*GetDualVertList()				{ return ptr_graphcut_->GetDualVertList(); }
	void				GetQueue(vector<int> &queue)	{ ptr_seqanalyzer_->GetQueue(queue); }
	//vector<BaseBulk*>	*GetBulk()			{ return ptr_seqanalyzer_->GetBulk(); }

	void				GetDeformation();  // Use stiffness matrix to solve deformation
	void				Debug();		   // return value: edge index in mesh, for cut rendering

public:
	WireFrame		*ptr_frame_;
	GraphCut		*ptr_graphcut_;
	SeqAnalyzer		*ptr_seqanalyzer_;
	ProcessAnalyzer	*ptr_procanalyzer_;
<<<<<<< HEAD

private:
	char			*ptr_path_;
	FiberPrintPARM	*ptr_parm_;
=======
	FiberPrintPARM  *ptr_parm_;
>>>>>>> 2c719846f6006b0658c93a4bf28bf6ac0236a416
};

#endif // FIBERPRINTPLUGIN_H

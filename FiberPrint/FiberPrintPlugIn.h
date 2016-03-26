#pragma once

#ifndef FIBERPRINTPLUGIN_H
#define FIBERPRINTPLUGIN_H

#include <vector>
#include "ADMMCut.h"
#include "NormalCut.h"
#include "NoneCut.h"
#include "FFAnalyzer.h"
#include "BFAnalyzer.h"
#include "ProcessAnalyzer.h"


class FiberPrintPlugIn
{
public:
public:
	typedef Eigen::MatrixXd MX;
	typedef Eigen::VectorXd VX;

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

	/* apply stiffness computation directly to the input frame shape */
	void			GetDeformation();

	vector<DualVertex*>*GetDualVertList()				{ return ptr_graphcut_->GetDualVertList(); }
	void				GetQueue(vector<int> &queue)	{ ptr_seqanalyzer_->GetQueue(queue); }
	//vector<BaseBulk*>	*GetBulk()			{ return ptr_seqanalyzer_->GetBulk(); }

	void				Debug();		// return value: edge index in mesh, for cut rendering

public:
	WireFrame		*ptr_frame_;
	GraphCut		*ptr_graphcut_;
	SeqAnalyzer		*ptr_seqanalyzer_;
	ProcessAnalyzer	*ptr_procanalyzer_;
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 3cef735a95cf86715af239aaec2119119cb169f4

private:
	char			*ptr_path_;
	FiberPrintPARM	*ptr_parm_;
<<<<<<< HEAD
=======
	FiberPrintPARM  *ptr_parm_;
>>>>>>> 2c719846f6006b0658c93a4bf28bf6ac0236a416
=======
>>>>>>> 3cef735a95cf86715af239aaec2119119cb169f4
};

#endif // FIBERPRINTPLUGIN_H

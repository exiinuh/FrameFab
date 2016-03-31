#pragma once

#ifndef FIBERPRINTPLUGIN_H
#define FIBERPRINTPLUGIN_H

#include <vector>
#include "ADMMCut.h"
#include "NormalCut.h"
#include "NoneCut.h"
#include "FFAnalyzer.h"
#include "BFAnalyzer.h"

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
	void			Init();

	/* Fiber printing */
	void			FrameFabPrint();
	void			BruteForcePrint();
	void			SweepingPrint();

	/* apply stiffness computation directly to the input frame shape */
	void			GetDeformation();

	int				ImportPrintOrder(char *fname);
	void			ExportPrintOrder(char *fname);

	void			InputPrintOrder(vector<int> &queue)		{ ptr_seqanalyzer_->InputPrintOrder(queue); }
	void			OutputPrintOrder(vector<int> &queue)	{ ptr_seqanalyzer_->OutputPrintOrder(queue); }

	void			Debug();		// return value: edge index in mesh, for cut rendering

public:
	WireFrame		*ptr_frame_;
	GraphCut		*ptr_graphcut_;
	SeqAnalyzer		*ptr_seqanalyzer_;

private:
	char			*ptr_path_;
	FiberPrintPARM	*ptr_parm_;
};

#endif // FIBERPRINTPLUGIN_H

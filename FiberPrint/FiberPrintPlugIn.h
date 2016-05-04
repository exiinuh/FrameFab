#pragma once

#ifndef FIBERPRINTPLUGIN_H
#define FIBERPRINTPLUGIN_H

#include <vector>
#include "ADMMCut.h"
#include "NormalCut.h"
#include "FFAnalyzer.h"
#include "BFAnalyzer.h"
#include "ProcAnalyzer.h"


class FiberPrintPlugIn
{
public:
public:
	typedef Eigen::MatrixXd MX;
	typedef Eigen::VectorXd VX;

public:
	FiberPrintPlugIn();
	FiberPrintPlugIn(WireFrame *ptr_frame, FiberPrintPARM *ptr_parm,
						char *path);
	~FiberPrintPlugIn();

public:
	void			Init();

	/* Fiber printing */
	void			FrameFabPrint();
	void			BruteForcePrint();
	void			SweepingPrint();
	void			OneLayerPrint();

	/* apply stiffness computation directly to the input frame shape */
	void			GetDeformation();

	int				ImportPrintOrder(char *fname);
	void			ExportPrintOrder(char *fname);

	void			InputPrintOrder(vector<int> &queue)			{ ptr_seqanalyzer_->InputPrintOrder(queue); }
	void			OutputPrintOrder(vector<WF_edge*> &queue)	{ ptr_seqanalyzer_->OutputPrintOrder(queue); }
	void			ExportRenderPath(int min_layer, int max_layer, char *ptr_path)
	{
		ptr_seqanalyzer_->WriteRenderPath(min_layer, max_layer, ptr_path);
	}

	void			PrintOutTimer();
	void			Debug();		// return value: edge index in mesh, for cut rendering

public:
	WireFrame			*ptr_frame_;
	DualGraph			*ptr_dualgraph_;
	QuadricCollision	*ptr_collision_;
	Stiffness			*ptr_stiffness_;

	GraphCut			*ptr_graphcut_;
	SeqAnalyzer			*ptr_seqanalyzer_;
	ProcAnalyzer		*ptr_procanalyzer_;

	char				*ptr_path_;
	FiberPrintPARM		*ptr_parm_;

private:
	Timer				fiber_print_;
};

#endif // FIBERPRINTPLUGIN_H

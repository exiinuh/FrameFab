#include "FiberPrintPlugIn.h"


FiberPrintPlugIn::FiberPrintPlugIn()
{
}


FiberPrintPlugIn::FiberPrintPlugIn(WireFrame *ptr_frame)
{
	ptr_frame_ = ptr_frame;
	ptr_graphcut_ = new GraphCut(ptr_frame);
	ptr_seqanalyzer_ = new SeqAnalyzer(ptr_graphcut_);
}


FiberPrintPlugIn::FiberPrintPlugIn(WireFrame *ptr_frame, 
	FiberPrintPARM *ptr_parm, char *path)
{
	ptr_frame_ = ptr_frame;
	ptr_graphcut_ = new GraphCut(ptr_frame, ptr_parm);
	ptr_seqanalyzer_ = new SeqAnalyzer(ptr_graphcut_, ptr_parm);
	ptr_procanalyzer_ = new ProcessAnalyzer(ptr_seqanalyzer_, path);
}


FiberPrintPlugIn::~FiberPrintPlugIn()
{
	delete ptr_graphcut_;
	ptr_graphcut_ = NULL;

	delete ptr_seqanalyzer_;
	ptr_seqanalyzer_ = NULL;
}


void FiberPrintPlugIn::Print()
{
	ptr_graphcut_->MakeLayers();

	while (!ptr_seqanalyzer_->LayerPrint())
	{
		ptr_frame_->RefineFrame();
	}

	ptr_procanalyzer_->ProcPrint();
}


void FiberPrintPlugIn::Debug()
{

}
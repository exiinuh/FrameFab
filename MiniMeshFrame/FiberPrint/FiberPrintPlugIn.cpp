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


FiberPrintPlugIn::FiberPrintPlugIn(WireFrame *ptr_frame, FiberPrintPARM *ptr_parm)
{
	ptr_frame_ = ptr_frame;
	ptr_graphcut_ = new GraphCut(ptr_frame, ptr_parm);
	ptr_seqanalyzer_ = new SeqAnalyzer(ptr_graphcut_);
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
	//ptr_seqanalyzer_->LayerPrint();
}


void FiberPrintPlugIn::SetStartEdge(int id)
{
	ptr_seqanalyzer_->SetStartEdge(id);
}


void FiberPrintPlugIn::ChangeOrientation()
{
	ptr_seqanalyzer_->ChangeOrientation();
}


void FiberPrintPlugIn::Debug()
{

}
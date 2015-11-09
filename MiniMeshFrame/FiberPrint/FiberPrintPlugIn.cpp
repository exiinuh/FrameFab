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


FiberPrintPlugIn::~FiberPrintPlugIn()
{
	delete ptr_graphcut_;
	ptr_graphcut_ = NULL;
}


void FiberPrintPlugIn::Print()
{
	ptr_graphcut_->MakeLayers();
	//ptr_seqanalyzer_->LayerPrint();
	//ptr_seqanalyzer_->Debug();
}


vector<DualVertex*> *FiberPrintPlugIn::GetDualVertList()
{
	return 	ptr_graphcut_->GetDualVertList();
}


vector<int> *FiberPrintPlugIn::GetLabel()
{
	return ptr_graphcut_->GetLabel();
}


vector<int> *FiberPrintPlugIn::GetCut()
{
	return ptr_graphcut_->GetCut();
}

void FiberPrintPlugIn::Debug()
{

}
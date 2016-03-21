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
	ptr_graphcut_ = new GraphCut(ptr_frame, ptr_parm, path);
	ptr_seqanalyzer_ = new SeqAnalyzer(ptr_graphcut_, ptr_parm, path);
	ptr_procanalyzer_ = new ProcessAnalyzer(ptr_seqanalyzer_, path);
	ptr_parm_ = ptr_parm;
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
	cout << "Graph Cut completed." << endl;
	getchar();

	//while (!ptr_seqanalyzer_->LayerPrint())
	//{
	//	cout << "Model not printable!" << endl;
	//	cout << "Press Enter to refine the mesh and continue..." << endl;
	//	getchar();
	//	ptr_frame_->RefineFrame();
	//}

	//ptr_procanalyzer_->ProcPrint();
}

void FiberPrintPlugIn::GetDeformation()
{
	DualGraph *ptr_dualgraph = new DualGraph(ptr_frame_);	
	Stiffness *ptr_stiff = new Stiffness(ptr_dualgraph, ptr_parm_);

	ptr_dualgraph->Dualization();
	ptr_stiff->Init();

	int Ns = ptr_dualgraph->SizeOfFreeFace();
	VX D(Ns);
	D.setZero();

	int Nd = ptr_dualgraph->SizeOfVertList();
	VX x(Nd);
	x.setOnes();

	ptr_stiff->CalculateD(D,x,1,1,0);
}

void FiberPrintPlugIn::Debug()
{
}
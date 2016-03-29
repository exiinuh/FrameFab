#include "FiberPrintPlugIn.h"


FiberPrintPlugIn::FiberPrintPlugIn()
{
}


FiberPrintPlugIn::FiberPrintPlugIn(WireFrame *ptr_frame)
{
	ptr_frame_ = ptr_frame;
}


FiberPrintPlugIn::FiberPrintPlugIn(WireFrame *ptr_frame, 
	FiberPrintPARM *ptr_parm, char *ptr_path)
{
	ptr_frame_ = ptr_frame;
	ptr_parm_ = ptr_parm;
	ptr_path_ = ptr_path;
}


FiberPrintPlugIn::~FiberPrintPlugIn()
{
	delete ptr_graphcut_;
	ptr_graphcut_ = NULL;

	delete ptr_seqanalyzer_;
	ptr_seqanalyzer_ = NULL;
}


void FiberPrintPlugIn::FrameFabPrint()
{
	ptr_graphcut_ = new ADMMCut(ptr_frame_, ptr_parm_, ptr_path_);
	ptr_seqanalyzer_ = new FFAnalyzer(ptr_graphcut_, ptr_parm_, ptr_path_);
	ptr_procanalyzer_ = new ProcessAnalyzer(ptr_seqanalyzer_, ptr_path_);

	//ptr_graphcut_->MakeLayers();
	cout << "Graph Cut completed." << endl;


	if (!ptr_seqanalyzer_->SeqPrint())
	{
		cout << "Model not printable!" << endl;
		getchar();

		return;
	}

	printf("FrameFab print done.\n");
	//ptr_procanalyzer_->ProcPrint();
	//ptr_seqanalyzer_->WritePathRender();

	//ptr_seqanalyzer_->BruteForcePrint();
}


void FiberPrintPlugIn::BruteForcePrint()
{
	ptr_graphcut_ = new NoneCut(ptr_frame_, ptr_parm_, ptr_path_);
	ptr_seqanalyzer_ = new BFAnalyzer(ptr_graphcut_, ptr_parm_, ptr_path_);

	if (!ptr_seqanalyzer_->SeqPrint())
	{
		cout << "Model not printable!" << endl;
		getchar();

		return;
	}
	printf("BruteForce print done.\n");
}


void FiberPrintPlugIn::SweepingPrint()
{
	ptr_graphcut_ = new NormalCut(ptr_frame_, ptr_parm_, ptr_path_);
	ptr_seqanalyzer_ = new FFAnalyzer(ptr_graphcut_, ptr_parm_, ptr_path_);

	ptr_graphcut_->MakeLayers();
	cout << "Graph Cut completed." << endl;

	if (!ptr_seqanalyzer_->SeqPrint())
	{
		cout << "Model not printable!" << endl;
		getchar();

		return;
	}
	printf("BruteForce print done.\n");
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

	ptr_stiff->CalculateD(D, x, 1, 1, 0);
}

void FiberPrintPlugIn::Debug()
{

}
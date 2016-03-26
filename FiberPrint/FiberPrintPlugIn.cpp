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
<<<<<<< HEAD
<<<<<<< HEAD
	ptr_parm_ = ptr_parm;
	ptr_path_ = ptr_path;
=======
	ptr_graphcut_ = new GraphCut(ptr_frame, ptr_parm, path);
	ptr_seqanalyzer_ = new SeqAnalyzer(ptr_graphcut_, ptr_parm, path);
	ptr_procanalyzer_ = new ProcessAnalyzer(ptr_seqanalyzer_, path);
	ptr_parm_ = ptr_parm;
>>>>>>> 2c719846f6006b0658c93a4bf28bf6ac0236a416
=======
	ptr_parm_ = ptr_parm;
	ptr_path_ = ptr_path;
>>>>>>> 3cef735a95cf86715af239aaec2119119cb169f4
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
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 3cef735a95cf86715af239aaec2119119cb169f4
	ptr_graphcut_ = new ADMMCut(ptr_frame_, ptr_parm_, ptr_path_);
	ptr_seqanalyzer_ = new FFAnalyzer(ptr_graphcut_, ptr_parm_, ptr_path_);
	ptr_procanalyzer_ = new ProcessAnalyzer(ptr_seqanalyzer_, ptr_path_);

	//ptr_graphcut_->MakeLayers();
	//cout << "Graph Cut completed." << endl;


	if (!ptr_seqanalyzer_->SeqPrint())
	{
		cout << "Model not printable!" << endl;
		getchar();
<<<<<<< HEAD
=======
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
>>>>>>> 2c719846f6006b0658c93a4bf28bf6ac0236a416
=======
>>>>>>> 3cef735a95cf86715af239aaec2119119cb169f4

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
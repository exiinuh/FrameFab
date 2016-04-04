#include "FiberPrintPlugIn.h"


FiberPrintPlugIn::FiberPrintPlugIn()
{
	ptr_graphcut_ = new GraphCut();
	ptr_seqanalyzer_ = new SeqAnalyzer();
}


FiberPrintPlugIn::FiberPrintPlugIn(WireFrame *ptr_frame)
{
	ptr_graphcut_ = new GraphCut();
	ptr_seqanalyzer_ = new SeqAnalyzer();

	ptr_frame_ = ptr_frame;
}


FiberPrintPlugIn::FiberPrintPlugIn(WireFrame *ptr_frame, 
	FiberPrintPARM *ptr_parm, char *ptr_path)
{
	ptr_graphcut_ = new GraphCut();
	ptr_seqanalyzer_ = new SeqAnalyzer();

	ptr_frame_ = ptr_frame;
	ptr_parm_ = ptr_parm;
	ptr_path_ = ptr_path;
}


FiberPrintPlugIn::~FiberPrintPlugIn()
{
	delete ptr_seqanalyzer_;
	ptr_seqanalyzer_ = NULL;

	delete ptr_graphcut_;
	ptr_graphcut_ = NULL;
}


void FiberPrintPlugIn::Init()
{
	delete ptr_seqanalyzer_;
	ptr_seqanalyzer_ = NULL;

	delete ptr_graphcut_;
	ptr_graphcut_ = NULL;
}


void FiberPrintPlugIn::FrameFabPrint()
{
	Init();

	ptr_graphcut_ = new ADMMCut(ptr_frame_, ptr_parm_, ptr_path_);
	ptr_seqanalyzer_ = new FFAnalyzer(ptr_graphcut_, ptr_parm_, ptr_path_);
	//ptr_procanalyzer_ = new ProcAnalyzer(ptr_seqanalyzer_, ptr_path_);

	ptr_graphcut_->MakeLayers();
	cout << "Graph Cut completed." << endl;


	if (!ptr_seqanalyzer_->SeqPrint())
	{
		cout << "Model not printable!" << endl;
		getchar();

		return;
	}

	printf("FrameFab print done.\n");

	ptr_graphcut_->PrintOutTimer();
	ptr_seqanalyzer_->PrintOutTimer();
}


void FiberPrintPlugIn::BruteForcePrint()
{
	Init();

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
	Init();

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
	printf("Sweeping print done.\n");
}

void FiberPrintPlugIn::GetDeformation()
{
	DualGraph *ptr_dualgraph = new DualGraph(ptr_frame_);
	Stiffness *ptr_stiff = new Stiffness(ptr_dualgraph, ptr_parm_, ptr_path_);

	ptr_dualgraph->Dualization();
	ptr_stiff->Init();

	int Ns = ptr_dualgraph->SizeOfFreeFace();
	VX D(Ns);
	D.setZero();

	int Nd = ptr_dualgraph->SizeOfVertList();
	VX x(Nd);
	x.setOnes();

	ptr_stiff->CalculateD(D, x, 0, true, true, true);
}


int FiberPrintPlugIn::ImportPrintOrder(char *fname)
{ 
	vector<int> queue;

	FILE *fp = fopen(fname, "r");
	int e_id;
	while (fscanf(fp, "%d", &e_id) != EOF)
	{
		queue.push_back(e_id * 2);
	}
	fclose(fp);

	ptr_seqanalyzer_->InputPrintOrder(queue);
	return queue.size();
}


void FiberPrintPlugIn::ExportPrintOrder(char *fname)
{ 
	vector<int> queue;
	ptr_seqanalyzer_->OutputPrintOrder(queue);

	int N = queue.size();
	FILE *fp = fopen(fname, "w");
	for (int i = 0; i < N; i++)
	{
		fprintf(fp, "%d\n", queue[i] / 2);
	}	
	fclose(fp);
}


void FiberPrintPlugIn::Debug()
{

}
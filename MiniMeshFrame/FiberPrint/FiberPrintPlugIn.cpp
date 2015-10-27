#include "FiberPrintPlugIn.h"


FiberPrintPlugIn::FiberPrintPlugIn()
{
}


FiberPrintPlugIn::FiberPrintPlugIn(WireFrame *ptr_frame)
{
	ptr_frame_ = ptr_frame;
	ptr_graphcut_ = new GraphCut(ptr_frame);
	ptr_seqanalyzer_ = new SeqAnalyzer();
}


FiberPrintPlugIn::~FiberPrintPlugIn()
{
	delete ptr_graphcut_;
	ptr_graphcut_ = NULL;
}


void FiberPrintPlugIn::Print()
{
	ptr_graphcut_->MakeLayers();
}


vector<DualVertex*> *FiberPrintPlugIn::GetDualVertexList()
{
	return 	ptr_graphcut_->GetDualVertexList();
}


VectorXi *FiberPrintPlugIn::GetLabel()
{
	return ptr_graphcut_->GetLabel();
}


void FiberPrintPlugIn::Debug()
{
	//ptr_stiffness_->CreateM();
	//ptr_stiffness_->CreateK();
	
	ptr_graphcut_->MakeLayers();

	/*
	std::vector<int> *e_id = ptr_graphcut_->edge_id();
	assert(e_id);
	return e_id;
	*/
	/*
	FILE *fp = fopen("E:\\test.txt", "wb+");

	assert(fp);

	int M = ptr_mesh_->num_of_edge_list();
	int N = ptr_mesh_->num_of_vertex_list();
	
	vector<HE_edge*> &edge = *(ptr_mesh_->get_edges_list());
	SparseMatrix<double> K;
	K.resize(3 * N, 3 * N);
	K = ptr_graphcut_->ptr_stiff_->K_;
	for (int i = 0; i < 3*N; i++)
	{
	for (int j = 0; j < 3*N; j++)
	{
	if (K.coeff(i,j) != 0)
	{
	fprintf(fp, "(%d, %d)   %.4f\r\n", i+1, j+1, K.coeff(i, j));
	}
	}
	}
	fclose(fp);
	*/
}
#include "DualGraph.h"


DualGraph::DualGraph()
{
}


DualGraph::DualGraph(WireFrame *ptr_frame)
{
	ptr_frame_ = ptr_frame;
}


DualGraph::~DualGraph()
{
}

void DualGraph::Dualization()
{
	vector<WF_edge*> edges = *(ptr_frame_->GetEdgeList());
	int M = ptr_frame_->SizeOfEdgeList();

	vert_list_ = new vector<DualVertex*>;
	(*vert_list_).resize(M);

	for (int i = 0; i < M; i++)
	{
		(*vert_list_)[i] = new DualVertex();
	}

	int Nd = 0;
	for (int i = 0; i < M; i++)
	{
		int j = edges[i]->ppair_->ID();
		
		if (i < j)
		{
			(*vert_list_)[i]->SetDualId(Nd);
			(*vert_list_)[j]->SetDualId(Nd);
			(*vert_list_)[Nd]->SetOrgId(i);
			Nd++;
		}
	}

	vector<WF_vert*> verts = *(ptr_frame_->GetVertList());
	int N = ptr_frame_->SizeOfVertList();

	edge_list_ = new vector<DualEdge*>;
	double maxz = ptr_frame_->maxZ();
	double minz = ptr_frame_->minZ();
	for (int i = 0; i < N; i++)
	{
		if (verts[i]->Degree() > 1)
		{
			double w = 1 - (verts[i]->Position().z() - minz)/ (maxz - minz);
			int u;
			int v;
			WF_edge *edge = verts[i]->pedge_;
			while (edge->pnext_ != NULL)
			{
				WF_edge *next_edge = edge->pnext_;
				u = (*vert_list_)[edge->ID()]->dual_id();
				v = (*vert_list_)[next_edge->ID()]->dual_id();

				edge_list_->push_back(new DualEdge(u, v, w));
				edge = next_edge;
			}

			if (verts[i]->Degree() > 2)
			{
				u = (*vert_list_)[edge->ID()]->dual_id();
				v = (*vert_list_)[verts[i]->pedge_->ID()]->dual_id();

				edge_list_->push_back(new DualEdge(u, v, w));
			}
		}
	}
}


void DualGraph::Debug()
{
	/*
	vector<HE_edge*> edges = *(ptr_mesh_->get_edges_list());
	int M = edges.size();
	for (int i = 0; i < M; i++)
	{
		int end_id = edges[i]->pvert_->id();
		int start_id = edges[i]->ppair_->pvert_->id();
		printf("start vertex: %d\n", start_id);
		printf("end vertex: %d\n", end_id);
		int dual_id = (*vert_list_)[i]->dual_id();
		printf("dual vertex: %d\n", dual_id);
		printf("original vertex: %d\n", (*vert_list_)[dual_id]->original_id());
		puts("");
		getchar();
	}
	*/
}

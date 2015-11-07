#include "DualGraph.h"


DualGraph::DualGraph()
{
}


DualGraph::DualGraph(WireFrame *ptr_frame)
{
	ptr_frame_ = ptr_frame;

	vert_list_ = NULL;
	edge_list_ = NULL;
	face_list_ = NULL;

	exist_vert_.resize(ptr_frame_->SizeOfVertList());
	exist_edge_.resize(ptr_frame_->SizeOfEdgeList());
}


DualGraph::~DualGraph()
{
}


void DualGraph::Dualization()
{
	vector<WF_vert*> verts = *(ptr_frame_->GetVertList());
	vector<WF_edge*> edges = *(ptr_frame_->GetEdgeList());
	int N = ptr_frame_->SizeOfVertList();
	int M = ptr_frame_->SizeOfEdgeList();

	// first time & all exsits
	fill(exist_vert_.begin(), exist_vert_.end(), true);
	fill(exist_edge_.begin(), exist_edge_.end(), true);

	// Initialize vert_list_ & edge_list_ & face_list_
	vert_list_ = new vector<DualVertex*>;
	(*vert_list_).resize(M);
	for (int i = 0; i < M; i++)
	{
		(*vert_list_)[i] = new DualVertex();
	}

	edge_list_ = new vector<DualEdge*>;

	// In dualization, each vert in original graph turns into dual face
	face_list_ = new vector<DualFace*>;
	(*face_list_).resize(N);
	for (int i = 0; i < N; i++)
	{
		(*face_list_)[i] = new DualFace();
	}

	Establish();
}


void DualGraph::Dualization(VectorXd *ptr_x)
{
	vector<WF_vert*> verts = *(ptr_frame_->GetVertList());
	vector<WF_edge*> edges = *(ptr_frame_->GetEdgeList());
	int N = ptr_frame_->SizeOfVertList();
	int M = ptr_frame_->SizeOfEdgeList();
	int Nd = Nd_;

	fill(exist_vert_.begin(), exist_vert_.end(), false);
	fill(exist_edge_.begin(), exist_edge_.end(), false);
	for (int e_id = 0; e_id < Nd; e_id++)
	{
		if ((*ptr_x)[e_id])
		{
			int i = (*vert_list_)[e_id]->orig_id();
			int j = edges[i]->ppair_->ID();
			int u = edges[i]->pvert_->ID();
			int v = edges[i]->ppair_->pvert_->ID();
			exist_vert_[u] = exist_vert_[v] = true;
			exist_edge_[i] = exist_edge_[j] = true;
		}
	}

	// Release the last edge_list_
	int Md = edge_list_->size();
	for (int i = 0; i < Md; i++)
	{
		delete (*edge_list_)[i];
	}
	edge_list_->clear();

	Establish();
}


void DualGraph::Establish()
{
	vector<WF_vert*> verts = *(ptr_frame_->GetVertList());
	vector<WF_edge*> edges = *(ptr_frame_->GetEdgeList());
	int N = ptr_frame_->SizeOfVertList();
	int M = ptr_frame_->SizeOfEdgeList();

	// vert_list_
	int Nd = 0;
	for (int i = 0; i < M; i++)
	{
		int j = edges[i]->ppair_->ID();
		if (i < j)
		{
			if (exist_edge_[i])
			{
				(*vert_list_)[i]->SetDualId(Nd);
				(*vert_list_)[j]->SetDualId(Nd);
				(*vert_list_)[Nd]->SetOrigId(i);
				Nd++;
			}
			else
			{
				(*vert_list_)[i]->SetDualId(-1);
				(*vert_list_)[j]->SetDualId(-1);
			}
		}
	}

	int Fd = 0;			// New id for each DualFace (node in orig graph)
	double maxz = ptr_frame_->maxZ();
	double minz = ptr_frame_->minZ();
	for (int i = 0; i < N; i++)
	{
		if (!exist_vert_[i])
		{
			continue;
		}

		// face_list_
		// representing renumbering of nodes in original graph after each cut
		(*face_list_)[i]->SetDualId(Fd);
		(*face_list_)[Fd]->SetOrigId(i);
		Fd++;

		// edge_list_
		// each dual id corresponding to shared orig nodes of two orig edge
		if (verts[i]->Degree() > 1)
		{
			double w = 1 - (verts[i]->Position().z() - minz) / (maxz - minz);
			int u;
			int v;
			WF_edge *edge = verts[i]->pedge_;
			while (edge->pnext_ != NULL)
			{
				WF_edge *next_edge = edge->pnext_;
				u = (*vert_list_)[edge->ID()]->dual_id();
				v = (*vert_list_)[next_edge->ID()]->dual_id();

				if (u != -1 && v != -1)
				{
					edge_list_->push_back(new DualEdge(u, v, w));
				}
				edge = next_edge;
			}

			if (verts[i]->Degree() > 2)
			{
				u = (*vert_list_)[edge->ID()]->dual_id();
				v = (*vert_list_)[verts[i]->pedge_->ID()]->dual_id();

				if (u != -1 && v != -1)
				{
					edge_list_->push_back(new DualEdge(u, v, w));
				}
			}
		}
	}

	Nd_ = Nd;
	Md_ = edge_list_->size();
	Fd_ = Fd;
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

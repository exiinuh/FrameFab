#include "DualGraph.h"


DualGraph::DualGraph()
{
}


DualGraph::DualGraph(WireFrame *ptr_frame)
{
	ptr_frame_ = ptr_frame;

	int N = ptr_frame_->SizeOfVertList();
	int M = ptr_frame_->SizeOfEdgeList();

	vert_list_ = new vector<DualVertex*>;
	vert_list_->resize(M);
	for (int i = 0; i < M; i++)
	{
		(*vert_list_)[i] = new DualVertex();
	}

	edge_list_ = new vector<DualEdge*>;

	face_list_ = new vector<DualFace*>;
	face_list_->resize(N);
	for (int i = 0; i < N; i++)
	{
		(*face_list_)[i] = new DualFace();
	}

	exist_vert_.resize(N);
	exist_edge_.resize(M);
}


DualGraph::~DualGraph()
{
	int N = vert_list_->size();
	for (int i = 0; i < N; i++)
	{
		delete (*vert_list_)[i];
		(*vert_list_)[i] = NULL;
	}
	delete vert_list_;
	vert_list_ = NULL;

	int M = edge_list_->size();
	for (int i = 0; i < M; i++)
	{
		delete (*edge_list_)[i];
		(*edge_list_)[i] = NULL;
	}
	delete edge_list_;
	edge_list_ = NULL;

	int F = face_list_->size();
	for (int i = 0; i < F; i++)
	{
		delete (*face_list_)[i];
		(*face_list_)[i] = NULL;
	}
	delete face_list_;
	face_list_ = NULL;
}


void DualGraph::Dualization()
{
	maxz_ = ptr_frame_->maxZ();
	minz_ = ptr_frame_->minZ();

	// first time & all exsits
	fill(exist_vert_.begin(), exist_vert_.end(), true);
	fill(exist_edge_.begin(), exist_edge_.end(), true);

	Establish();
}


void DualGraph::UpdateDualization(VectorXd *ptr_x)
{
	maxz_ = -1e20;

	int Nd = Nd_;

	fill(exist_vert_.begin(), exist_vert_.end(), false);
	fill(exist_edge_.begin(), exist_edge_.end(), false);
	for (int e_id = 0; e_id < Nd; e_id++)
	{
		if ((*ptr_x)[e_id])
		{
			WF_edge *ei = ptr_frame_->GetEdge((*vert_list_)[e_id]->orig_id());
			WF_edge *ej = ei->ppair_;
			WF_vert *u = ej->pvert_;
			WF_vert *v = ei->pvert_;

			if (u->Position().z() > maxz_)
			{
				maxz_ = u->Position().z();
			}
			if (v->Position().z() > maxz_)
			{
				maxz_ = v->Position().z();
			}

			exist_vert_[u->ID()] = exist_vert_[v->ID()] = true;
			exist_edge_[ei->ID()] = exist_edge_[ej->ID()] = true;
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
	int N = ptr_frame_->SizeOfVertList();
	int M = ptr_frame_->SizeOfEdgeList();

	// vert_list_
	int Nd = 0;
	for (int i = 0; i < M; i++)
	{
		WF_edge *ei = ptr_frame_->GetEdge(i);
		int j = ei->ppair_->ID();
		if (i < j)
		{
			if (exist_edge_[i])
			{
				(*vert_list_)[i]->SetDualId(Nd);
				(*vert_list_)[j]->SetDualId(Nd);
				(*vert_list_)[Nd]->SetOrigId(i);
				(*vert_list_)[Nd]->SetHeight((ei->CenterPos()).z());
				Nd++;
			}
			else
			{
				(*vert_list_)[i]->SetDualId(-1);
				(*vert_list_)[j]->SetDualId(-1);
			}
		}
	}

	int Fd = 0;
	for (int i = 0; i < N; i++)
	{
		if (!exist_vert_[i])
		{
			continue;
		}

		// face_list_
		(*face_list_)[i]->SetDualId(Fd);
		(*face_list_)[Fd]->SetOrigId(i);
		Fd++;

		// edge_list_
		if (ptr_frame_->GetDegree(i) > 1)
		{
			//double w = 1 - (verts[i]->Position().z() - minz) / (maxz - minz);
			double w = exp(- 3 * pow((ptr_frame_->GetPosition(i).z() - minz_) / (maxz_ - minz_), 2));
			int u;
			int v;
			WF_edge *edge = ptr_frame_->GetNeighborEdge(i);
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

			if (ptr_frame_->GetDegree(i) > 2)
			{
				u = (*vert_list_)[edge->ID()]->dual_id();
				v = (*vert_list_)[ptr_frame_->GetNeighborEdge(i)->ID()]->dual_id();

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

/*
void DualGraph::Debug()
{
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
}
*/
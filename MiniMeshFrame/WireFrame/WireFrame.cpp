#include "WireFrame.h"


WireFrame::WireFrame()
		  :delta_tol_(2e-3), unify_size_(2.0)
{
	pvert_list_ = new vector<WF_vert*>;
	pedge_list_ = new vector<WF_edge*>;
}


WireFrame::~WireFrame()
{
	int N = pvert_list_->size();
	for (int i = 0; i < N; i++)
	{
		delete (*pvert_list_)[i];
		(*pvert_list_)[i] = NULL;
	}
	delete pvert_list_;
	pvert_list_ = NULL;

	int M = pedge_list_->size();
	for (int i = 0; i < M; i++)
	{
		delete (*pedge_list_)[i];
		(*pedge_list_)[i] = NULL;
	}
	delete pedge_list_;
	pedge_list_ = NULL;
}


void WireFrame::LoadFromOBJ(const char *path)
{
	FILE *fp = fopen(path, "r");

	try
	{
		// read vertexes
		fseek(fp, 0, SEEK_SET);
		char pLine[512];
		char *tok;
		while (fgets(pLine, 512, fp))
		{
			if (pLine[0] == 'v' && pLine[1] == ' ')
			{
				Vec3f p;
				char tmp[128];
				tok = strtok(pLine, " ");
				for (int i = 0; i<3; i++)
				{
					tok = strtok(NULL, " ");
					strcpy(tmp, tok);
					tmp[strcspn(tmp, " ")] = 0;
					p[i] = (float)atof(tmp);
				}
				p = point(p.x(), p.y(), p.z());
				InsertVertex(p);
			}
		}

		// read lines
		fseek(fp, 0, SEEK_SET);
		char c;
		int prev;
		int curv;
		while (c = fgetc(fp), c != EOF)
		{
			while (c != 'l' && c != EOF)
			{
				c = fgetc(fp);
			}

			if ((c = fgetc(fp)) != ' ')
			{
				continue;
			}

			prev = -1;
			while (c != '\n' && c != EOF)
			{
				while (c = fgetc(fp), c != '\n' && c != EOF && !isdigit(c))
					;

				for (curv = 0; isdigit(c); c = fgetc(fp))
				{
					curv = curv * 10 + c - '0';
				}

				if (prev != -1)
				{
					InsertEdge(prev - 1, curv - 1);
				}

				prev = curv;
			}
		}

		// read faces
		fseek(fp, 0, SEEK_SET);
		while (fgets(pLine, 512, fp))
		{
			if (pLine[0] == 'f' && pLine[1] == ' ')
			{
				vector<int> pts;
				pts.resize(3);

				tok = strtok(pLine, " ");
				char tmp[128];
				for (int i = 0; i<3; i++)
				{
					tok = strtok(NULL, " ");
					strcpy(tmp, tok);
					tmp[strcspn(tmp, " ")] = 0;
					pts[i] = (int)atof(tmp) - 1;
				}
				InsertEdge(pts[0], pts[1]);
				InsertEdge(pts[1], pts[2]);
				InsertEdge(pts[2], pts[0]);
			}
		}

		// read bounds
		fseek(fp, 0, SEEK_SET);
		while (c = fgetc(fp), c != EOF)
		{
			while (c != 'b' && c != EOF)
			{
				c = fgetc(fp);
			}

			if ((c = fgetc(fp)) != ' ')
			{
				continue;
			}

			int bound_id;
			while (c != '\n' && c != EOF)
			{
				while (c = fgetc(fp), c != '\n' && c != EOF && !isdigit(c))
					;

				for (bound_id = 0; isdigit(c); c = fgetc(fp))
				{
					bound_id = bound_id * 10 + c - '0';
				}

				(*pvert_list_)[bound_id - 1]->SetFixed(true);
				WF_edge *e = (*pvert_list_)[bound_id - 1]->pedge_;
				while (e != NULL)
				{
					e->SetPillar(true);
					e->ppair_->SetPillar(true);
					e = e->pnext_;
				}
			}
		}

		Unify();
	}
	catch (...)
	{
		return;
	}

	fclose(fp);
}


void WireFrame::WriteToOBJ(const char *path)
{
	FILE *fp = fopen(path, "wb+");
	int N = SizeOfVertList();
	int M = SizeOfEdgeList();

	for (int i = 0; i < N; i++)
	{
		point p = (*pvert_list_)[i]->Position();
		fprintf(fp, "v %lf %lf %lf\n", p.x(), p.y(), p.z());
	}

	for (int i = 0; i < N; i++)
	{
		WF_edge *edge = (*pvert_list_)[i]->pedge_;
		while (edge != NULL)
		{
			if (edge->ID() < edge->ppair_->ID())
			{
				fprintf(fp, "l %d %d\n", i + 1, edge->pvert_->ID() + 1);
			}
			edge = edge->pnext_;
		}
	}

	for (int i = 0; i < N; i++)
	{
		if ((*pvert_list_)[i]->isFixed())
		{
			fprintf(fp, "b %d\n", i + 1);
		}
	}

	fclose(fp);
}


WF_vert* WireFrame::InsertVertex(Vec3f p)
{
	WF_vert *vert = new WF_vert(p);
	pvert_list_->push_back(vert);
	int N = pvert_list_->size();
	vert->SetID(N - 1);
	return vert;
}


void WireFrame::InsertEdge(int u, int v)
{
	WF_edge *edge = (*pvert_list_)[u]->pedge_;
	while (edge != NULL)
	{
		if (edge->pvert_->ID() == v)
		{
			return;
		}
		edge = edge->pnext_;
	}

	InsertOneWayEdge((*pvert_list_)[u], (*pvert_list_)[v]);
	InsertOneWayEdge((*pvert_list_)[v], (*pvert_list_)[u]);
	int M = pedge_list_->size();
	(*pedge_list_)[M - 1]->SetID(M - 1);
	(*pedge_list_)[M - 2]->SetID(M - 2);
	(*pedge_list_)[M - 1]->ppair_ = (*pedge_list_)[M - 2];
	(*pedge_list_)[M - 2]->ppair_ = (*pedge_list_)[M - 1];
}


void WireFrame::InsertOneWayEdge(WF_vert *u, WF_vert *v)
{
	WF_edge *edge = new WF_edge();
	edge->pvert_ = v;
	edge->pnext_ = u->pedge_;
	u->pedge_ = edge;
	u->IncreaseDegree();
	
	pedge_list_->push_back(edge);
}


void WireFrame::Unify()
{
	maxx_ = -1e10;
	maxy_ = -1e10;
	maxz_ = -1e10;
	minx_ = 1e10;
	miny_ = 1e10;
	minz_ = 1e10;

	int N = SizeOfVertList();
	for (int i = 0; i < N; i++)
	{
		(*pvert_list_)[i]->SetID(i);

		if (!(*pvert_list_)[i]->isFixed())
		{
			point p = (*pvert_list_)[i]->Position();
			if (p.x() > maxx_)
			{
				maxx_ = p.x();
			}
			if (p.y() > maxy_)
			{
				maxy_ = p.y();
			}
			if (p.z() > maxz_)
			{
				maxz_ = p.z();
			}
			if (p.x() < minx_)
			{
				minx_ = p.x();
			}
			if (p.y() < miny_)
			{
				miny_ = p.y();
			}
			if (p.z() < minz_)
			{
				minz_ = p.z();
			}
		}
	}

	int M = SizeOfEdgeList();
	for (int i = 0; i < M; i++)
	{
		(*pedge_list_)[i]->SetID(i);
	}

	float scaleX = maxx_ - minx_;
	float scaleY = maxy_ - miny_;
	float scaleZ = maxz_ - minz_;
	float scaleMax = scaleX;
	if (scaleMax < scaleY)
	{
		scaleMax = scaleY;
	}
	if (scaleMax < scaleZ)
	{
		scaleMax = scaleZ;
	}

	scaleV_ = unify_size_ / scaleMax;
	center_pos_ = point((minx_ + maxx_) / 2.f, (miny_ + maxy_) / 2.f, (minz_ + maxz_) / 2.f);

	for (size_t i = 0; i < N; i++)
	{
		(*pvert_list_)[i]->SetRenderPos( Unify((*pvert_list_)[i]->Position()) );
	}
}


point WireFrame::Unify(Vec3f p)
{
	return (p - center_pos_) * scaleV_;
}


void WireFrame::SimplifyFrame()
{
	int N = SizeOfVertList();
	int M = SizeOfEdgeList();

	vector<bool> delete_vert;
	delete_vert.resize(N);
	fill(delete_vert.begin(), delete_vert.end(), false);

	vector<bool> delete_edge;
	delete_edge.resize(M);
	fill(delete_edge.begin(), delete_edge.end(), false);

	for (int i = 0; i < N; i++)
	{
		WF_vert *u = (*pvert_list_)[i];
		if (u->Degree() == 2)
		{
			WF_edge *e1 = (*pvert_list_)[i]->pedge_;
			WF_edge *e2 = (*pvert_list_)[i]->pedge_->pnext_;
			WF_vert *v1 = e1->pvert_;
			WF_vert *v2 = e2->pvert_;

			if (ArcHeight(u->Position(), v1->Position(), v2->Position()) < delta_tol_)
			{
				delete_vert[u->ID()] = true;
				delete_edge[e1->ID()] = true;
				delete_edge[e2->ID()] = true;

				WF_edge *e1_pair = e1->ppair_;
				WF_edge *e2_pair = e2->ppair_;
				e1_pair->pvert_ = v2;
				e2_pair->pvert_ = v1;
				e1_pair->ppair_ = e2_pair;
				e2_pair->ppair_ = e1_pair;
			}
		}
	}

	vector<WF_vert*>::iterator itv = pvert_list_->begin();
	while (itv != pvert_list_->end())
	{
		if (delete_vert[(*itv)->ID()])
		{
			itv = pvert_list_->erase(itv);
		}
		else
		{
			itv++;
		}
	}

	vector<WF_edge*>::iterator ite = pedge_list_->begin();
	while (ite != pedge_list_->end())
	{
		if (delete_edge[(*ite)->ID()])
		{
			ite = pedge_list_->erase(ite);
		}
		else
		{
			ite++;
		}
	}

	Unify();
}


void WireFrame::ProjectBound(vector<int> *bound)
{
	int N = SizeOfVertList();
	for (int i = 0; i < N; i++)
	{
		WF_vert *u = (*pvert_list_)[i]; 
		if ((*bound)[i])
		{
			point v_pos = u->Position();
			v_pos.z() = minz_ - (maxz_ - minz_)*0.1;

			WF_vert *v = InsertVertex(v_pos);
			v->SetFixed(true);
			
			int j = SizeOfVertList() - 1; 
			InsertEdge(i, j);
			/*
			if (j > N)
			{
				InsertEdge(j - 1, j);
			}
			*/
		}
	}
	
	Unify();
	//InsertEdge(N, SizeOfVertList() - 1);
	//UpdateFrame();
}

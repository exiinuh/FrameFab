#include "WireFrame.h"


WireFrame::WireFrame()
	:fixed_vert_(0), delta_tol_(5e-4), unify_size_(2.0), max_layer_(0)
{
	pvert_list_ = new vector<WF_vert*>;
	pedge_list_ = new vector<WF_edge*>;
	pface_list_ = new vector<WF_face*>;
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

	int F = pface_list_->size();
	for (int i = 0; i < F; i++)
	{
		delete (*pface_list_)[i];
		(*pface_list_)[i] = NULL;
	}
	delete pface_list_;
	pface_list_ = NULL;
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
		vector<WF_vert*> tmp_points;
		while (fgets(pLine, 512, fp))
		{
			if (pLine[0] == 'v' && pLine[1] == ' ')
			{
				Vec3f p;
				char tmp[128];
				tok = strtok(pLine, " ");
				for (int i = 0; i < 3; i++)
				{
					tok = strtok(NULL, " ");
					strcpy(tmp, tok);
					tmp[strcspn(tmp, " ")] = 0;
					p[i] = (float)atof(tmp);
				}

				p = point(p.x(), p.y(), p.z());
				WF_vert *u = InsertVertex(p);
				tmp_points.push_back(u);
			}
		}

		// read lines
		char c;
		int prev;
		int curv;
		fseek(fp, 0, SEEK_SET);
		while (c = fgetc(fp), c != EOF)
		{
			while (c != 'l' && c != EOF)
			{
				c = fgetc(fp);
			}

			if (c == '\n' || c == EOF || (c = fgetc(fp)) != ' ')
			{
				continue;
			}

			prev = -1;
			while (c != '\n' && c != EOF)
			{
				while (c = fgetc(fp), c != '\n' && c != EOF && !isdigit(c))
					;

				if (c == '\n' || c == EOF)
				{
					break;
				}

				for (curv = 0; isdigit(c); c = fgetc(fp))
				{
					curv = curv * 10 + c - '0';
				}
				curv--;

				if (prev != -1)
				{
					InsertEdge(tmp_points[prev], tmp_points[curv]);
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
				vector<WF_vert*> bound_points;
				tok = strtok(pLine, " ");
				char tmp[128];
				while (tok = strtok(NULL, " "))
				{
					strcpy(tmp, tok);
					tmp[strcspn(tmp, " ")] = 0;
					int u = (int)atof(tmp) - 1;
					bound_points.push_back(tmp_points[u]);
				}

				int Bn = bound_points.size();
				for (int i = 0; i < Bn - 1; i++)
				{
					InsertEdge(bound_points[i], bound_points[i + 1]);
				}
				InsertEdge(bound_points[Bn - 1], bound_points[0]);
				InsertFace(bound_points);
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
	int F = SizeOfFaceList();

	for (int i = 0; i < N; i++)
	{
		point p = (*pvert_list_)[i]->Position();
		fprintf(fp, "v %lf %lf %lf\n", p.x(), p.y(), p.z());
	}

	for (int i = 0; i < M; i++)
	{
		WF_edge *e1 = (*pedge_list_)[i];
		WF_edge *e2 = e1->ppair_;
		if (i < e2->ID())
		{
			fprintf(fp, "l %d %d\n", e1->pvert_->ID() + 1, e2->pvert_->ID() + 1);
		}
	}

	for (int i = 0; i < F; i++)
	{
		fprintf(fp, "f");
		vector<WF_vert*> bound_points = *((*pface_list_)[i]->bound_points_);
		int n = bound_points.size();
		for (int j = 0; j < n; j++)
		{
			fprintf(fp, " %d", bound_points[j]->ID() + 1);
		}
		fprintf(fp, "\n");
	}

	fclose(fp);
}


void WireFrame::LoadFromPWF(const char *path)
{
	FILE *fp = fopen(path, "r");

	try
	{
		// read vertexes
		fseek(fp, 0, SEEK_SET);
		char pLine[512];
		char *tok;
		vector<WF_vert*> tmp_points;
		while (fgets(pLine, 512, fp))
		{
			if (pLine[0] == 'v' && pLine[1] == ' ')
			{
				Vec3f p;
				char tmp[128];
				tok = strtok(pLine, " ");
				for (int i = 0; i < 3; i++)
				{
					tok = strtok(NULL, " ");
					strcpy(tmp, tok);
					tmp[strcspn(tmp, " ")] = 0;
					p[i] = (float)atof(tmp);
				}

				p = point(p.x(), p.y(), p.z());
				WF_vert *u = InsertVertex(p);
				tmp_points.push_back(u);
			}
		}

		// read layer
		fseek(fp, 0, SEEK_SET);
		while (fgets(pLine, 512, fp))
		{
			if (pLine[0] == 'g' && pLine[1] == ' ')
			{
				tok = strtok(pLine, " ");

				char tmp[128];
				tok = strtok(NULL, " ");
				strcpy(tmp, tok);
				tmp[strcspn(tmp, " ")] = 0;
				int u = (int)atof(tmp) - 1;

				tok = strtok(NULL, " ");
				strcpy(tmp, tok);
				tmp[strcspn(tmp, " ")] = 0;
				int v = (int)atof(tmp) - 1;

				tok = strtok(NULL, " ");
				strcpy(tmp, tok);
				tmp[strcspn(tmp, " ")] = 0;
				int layer = (int)atof(tmp);

				WF_edge *e = InsertEdge((*pvert_list_)[u], (*pvert_list_)[v]);
				if (e != NULL)
				{
					e->SetLayer(layer);
					e->ppair_->SetLayer(layer);
				}
			}
		}

		// read ceiling
		fseek(fp, 0, SEEK_SET);
		while (fgets(pLine, 512, fp))
		{
			if (pLine[0] == 'c' && pLine[1] == ' ')
			{
				tok = strtok(pLine, " ");

				char tmp[128];
				tok = strtok(NULL, " ");
				strcpy(tmp, tok);
				tmp[strcspn(tmp, " ")] = 0;
				int u = (int)atof(tmp) - 1;

				tok = strtok(NULL, " ");
				strcpy(tmp, tok);
				tmp[strcspn(tmp, " ")] = 0;
				int v = (int)atof(tmp) - 1;

				WF_edge *e = InsertEdge((*pvert_list_)[u], (*pvert_list_)[v]);
				if (e != NULL)
				{
					e->SetCeiling(true);
					e->ppair_->SetCeiling(true);
				}
			}
		}

		// read faces
		fseek(fp, 0, SEEK_SET);
		while (fgets(pLine, 512, fp))
		{
			if (pLine[0] == 'f' && pLine[1] == ' ')
			{
				vector<WF_vert*> bound_points;
				tok = strtok(pLine, " ");
				char tmp[128];
				while (tok = strtok(NULL, " "))
				{
					strcpy(tmp, tok);
					tmp[strcspn(tmp, " ")] = 0;
					int u = (int)atof(tmp) - 1;
					bound_points.push_back(tmp_points[u]);
				}

				int Bn = bound_points.size();
				for (int i = 0; i < Bn - 1; i++)
				{
					InsertEdge(bound_points[i], bound_points[i + 1]);
				}
				InsertEdge(bound_points[Bn - 1], bound_points[0]);
				InsertFace(bound_points);
			}
		}

		// read lines
		char c;
		int prev;
		int curv;
		fseek(fp, 0, SEEK_SET);
		while (c = fgetc(fp), c != EOF)
		{
			while (c != 'l' && c != EOF)
			{
				c = fgetc(fp);
			}

			if (c == '\n' || c == EOF || (c = fgetc(fp)) != ' ')
			{
				continue;
			}

			prev = -1;
			while (c != '\n' && c != EOF)
			{
				while (c = fgetc(fp), c != '\n' && c != EOF && !isdigit(c))
					;

				if (c == '\n' || c == EOF)
				{
					break;
				}

				for (curv = 0; isdigit(c); c = fgetc(fp))
				{
					curv = curv * 10 + c - '0';
				}
				curv--;

				if (prev != -1)
				{
					InsertEdge(tmp_points[prev], tmp_points[curv]);
				}

				prev = curv;
			}
		}

		// read base
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

			int base_id;
			while (c != '\n' && c != EOF)
			{
				while (c = fgetc(fp), c != '\n' && c != EOF && !isdigit(c))
					;

				for (base_id = 0; isdigit(c); c = fgetc(fp))
				{
					base_id = base_id * 10 + c - '0';
				}

				(*pvert_list_)[base_id - 1]->SetFixed(true);
				WF_edge *e = (*pvert_list_)[base_id - 1]->pedge_;
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


void WireFrame::WriteToPWF(const char *path)
{
	FILE *fp = fopen(path, "wb+");
	int N = SizeOfVertList();
	int M = SizeOfEdgeList();
	int F = SizeOfFaceList();

	for (int i = 0; i < N; i++)
	{
		point p = (*pvert_list_)[i]->Position();
		fprintf(fp, "v %lf %lf %lf\n", p.x(), p.y(), p.z());
	}

	for (int i = 0; i < F; i++)
	{
		fprintf(fp, "f");
		vector<WF_vert*> bound_points = *((*pface_list_)[i]->bound_points_);
		int n = bound_points.size();
		for (int j = 0; j < n; j++)
		{
			fprintf(fp, " %d", bound_points[j]->ID() + 1);
		}
		fprintf(fp, "\n");
	}

	for (int i = 0; i < M; i++)
	{
		WF_edge *e1 = (*pedge_list_)[i];
		WF_edge *e2 = e1->ppair_;
		if (i < e2->ID())
		{
			fprintf(fp, "l %d %d\n", e1->pvert_->ID() + 1, e2->pvert_->ID() + 1);
		}
	}

	for (int i = 0; i < N; i++)
	{
		if ((*pvert_list_)[i]->isFixed())
		{
			fprintf(fp, "b %d\n", i + 1);
		}
	}

	for (int i = 0; i < M; i++)
	{
		WF_edge *e1 = (*pedge_list_)[i];
		WF_edge *e2 = e1->ppair_;
		if (i < e2->ID() && e1->isCeiling())
		{
			fprintf(fp, "c %d %d\n", e2->pvert_->ID() + 1, e1->pvert_->ID() + 1);
		}
	}

	for (int i = 0; i < M; i++)
	{
		WF_edge *e1 = (*pedge_list_)[i];
		WF_edge *e2 = e1->ppair_;
		if (i < e2->ID() && e1->Layer() != -1)
		{
			fprintf(fp, "g %d %d %d\n", e2->pvert_->ID() + 1, 
				e1->pvert_->ID() + 1, e1->Layer());
		}
	}

	fclose(fp);
}


void WireFrame::WriteLayerToOBJ(const char *path, int layer)
{
	FILE *fp = fopen(path, "wb+");
	int N = SizeOfVertList();
	int M = SizeOfEdgeList();
	int F = SizeOfFaceList();

	vector<int> export_vert;
	vector<int> hash(N);
	fill(hash.begin(), hash.end(), -1);

	for (int i = 0; i < M; i++)
	{
		WF_edge *e1 = (*pedge_list_)[i];
		WF_edge *e2 = e1->ppair_;
		if (i < e2->ID() && e1->Layer() == layer)
		{
			int u = e2->pvert_->ID();
			int v = e1->pvert_->ID();
			if (hash[u] == -1)
			{
				export_vert.push_back(u);
				hash[u] = export_vert.size();
			}
			if (hash[v] == -1)
			{
				export_vert.push_back(v);
				hash[v] = export_vert.size();
			}
		}
	}

	int Ne = export_vert.size();
	for (int i = 0; i < Ne; i++)
	{
		point p = (*pvert_list_)[export_vert[i]]->Position();
		fprintf(fp, "v %lf %lf %lf\n", p.x(), p.y(), p.z());
	}

	//for (int i = 0; i < F; i++)
	//{
	//	fprintf(fp, "f");
	//	vector<WF_vert*> bound_points = *((*pface_list_)[i]->bound_points_);
	//	int n = bound_points.size();
	//	for (int j = 0; j < n; j++)
	//	{
	//		fprintf(fp, " %d", bound_points[j]->ID() + 1);
	//	}
	//	fprintf(fp, "\n");
	//}

	for (int i = 0; i < M; i++)
	{
		WF_edge *e1 = (*pedge_list_)[i];
		WF_edge *e2 = e1->ppair_;
		if (i < e2->ID() && e1->Layer() == layer)
		{
			int u = e2->pvert_->ID();
			int v = e1->pvert_->ID();
			fprintf(fp, "l %d %d\n", hash[u], hash[v]);
		}
	}

	fclose(fp);
}


void WireFrame::WriteLayersToPWF(const char *path, int max_layer)
{
	FILE *fp = fopen(path, "wb+");
	int N = SizeOfVertList();
	int M = SizeOfEdgeList();
	int F = SizeOfFaceList();
	
	vector<int> export_vert;
	vector<int> hash(N);
	fill(hash.begin(), hash.end(), -1);

	for (int i = 0; i < M; i++)
	{
		WF_edge *e1 = (*pedge_list_)[i];
		WF_edge *e2 = e1->ppair_;
		if (i < e2->ID() && e1->Layer() < max_layer)
		{
			int u = e2->pvert_->ID();
			int v = e1->pvert_->ID();
			if (hash[u] == -1)
			{
				export_vert.push_back(u);
				hash[u] = export_vert.size();
			}
			if (hash[v] == -1)
			{
				export_vert.push_back(v);
				hash[v] = export_vert.size();
			}
		}
	}

	int Ne = export_vert.size();
	for (int i = 0; i < Ne; i++)
	{
		point p = (*pvert_list_)[export_vert[i]]->Position();
		fprintf(fp, "v %lf %lf %lf\n", p.x(), p.y(), p.z());
	}

	//for (int i = 0; i < F; i++)
	//{
	//	fprintf(fp, "f");
	//	vector<WF_vert*> bound_points = *((*pface_list_)[i]->bound_points_);
	//	int n = bound_points.size();
	//	for (int j = 0; j < n; j++)
	//	{
	//		fprintf(fp, " %d", bound_points[j]->ID() + 1);
	//	}
	//	fprintf(fp, "\n");
	//}

	for (int i = 0; i < M; i++)
	{
		WF_edge *e1 = (*pedge_list_)[i];
		WF_edge *e2 = e1->ppair_;
		if (i < e2->ID() && e1->Layer() < max_layer)
		{
			int u = e2->pvert_->ID();
			int v = e1->pvert_->ID();
			fprintf(fp, "l %d %d\n", hash[u], hash[v]);
		}
	}

	for (int i = 0; i < Ne; i++)
	{
		int u = export_vert[i];
		if ((*pvert_list_)[u]->isFixed())
		{
			fprintf(fp, "b %d\n", hash[u]);
		}
	}

	for (int i = 0; i < M; i++)
	{
		WF_edge *e1 = (*pedge_list_)[i];
		WF_edge *e2 = e1->ppair_;
		if (i < e2->ID() && e1->Layer() < max_layer && e1->isCeiling())
		{
			int u = e2->pvert_->ID();
			int v = e1->pvert_->ID();
			fprintf(fp, "c %d %d\n", hash[u], hash[v]);
		}
	}

	for (int i = 0; i < M; i++)
	{
		WF_edge *e1 = (*pedge_list_)[i];
		WF_edge *e2 = e1->ppair_;
		if (i < e2->ID() && e1->Layer() < max_layer && e1->Layer() != -1)
		{
			int u = e2->pvert_->ID();
			int v = e1->pvert_->ID();
			fprintf(fp, "g %d %d %d\n", hash[u], hash[v], e1->Layer());
		}
	}

	fclose(fp);
}


void WireFrame::ExportPoints(const char *path)
{
	FILE *fp = fopen(path, "wb+");

	int N = SizeOfVertList();
	for (int i = 0; i < N; i++)
	{
		point p = (*pvert_list_)[i]->RenderPos();
		fprintf(fp, "%lf %lf %lf\n", p.x(), p.y(), p.z());
	}

	fclose(fp);
}


void WireFrame::ExportLines(const char *path)
{
	FILE *fp = fopen(path, "wb+");

	int M = SizeOfEdgeList();
	for (int i = 0; i < M; i++)
	{
		WF_edge *e1 = (*pedge_list_)[i];
		WF_edge *e2 = e1->ppair_;
		if (e1->ID() < e2->ID())
		{
			point p1 = e2->pvert_->RenderPos();
			point p2 = e1->pvert_->RenderPos();
			fprintf(fp, "%lf %lf %lf ", p1.x(), p1.y(), p1.z());
			fprintf(fp, "%lf %lf %lf\n", p2.x(), p2.y(), p2.z());
		}
	}

	fclose(fp);
}


void WireFrame::ExportLayer(const char *path, int layer)
{
	string vert_path = (string)path + "/verts_layer.txt";
	string edge_path = (string)path + "/edges_layer.txt";
	FILE *fp_v = fopen(vert_path.c_str(), "wb+");
	FILE *fp_e = fopen(edge_path.c_str(), "wb+");

	int N = SizeOfVertList();
	int M = SizeOfEdgeList();
	int F = SizeOfFaceList();

	vector<bool> is_export_vert(N);
	fill(is_export_vert.begin(), is_export_vert.end(), false);

	for (int i = 0; i < M; i++)
	{
		WF_edge *e1 = (*pedge_list_)[i];
		WF_edge *e2 = e1->ppair_;
		if (i < e2->ID() && e1->Layer() == layer)
		{
			is_export_vert[e2->pvert_->ID()] = true;
			is_export_vert[e1->pvert_->ID()] = true;

			point u = e2->pvert_->RenderPos();
			point v = e1->pvert_->RenderPos();

			fprintf(fp_e, "%lf %lf %lf %lf %lf %lf\n",
				u.x(), u.y(), u.z(), v.x(), v.y(), v.z());
		}
	}

	for (int i = 0; i < N; i++)
	{
		if (is_export_vert[i])
		{
			point u = (*pvert_list_)[i]->RenderPos();
			fprintf(fp_v, "%lf %lf %lf\n", u.x(), u.y(), u.z());
		}
	}

	fclose(fp_v);
	fclose(fp_e);
}


void WireFrame::ExportLayers(const char *path, int max_layer)
{
	string vert_path = (string)path + "/verts_layers.txt";
	string edge_path = (string)path + "/edges_layers.txt";
	FILE *fp_v = fopen(vert_path.c_str(), "wb+");
	FILE *fp_e = fopen(edge_path.c_str(), "wb+");

	int N = SizeOfVertList();
	int M = SizeOfEdgeList();
	int F = SizeOfFaceList();

	vector<bool> is_export_vert(N);
	fill(is_export_vert.begin(), is_export_vert.end(), false);

	for (int i = 0; i < M; i++)
	{
		WF_edge *e1 = (*pedge_list_)[i];
		WF_edge *e2 = e1->ppair_;
		if (i < e2->ID() && e1->Layer() < max_layer)
		{
			is_export_vert[e2->pvert_->ID()] = true;
			is_export_vert[e1->pvert_->ID()] = true;

			point u = e2->pvert_->RenderPos();
			point v = e1->pvert_->RenderPos();

			fprintf(fp_e, "%lf %lf %lf %lf %lf %lf\n",
				u.x(), u.y(), u.z(), v.x(), v.y(), v.z());
		}
	}

	for (int i = 0; i < N; i++)
	{
		if (is_export_vert[i])
		{
			point u = (*pvert_list_)[i]->RenderPos();
			fprintf(fp_v, "%lf %lf %lf\n", u.x(), u.y(), u.z());
		}
	}

	fclose(fp_v);
	fclose(fp_e);
}


WF_vert* WireFrame::InsertVertex(Vec3f p)
{
	// detect duplication
	int N = SizeOfVertList();
	for (int i = 0; i < N; i++)
	{
		if (Dist(p, (*pvert_list_)[i]->Position()) < 1e-3)
		{
			return ( *pvert_list_)[i];
		}
	}

	WF_vert *vert = new WF_vert(p);
	pvert_list_->push_back(vert);
	return vert;
}


WF_edge* WireFrame::InsertEdge(WF_vert *u, WF_vert *v)
{
	// detect duplication
	WF_edge *e = u->pedge_;
	while (e != NULL)
	{
		if (e->pvert_ == v)
		{
			return e;
		}
		e = e->pnext_;
	}

	WF_edge *e1 = InsertOneWayEdge(u, v);
	WF_edge *e2 = InsertOneWayEdge(v, u);
	if (e1 != NULL)
	{
		e1->ppair_ = e2;
		e2->ppair_ = e1;
	}
	return e1;
}


WF_edge* WireFrame::InsertOneWayEdge(WF_vert *u, WF_vert *v)
{
	if (u == v)
	{
		return NULL;
	}

	WF_edge *edge = new WF_edge();
	edge->pvert_ = v;
	edge->pnext_ = u->pedge_;
	u->pedge_ = edge;
	u->IncreaseDegree();
	
	pedge_list_->push_back(edge);
	return edge;
}


void WireFrame::InsertFace(vector<WF_vert*>	&bound_points)
{
	WF_face *f = new WF_face();

	int N = bound_points.size();
	for (int i = 0; i < N; i++)
	{
		f->bound_points_->push_back(bound_points[i]);
	}

	pface_list_->push_back(f);
}


void WireFrame::Unify()
{
	maxx_ = -1e20;
	maxy_ = -1e20;
	maxz_ = -1e20;
	minx_ = 1e20;
	miny_ = 1e20;
	minz_ = 1e20;
	base_ = 1e20;

	fixed_vert_ = 0;
	pillar_size_ = 0;
	ceiling_size_ = 0;
	max_layer_ = -1;

	int N = SizeOfVertList();
	for (int i = 0; i < N; i++)
	{
		(*pvert_list_)[i]->SetID(i);
		point p = (*pvert_list_)[i]->Position();

		if (!(*pvert_list_)[i]->isFixed())
		{
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
		else
		{
			if (p.z() < base_)
			{
				base_ = p.z();
			}
		}
	}

	if (base_ > minz_)
	{
		base_ = minz_;
	}

	int M = SizeOfEdgeList();
	for (int i = 0; i < M; i++)
	{
		(*pedge_list_)[i]->SetID(i);
		if ((*pedge_list_)[i]->isPillar())
		{
			pillar_size_++;
		}
		if ((*pedge_list_)[i]->isCeiling())
		{
			ceiling_size_++;
		}
		if ((*pedge_list_)[i]->Layer() > max_layer_)
		{
			max_layer_ = (*pedge_list_)[i]->Layer();
		}
	}
	max_layer_++;

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
		if ((*pvert_list_)[i]->isFixed())
		{
			fixed_vert_++;
		}
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


void WireFrame::RefineFrame()
{
	int M = SizeOfEdgeList();
	vector<WF_vert*> split(M);
	for (int i = 0; i < M; i++)
	{
		split[i] = NULL;
	}

	int F = SizeOfFaceList();
	vector<WF_face*>	*pface_list = pface_list_;
	vector<WF_vert*>	center_points(F);
	vector<vector<WF_edge*>>	bound_edges(F);
	vector<vector<double>>		length(F);
	pface_list_ = new vector<WF_face*>;

	for (int i = 0; i < F; i++)
	{
		vector<WF_vert*> bound_points = *((*pface_list)[i]->bound_points_);
		int Fn = bound_points.size();
		double l = 0;
		point center = point(0.0, 0.0, 0.0);
		for (int j = 0; j < Fn; j++)
		{
			center += bound_points[j]->Position();

			int k = j + 1;
			if (k >= Fn)
			{
				k -= Fn;
			}
			WF_edge *e = bound_points[j]->pedge_;
			while (e != NULL)
			{
				if (e->pvert_ == bound_points[k])
				{
					break;
				}
				e = e->pnext_;
			}
			bound_edges[i].push_back(e);
			l += e->Length();

			if (bound_points[k]->Degree() > 2)
			{
				length[i].push_back(l);
				l = 0;
			}
		}

		center /= Fn;
		center_points[i] = InsertVertex(center);
	}

	for (int i = 0; i < F; i++)
	{
		vector<WF_vert*> bound_points = *((*pface_list)[i]->bound_points_);
		int Fn = bound_points.size();
		int Bi = 0;
		int Bn = length[i].size();						// number of boundary

		vector<WF_vert*> cur_bound;
		WF_vert *first_part = NULL;
		WF_vert *first_stop = NULL;
		WF_vert *center_p = center_points[i];

		double sum_l = 0;
		for (int j = 0; j < Fn; j++)
		{
			double l = length[i][Bi];
			double half_l = l / 2;
			double tol_l = l / 2;

			WF_edge *e = bound_edges[i][j];
			WF_vert *u = e->ppair_->pvert_;
			WF_vert *v = e->pvert_;
			sum_l += e->Length();
			cur_bound.push_back(u);

			WF_vert *partition = NULL;
			if (half_l <= sum_l)
			{
				if (sum_l - half_l < tol_l)				// tolerence
				{
					if (half_l - sum_l + e->Length() > sum_l - half_l)
					{
						partition = v;
						cur_bound.push_back(v);
					}
					else
					{
						partition = u;
					}
				}
				else
				{
					point p = (u->Position() + v->Position()) * (float)0.5;
					WF_vert *new_p = InsertVertex(p);

					split[e->ID()] = split[e->ppair_->ID()] = new_p;
					partition = new_p;
					cur_bound.push_back(new_p);
				}

				cur_bound.push_back(center_p);

				if (first_part == NULL)
				{
					first_part = partition;
					first_stop = u;
				}
				else
				{
					InsertFace(cur_bound);
				}

				cur_bound.clear();
				if (partition != v)
				{
					cur_bound.push_back(partition);
				}
				while (j < Fn - 1 && bound_points[j + 1]->Degree() <= 2)
				{
					j++;
					cur_bound.push_back(bound_points[j]);
				}

				Bi++;
				sum_l = 0;
				InsertEdge(center_p, partition);
			}
		}

		for (int j = 0; j < Fn; j++)
		{
			cur_bound.push_back(bound_points[j]);
			if (bound_points[j] == first_stop)
			{
				if (first_stop != first_part)
				{
					cur_bound.push_back(first_part);
				}
				break;
			}
		}
		cur_bound.push_back(center_p);
		InsertFace(cur_bound);
	}

	for (int i = 0; i < F; i++)
	{
		vector<WF_vert*> bound_points = *((*pface_list)[i]->bound_points_);
		int Fn = bound_edges[i].size();
		int Pi = 0;
		for (int j = 0; j < Fn; j++)
		{
			WF_edge *e = bound_edges[i][j];
			if (split[e->ID()] != NULL)
			{
				WF_vert *u = e->ppair_->pvert_;
				WF_vert *v = e->pvert_;
				WF_vert *new_p = split[e->ID()];
				split[e->ID()] = split[e->ppair_->ID()] = NULL;

				WF_edge *new_e1 = InsertOneWayEdge(new_p, u);
				WF_edge *new_e2 = InsertOneWayEdge(new_p, v);
				if (new_e1 != NULL && new_e2 != NULL)
				{
					e->pvert_ = new_p;
					e->ppair_->pvert_ = new_p;
					new_e1->ppair_ = e;
					new_e2->ppair_ = e->ppair_;
					e->ppair_->ppair_ = new_e2;
					e->ppair_ = new_e1;
				}
			}
		}
	}

	Unify();

	for (int i = 0; i < F; i++)
	{
		delete (*pface_list)[i];
		(*pface_list)[i] = NULL;
	}
	delete pface_list;
}


void WireFrame::ProjectBound(vector<WF_vert*> &bound, double len)
{
	int Nb = bound.size();
	for (int i = 0; i < Nb; i++)
	{
		WF_vert *u = bound[i]; 

		point v_pos = u->Position();
		v_pos.z() = minz_ - len;

		WF_vert *v = InsertVertex(v_pos);
		v->SetFixed(true);

		WF_edge *e = InsertEdge(u, v);
		e->SetPillar(true);
		e->ppair_->SetPillar(true);
	}
	
	Unify();
	//InsertEdge(N, SizeOfVertList() - 1);
	//UpdateFrame();
}


void WireFrame::ModifyProjection(double len)
{
	if (SizeOfFixedVert() == 0)
	{
		return;
	}

	int N = SizeOfVertList();
	for (int i = 0; i < N; i++)
	{
		WF_vert *u = (*pvert_list_)[i];
		if (u->isFixed())
		{
			double x = u->Position().x();
			double y = u->Position().y();
			double z = u->Position().z();
			z += u->pedge_->Length();
			z -= len;
			u->SetPosition(x, y, z);
		}
	}
	Unify();
}


void WireFrame::MakeCeiling(vector<WF_edge*> &bound)
{
	int Mb = bound.size();
	for (int i = 0; i < Mb; i++)
	{
		WF_edge *e = bound[i];
		e->SetCeiling(true);
		e->ppair_->SetCeiling(true);
	}

	Unify();
}
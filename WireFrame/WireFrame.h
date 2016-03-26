#pragma once

#include <vector>
#include <map>
#include <cmath>
#include "WireFrame\Vec.h"

using namespace std;
using trimesh::vec;
using trimesh::point;

typedef trimesh::point point;
typedef trimesh::vec3  Vec3f;
typedef trimesh::vec4  Vec4f;


class WF_vert;
class WF_edge;

class WF_vert
{
public:
	WF_vert()
		: pedge_(NULL), id_(0), degree_(0), fixed_(0)
	{}
	WF_vert(Vec3f p)
		: pedge_(NULL), position_(p), render_pos_(p), 
		id_(0), degree_(0), fixed_(0)
	{}
	WF_vert(double x, double y, double z)
		: pedge_(NULL), position_(point(x, y, z)), render_pos_(point(x, y, z)), 
		id_(0), degree_(0), fixed_(0)
	{}
	~WF_vert(){}

public:
	point		Position()	{ return position_; }
	point		RenderPos()	{ return render_pos_; }
	int			ID()		{ return id_; }
	int			Degree()	{ return degree_; }
	bool		isFixed()	{ return fixed_; }

	void		SetPosition(point p)						{ position_ = p; }
	void		SetPosition(double x, double y, double z)	{ position_ = point(x, y, z); }
	void		SetRenderPos(point p)						{ render_pos_ = p; }
	void		SetRenderPos(double x, double y, double z)	{ render_pos_ = point(x, y, z); }
	void		SetID(int id)								{ id_ = id; }
	void		IncreaseDegree()							{ degree_++; }
	void		SetFixed(bool fixed)						{ fixed_ = fixed; }

public:
	WF_edge		*pedge_;

private:
	point		position_;
	point		render_pos_;
	int			id_;
	int			degree_;
	bool		fixed_;
};


class WF_edge
{
public:
	WF_edge()
		:pvert_(NULL), pnext_(NULL), ppair_(NULL), 
		id_(0), layer_(-1), pillar_(false), ceiling_(false)
	{}
	~WF_edge(){}

public:
	int			ID()			{ return id_; }
	int			Layer()			{ return layer_; }
	bool		isPillar()		{ return pillar_; }
	bool		isCeiling()		{ return ceiling_; }

	void		SetID(int id)				{ id_ = id; }
	void		SetLayer(int layer)			{ layer_ = layer; }
	void		SetPillar(bool pillar)		{ pillar_ = pillar; }
	void		SetCeiling(bool ceiling)	{ ceiling_ = ceiling; }

	point CenterPos()
	{
		point u = pvert_->Position();
		point v = ppair_->pvert_->Position();
		return point((u.x() + v.x()) / 2, (u.y() + v.y()) / 2, (u.z() + v.z()) / 2);
	}

	double Length()
	{
		point u = pvert_->Position();
		point v = ppair_->pvert_->Position();
		double dx = u.x() - v.x();
		double dy = u.y() - v.y();
		double dz = u.z() - v.z();
		return sqrt(dx*dx + dy*dy + dz*dz);
	}

public:
	WF_vert		*pvert_;
	WF_edge		*pnext_;
	WF_edge		*ppair_;

private:
	int			id_;
	int			layer_;
	bool		pillar_;
	bool		ceiling_;
};


class WF_face
{
public:
	WF_face()	{ bound_points_ = new vector<WF_vert*>; }
	~WF_face()	{ delete bound_points_; }

public:
	vector<WF_vert*>	*bound_points_;
};


class WireFrame
{
public:
	WireFrame();
	WireFrame(vector<int> *bound);
	~WireFrame();

public:
	void		LoadFromOBJ(const char *path);
	void		LoadFromPWF(const char *path);
	void		WriteToOBJ(const char *path);
	void		WriteToPWF(bool bVert, bool bLine, 
							bool bBase, bool bCeiling, bool bCut,
							int min_layer, int max_layer, const char *path);

	void		ImportFrom3DD(const char *path);

	void		ExportPoints(int min_layer, int max_layer, const char *path);
	void		ExportLines(int min_layer, int max_layer, const char *path);

	WF_vert*	InsertVertex(const Vec3f p);
	WF_edge*	InsertEdge(WF_vert *u, WF_vert *v);
	WF_edge*	InsertOneWayEdge(WF_vert *u, WF_vert *v);

	void		Unify();
	point		Unify(Vec3f p);

	void		SimplifyFrame();
	void		ProjectBound(vector<WF_vert*> &bound, double len);
	void		ModifyProjection(double len);
	void		MakeCeiling(vector<WF_edge*> &bound);

	inline int					SizeOfVertList()		{ return pvert_list_->size(); }
	inline int					SizeOfEdgeList()		{ return pedge_list_->size(); }
	inline int					SizeOfFixedVert()		{ return fixed_vert_; }
	inline int					SizeOfPillar()			{ return pillar_size_; }
	inline int					SizeOfCeiling()			{ return ceiling_size_; }

	inline vector<WF_vert*>		*GetVertList()			{ return pvert_list_; }
	inline vector<WF_edge*>		*GetEdgeList()			{ return pedge_list_; }
	inline WF_vert				*GetVert(int u)			{ return (u >= SizeOfVertList() || u < 0) ? NULL : (*pvert_list_)[u]; }
	inline WF_edge				*GetEdge(int i)			{ return (i >= SizeOfEdgeList() || i < 0) ? NULL : (*pedge_list_)[i]; }
	inline WF_edge				*GetNeighborEdge(int u)	{ return (u >= SizeOfVertList() || u < 0) ? NULL : (*pvert_list_)[u]->pedge_; }

	inline point		GetPosition(int u)			{ return (u >= SizeOfVertList() || u < 0) ? NULL : (*pvert_list_)[u]->Position(); }
	inline int			GetDegree(int u)			{ return (u >= SizeOfVertList() || u < 0) ? NULL : (*pvert_list_)[u]->Degree(); }

	inline int			GetEndu(int i)				{ return (i >= SizeOfEdgeList() || i < 0) ? NULL : (*pedge_list_)[i]->ppair_->pvert_->ID(); }
	inline int			GetEndv(int i)				{ return (i >= SizeOfEdgeList() || i < 0) ? NULL : (*pedge_list_)[i]->pvert_->ID(); }
	inline point		GetCenterPos(int i)			{ return (i >= SizeOfEdgeList() || i < 0) ? NULL : (*pedge_list_)[i]->CenterPos(); }

	inline bool			isFixed(int u)				{ return (u >= SizeOfVertList() || u < 0) ? NULL : (*pvert_list_)[u]->isFixed(); }
	inline bool			isPillar(int i)				{ return (i >= SizeOfEdgeList() || i < 0) ? NULL : (*pedge_list_)[i]->isPillar(); }

	inline void 		SetMaxLayer(int max_layer)	{ max_layer_ = max_layer; }
	inline int			GetMaxLayer()				{ return max_layer_; }

	inline double		maxX()		{ return maxx_; }
	inline double		minX()		{ return minx_; }
	inline double		maxY()		{ return maxy_; }
	inline double		minY()		{ return miny_; }
	inline double		maxZ()		{ return maxz_; }
	inline double		minZ()		{ return minz_; }
	inline double		Base()		{ return base_; }

	inline double Norm(point u)
	{
		return sqrt(u.x()*u.x() + u.y()*u.y() + u.z()*u.z());
	}

	inline double Dist(point u, point v)
	{
		double dx = u.x() - v.x();
		double dy = u.y() - v.y();
		double dz = u.z() - v.z();
		return sqrt(dx*dx + dy*dy + dz*dz);
	}

	inline point CrossProduct(point u, point v)
	{
		return point(u.y() * v.z() - u.z() * v.y(), u.z() * v.x() - u.x() * v.z(),
			u.x() * v.y() - u.y() * v.x());
	}

	inline double ArcHeight(point u, point v1, point v2)
	{
		point alpha = u - v1;
		point beta = v2 - v1;

		return Norm(CrossProduct(alpha, beta)) / Norm(beta);
	}

private:
	vector<WF_vert*>	*pvert_list_;
	vector<WF_edge*>	*pedge_list_;

	int					fixed_vert_;
	int					pillar_size_;
	int					ceiling_size_;
	int					max_layer_;

	double				maxx_;
	double				maxy_;
	double				maxz_;
	double				minx_;
	double				miny_;
	double				minz_;
	double				base_;

	Vec3f				center_pos_;
	float				scaleV_;
	double				unify_size_;
	double				delta_tol_;
};


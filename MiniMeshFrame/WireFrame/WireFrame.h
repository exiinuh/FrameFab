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
		: pedge_(NULL), position_(p), render_pos_(p), id_(0), degree_(0), fixed_(0)
	{}
	WF_vert(double x, double y, double z)
		: pedge_(NULL), position_(point(x, y, z)), render_pos_(point(x, y, z)), id_(0), degree_(0), fixed_(0)
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
		:pvert_(NULL), pnext_(NULL), ppair_(NULL), id_(0), pillar_(false), support_(false)
	{}
	~WF_edge(){}

public:
	int			ID()			{ return id_; }
	bool		isPillar()		{ return pillar_; }
	bool		isSupport()		{ return support_; }

	void		SetID(int id)				{ id_ = id; }
	void		SetPillar(bool pillar)		{ pillar_ = pillar; }
	void		SetSupport(bool support)	{ support_ = support; }

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
	bool		pillar_;
	bool		support_;
};


class WF_face
{
public:
	WF_face(){ bound_points_ = new vector<WF_vert*>; }
	~WF_face(){ delete bound_points_; }

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
	void		WriteToOBJ(const char *path);
	void		ExportPoints(const char *path);
	void		ExportLines(const char *path);

	WF_vert*	InsertVertex(const Vec3f p);
	void		InsertEdge(WF_vert *u, WF_vert *v);
	WF_edge*	InsertOneWayEdge(WF_vert *u, WF_vert *v);
	void		InsertFace(vector<WF_vert*>	&bound_points);
	void		InsertModifiedFace(vector<WF_vert*>	&bound_points);

	void		InsertSupport(int u);
	void		InsertSupport(int u, int v);

	void		Unify();
	point		Unify(Vec3f p);

	void		SimplifyFrame();
	void		RefineFrame();
	void		ProjectBound(vector<int> *bound, double len);
	void		ModifyProjection(double len);

	inline int					SizeOfVertList()		{ return pvert_list_->size(); }
	inline int					SizeOfEdgeList()		{ return pedge_list_->size(); }
	inline int					SizeOfFaceList()		{ return pface_list_->size(); }
	inline int					SizeOfFixedVert()		{ return fixed_vert_; }

	inline vector<WF_vert*>		*GetVertList()			{ return pvert_list_; }
	inline vector<WF_edge*>		*GetEdgeList()			{ return pedge_list_; }
	inline WF_vert				*GetVert(int u)			{ return (u >= SizeOfVertList() || u < 0) ? NULL : (*pvert_list_)[u]; }
	inline WF_edge				*GetEdge(int i)			{ return (i >= SizeOfEdgeList() || i < 0) ? NULL : (*pedge_list_)[i]; }
	inline WF_edge				*GetNeighborEdge(int u)	{ return (u >= SizeOfVertList() || u < 0) ? NULL : (*pvert_list_)[u]->pedge_; }

	inline point				GetPosition(int u)	{ return (u >= SizeOfVertList() || u < 0) ? NULL : (*pvert_list_)[u]->Position(); }
	inline int					GetDegree(int u)	{ return (u >= SizeOfVertList() || u < 0) ? NULL : (*pvert_list_)[u]->Degree(); }

	inline int					GetEndu(int i)		{ return (i >= SizeOfEdgeList() || i < 0) ? NULL : (*pedge_list_)[i]->ppair_->pvert_->ID(); }
	inline int					GetEndv(int i)		{ return (i >= SizeOfEdgeList() || i < 0) ? NULL : (*pedge_list_)[i]->pvert_->ID(); }

	inline bool					isFixed(int u)		{ return (u >= SizeOfVertList() || u < 0) ? NULL : (*pvert_list_)[u]->isFixed(); }
	inline bool					isPillar(int i)		{ return (i >= SizeOfEdgeList() || i < 0) ? NULL : (*pedge_list_)[i]->isPillar(); }

	inline double				maxX()	{ return maxx_; }
	inline double				minX()	{ return minx_; }
	inline double				maxY()	{ return maxy_; }
	inline double				minY()	{ return miny_; }
	inline double				maxZ()	{ return maxz_; }
	inline double				minZ()	{ return minz_; }

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
	vector<WF_face*>	*pface_list_;

	double				maxx_;
	double				maxy_;
	double				maxz_;
	double				minx_;
	double				miny_;
	double				minz_;

	int					fixed_vert_;
	Vec3f				center_pos_;
	float				scaleV_;
	double				unify_size_;
	double				delta_tol_;
};


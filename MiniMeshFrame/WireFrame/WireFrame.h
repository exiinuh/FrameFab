#pragma once

#include <vector>
#include <map>
#include "Vec.h"

using namespace std;
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
	~WF_vert(){}

public:
	point		Position(){ return position_; }
	point		RenderPos(){ return render_pos_; }
	int			ID(){ return id_; }
	int			Degree(){ return degree_; }
	bool		IsFixed(){ return fixed_; }

	void		SetPosition(point p){ position_ = p; }
	void		SetPosition(double x, double y, double z){ position_ = point(x, y, z); }
	void		SetRenderPos(point p){ render_pos_ = p; }
	void		SetRenderPos(double x, double y, double z){ render_pos_ = point(x, y, z); }
	void		SetID(int id){ id_ = id; }
	void		IncreaseDegree(){ degree_++; }
	void		SetFixed(){ fixed_ = true; }

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
		:pvert_(NULL), pnext_(NULL), ppair_(NULL), id_(0)
	{}
	~WF_edge(){}

public:
	int			ID(){ return id_; }
	void		SetID(int id){ id_ = id; }

public:
	WF_vert		*pvert_;
	WF_edge		*pnext_;
	WF_edge		*ppair_;

private:
	int			id_;
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

	WF_vert*	InsertVertex(const Vec3f p);
	void		InsertEdge(int u, int v); 
	void		InsertOneWayEdge(WF_vert *u, WF_vert *v);
	void		UpdateFrame();
	void		Unify(double size);

	void		SimplifyFrame();
	void		ProjectBound(vector<int> *bound);

	inline int					SizeOfVertList(){ return pvert_list_->size(); }
	inline int					SizeOfEdgeList(){ return pedge_list_->size(); }
	inline vector<WF_vert*>		*GetVertList(){ return pvert_list_; }
	inline vector<WF_edge*>		*GetEdgeList(){ return pedge_list_; }
	inline WF_edge				*GetEdge(int i){ return (i >= SizeOfEdgeList() || i < 0) ? NULL : (*pedge_list_)[i]; }
	
	inline double				maxX(){ return maxx_; }
	inline double				minX(){ return minx_; }
	inline double				maxY(){ return maxy_; }
	inline double				minY(){ return miny_; }
	inline double				maxZ(){ return maxz_; }
	inline double				minZ(){ return minz_; }

	inline double				Norm(point u){ return sqrt(u.x()*u.x() + u.y()*u.y() + u.z()*u.z()); }
	inline double				Dist(point u, point v){ return sqrt((u.x() - v.x()) * (u.x() - v.x()) 
																+ (u.y() - v.y()) * (u.y() - v.y())
																+ (u.z() - v.z()) * (u.z() - v.z())); }
	inline point				Cross(point u, point v){ return point(u.y() * v.z() - u.z() * v.y(),
																		u.z() * v.x() - u.x() * v.z(),
																			u.x() * v.y() - u.y() * v.x()); }

private:
	vector<WF_vert*>	*pvert_list_;
	vector<WF_edge*>	*pedge_list_;

	double				maxx_;
	double				maxy_;
	double				maxz_;
	double				minx_;
	double				miny_;
	double				minz_;

	double				delta_tol_;
};


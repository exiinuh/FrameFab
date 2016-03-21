#pragma once
#include <iostream>
#include <Eigen/Core>

#include "WireFrame\WireFrame.h"

using namespace std;
using namespace Eigen;


class DualVertex
{
public:
	DualVertex(){ orig_id_ = -1; dual_id_ = -1; }
	~DualVertex(){}

public:
	void		SetOrigId(int orig_id)	{ orig_id_ = orig_id; }
	void		SetDualId(int dual_id)	{ dual_id_ = dual_id; }
	void		SetHeight(double height){ height_ = height; }

	inline int		orig_id()	const { return orig_id_; }
	inline int		dual_id()	const { return dual_id_; }
	inline double	Height()	const { return height_; }

private:
	int		orig_id_;										// indexed by dual edge id
	int		dual_id_;										// indexed by original edge id
	double	height_;										// z of central points on this original edge
															// indexed by dual id
};


class DualEdge
{
public:
	DualEdge(){}
	DualEdge(int u, int v, double w)
	{
		u_ = u;
		v_ = v;
		w_ = w;
	}
	~DualEdge(){}

public:
	inline double	w()			const { return w_; }
	inline int		u()			const { return u_; }
	inline int		v()			const { return v_; }

private:
	int			u_;
	int			v_;
	double		w_;											// weight on edge in orig graph
};


class DualFace
{
public:
	DualFace(){ orig_id_ = -1; dual_id_ = -1; }
	~DualFace(){}

public:
	void		SetOrigId(int orig_id)	{ orig_id_ = orig_id; }
	void		SetDualId(int dual_id)	{ dual_id_ = dual_id; }

	inline int	orig_id()				const { return orig_id_; }
	inline int	dual_id()				const { return dual_id_; }

private:
	int		orig_id_;										// indexed by dual vertex id
	int		dual_id_;										// indexed by original vertex id
};


class DualGraph
{
public:
	DualGraph();
	DualGraph(WireFrame *ptr_frame);
	~DualGraph();

public:

	void	Dualization();									// dualization on the whole frame

	void	UpdateDualization(VectorXd *ptr_x);				// update from graphcut
	void	Establish();

	void	UpdateDualization(WF_edge *e);					// insert a trail edge ei from frame 
	void	RemoveUpdation(WF_edge *e);						// remove the trail edge

	void	InsertVertex(WF_edge *e);
	void	InsertEdge(WF_edge *e1, WF_edge *e2, double w);
	void	InsertFace(WF_vert *p);
	void	DeleteVertex(WF_edge *e);
	void	DeleteFace(WF_vert *p);

	vector<DualVertex*>		*GetVertList()		{ return vert_list_; }
	vector<DualEdge*>		*GetEdgeList()		{ return edge_list_; }
	vector<DualFace*>		*GetFaceList()		{ return face_list_; }

	int		SizeOfVertList()	{ return Nd_; }
	int		SizeOfEdgeList()	{ return Md_; }
	int		SizeOfFaceList()	{ return Fd_; }
    int     SizeOfFreeFace()    { return Fd_free_; }

	int		u(int ei)			{ return (*edge_list_)[ei]->u(); }
	int		v(int ei)			{ return (*edge_list_)[ei]->v(); }
	int		e_orig_id(int u)	{ return (*vert_list_)[u]->orig_id(); }
	int		e_dual_id(int u)	{ return (*vert_list_)[u]->dual_id(); }
	int		v_orig_id(int i)	{ return (*face_list_)[i]->orig_id(); }
	int		v_dual_id(int i)	{ return (*face_list_)[i]->dual_id(); }

	double	Weight(int ei)		{ return (*edge_list_)[ei]->w(); }
	double	Height(int ei)		{ return (*vert_list_)[ei]->Height(); }
	double	maxZ()				{ return maxz_; }
	double	minZ()				{ return minz_; }

	bool isExistingVert(int u)	{ return (exist_vert_[u] > 0); }
	bool isExistingEdge(int ei)	{ return exist_edge_[ei]; }

	bool isAdjacent(int i, int j)
	{
		WF_edge *e1 = ptr_frame_->GetEdge(e_orig_id(i));
		WF_edge *e2 = ptr_frame_->GetEdge(e_orig_id(j));
		int u1 = e1->ppair_->pvert_->ID();
		int v1 = e1->pvert_->ID();
		int u2 = e2->ppair_->pvert_->ID();
		int v2 = e2->pvert_->ID();

		if (u1 == u2 || u1 == v2 || v1 == u2 || v1 == v2)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	void	Debug();

public:
	WireFrame				*ptr_frame_;

private:
	vector<DualEdge*>		*edge_list_;				// dual edge: original edge -> original edge
	vector<DualVertex*>		*vert_list_;				// dual vert: original edge
	vector<DualFace*>		*face_list_;				// dual face: original vert

	vector<int>				exist_vert_;				// indexed by original id
	vector<bool>			exist_edge_;				// indexed by original id

	int						Nd_;
	int						Md_;
	int						Fd_;
    int                     Fd_free_;

	double					maxz_;
	double					minz_;
};

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
	/* dualization on the whole frame */
	void	Dualization();

	/* update from graphcut */
	void	UpdateDualization(VectorXd *ptr_x);

	void	Establish();

	/* insert a trail edge ei from frame */
	void	UpdateDualization(int ei);

	/* remove the trail edge */
	void	RemoveUpdation(int ei);

	vector<DualVertex*>		*GetVertList()		{ return vert_list_; }
	vector<DualEdge*>		*GetEdgeList()		{ return edge_list_; }
	vector<DualFace*>		*GetFaceList()		{ return face_list_; }

	int		SizeOfVertList()	{ return Nd_; }
	int		SizeOfEdgeList()	{ return Md_; }
	int		SizeOfFaceList()	{ return Fd_; }

	int		u(int ei)			{ return (*edge_list_)[ei]->u(); }
	int		v(int ei)			{ return (*edge_list_)[ei]->v(); }
	int		e_orig_id(int u)	{ return (*vert_list_)[u]->orig_id(); }
	int		e_dual_id(int u)	{ return (*vert_list_)[u]->dual_id(); }
	int		v_orig_id(int i)	{ return (*face_list_)[i]->orig_id(); }
	int		v_dual_id(int i)	{ return (*face_list_)[i]->dual_id(); }
	double	Weight(int ei)		{ return (*edge_list_)[ei]->w(); }
	double	Height(int ei)		{ return (*vert_list_)[ei]->Height(); }
	
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
	//void					Debug();

public:
	WireFrame				*ptr_frame_;

private:
	vector<DualEdge*>		*edge_list_;				// dual edge: original edge -> original edge
	vector<DualVertex*>		*vert_list_;				// dual vert: original edge
	vector<DualFace*>		*face_list_;				// dual face: original vert

	vector<bool>			exist_vert_;
	vector<bool>			exist_edge_;

	int						Nd_;
	int						Md_;
	int						Fd_;

	double					maxz_;
	double					minz_;
};

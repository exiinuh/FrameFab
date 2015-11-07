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

	inline int	orig_id()				const { return orig_id_; }
	inline int	dual_id()				const { return dual_id_; }

private:
	int		orig_id_;										// indexed by dual edge id
	int		dual_id_;										// indexed by original edge id
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
	void					Dualization();
	void					Dualization(VectorXd *ptr_x);
	void					Establish();

	int						SizeOfVertList()	{ return Nd_; }
	int						SizeOfEdgeList()	{ return Md_; }
	int						SizeOfFaceList()	{ return Fd_; }

	vector<DualVertex*>		*GetVertList()		{ return vert_list_; }
	vector<DualEdge*>		*GetEdgeList()		{ return edge_list_; }
	vector<DualFace*>		*GetFaceList()		{ return face_list_; }

	int						u(int ei)			{ return (*edge_list_)[ei]->u(); }
	int						v(int ei)			{ return (*edge_list_)[ei]->v(); }
	int						e_orig_id(int ei)	{ return (*vert_list_)[ei]->orig_id(); }
	int						e_dual_id(int ei)	{ return (*vert_list_)[ei]->dual_id(); }
	int						v_orig_id(int i)	{ return (*face_list_)[i]->orig_id(); }
	int						v_dual_id(int i)	{ return (*face_list_)[i]->dual_id(); }

	void					Debug();

private:
	int						Nd_;
	int						Md_;
	int						Fd_;

	vector<DualEdge*>		*edge_list_;				// dual edge: original edge -> original edge
	vector<DualVertex*>		*vert_list_;				// dual vert: original edge
	vector<DualFace*>		*face_list_;				// dual face: original vert
	WireFrame				*ptr_frame_;

	vector<bool>			exist_vert_;
	vector<bool>			exist_edge_;
};

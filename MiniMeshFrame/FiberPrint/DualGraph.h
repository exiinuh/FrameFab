#pragma once
#include <iostream>
#include "WireFrame\WireFrame.h"

using namespace std;


class DualVertex
{
public:
	DualVertex(){ org_id_ = -1; dual_id_ = -1; }
	~DualVertex(){}

public:
	void		SetOrgId(int org_id)	{ org_id_ = org_id; }
	void		SetDualId(int dual_id)	{ dual_id_ = dual_id; }

	inline int	original_id()			const { return org_id_; }
	inline int	dual_id()				const { return dual_id_; }

private:
	int		org_id_;										// indexed by dual graph id
	int		dual_id_;										// indexed by original graph id
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


class DualGraph
{
public:
	DualGraph();
	DualGraph(WireFrame *ptr_frame);
	~DualGraph();

public:
	void					Dualization();

	int						num_of_edge_list()	{ return edge_list_->size(); }
	vector<DualEdge*>		*get_edge_list()	{ return edge_list_; }
	vector<DualVertex*>		*get_vert_list()	{ return vert_list_; }
	int						u(int ei)			{ return (*edge_list_)[ei]->u(); }
	int						v(int ei)			{ return (*edge_list_)[ei]->v(); }
	int						org_id(int ei)		{ return (*vert_list_)[ei]->original_id(); }
	int						dual_id(int ei)		{ return (*vert_list_)[ei]->dual_id(); }

	void					Debug();

private:
	vector<DualEdge*>		*edge_list_;
	vector<DualVertex*>		*vert_list_;
	WireFrame				*ptr_frame_;
};

/*
* ==========================================================================
*
*       class: Collision
*
*    Description:  Collision detection for tool path, obtaining feasible 
*				   orientation range for extruder
*
*	 Version:  1.0
*	 Created:  Oct/20/2015
*    Update :  Dec/08/2015
*
*	 Author:   Guoxian Song, Xin Hu, Yijiang Huang
*	 Company:  GCL@USTC
*	 Note:     This file uses mathematical part of Geometric Tools Engine, 
*			   a library of source code for computing in the fields of 
*			   mathematics, graphics, image analysis, and physics.
*			   For more info, please refer to http://www.geometrictools.com/index.html
*			  
* ==========================================================================
*/
#pragma once
#include <iostream>
#include <vector>
#include <math.h>

#include "WireFrame\WireFrame.h"
#include "FiberPrint\DualGraph.h"
#include "Geometry.h"
#include "Stiffness\GCommon.h"

#include "CommonBulk.h"
#include "SpecialBulk.h"
#include "BaseBulk.h"
#include "CylinderBulk.h"


typedef Geometry::Vector3d GeoV3;
typedef unsigned long long lld;


class Collision
{
public:
	Collision();
	Collision(WireFrame *ptr_frame);
	~Collision();

	//---------------------------add Guoxian 1220----------------------------------
	// aim:  new collision model, input  WF_frame, vector<int> order (existing structure), int target_id  edge, output double cost,( the larger cost, the more difficult for printing)
	// vector<int> order and target_id is for orginal frame index
	// vector<GeoV3>normal: a discreted range 
	// notice: can't compute cost for pillar
public:
	void		DetectCollision(WF_edge *target_e, DualGraph *ptr_subgraph);
	void		DetectCollision(WF_edge *target_e, WF_edge *order_e);

private:
	void		DetectEdge(WF_edge *order_e);
	void		CreatePrintTable();
	void		GenerateSampleNormal();

	double		DisSegSeg(point start, point end, point target_start, point target_end);
	double		DisSegPoint(point start, point end, point target);

	bool		IsColVec(GeoV3 start, GeoV3 end, GeoV3 target);
	bool		IsColTable(GeoV3 target);

	GeoV3		ColAngle(point target, point start, point end);
	bool		IsInside(point a);

	gte::Segment<3, float>		Segement_(point target_start, point target_end);
	vector<point>				Consider(point order_start, point order_end, 
										point target_start, point target_end);

public:
	lld		Angle()				{ return angle_; }
	int		Divide()			{ return divide_; }
	int		ColFreeAngle()
	{
		int colfree_angle = 0;
		for (int i = 0; i < divide_; i++)
		{
			lld mask = (1 << i);
			if (!(mask&angle_))
			{
				colfree_angle++;
			}
		}
		return colfree_angle;
	}

	int		ColFreeAngle(lld angle)
	{
		int colfree_angle = 0;
		for (int i = 0; i < divide_; i++)
		{
			lld mask = (1 << i);
			if (!(mask&angle))
			{
				colfree_angle++;
			}
		}
		return colfree_angle;
	}

public:
	/* Interface Data Structure*/
	WireFrame			*ptr_frame_;
	WF_edge				*target_e_;

private:
	lld					angle_;
	int					divide_;
	ExtruderCone		extruder_;
	vector<GeoV3>		normal_;							// normal list
	gte::Plane3<float>	table_;

	bool				considerable_;
};
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


class Collision
{
public:
	Collision();
	Collision(WireFrame *ptr_frame, WF_edge *target_e);
	~Collision();

	//---------------------------add Guoxian 1220----------------------------------
	// aim:  new collision model, input  WF_frame, vector<int> order (existing structure), int target_id  edge, output double cost,( the larger cost, the more difficult for printing)
	// vector<int> order and target_id is for orginal frame index
	// vector<GeoV3>normal: a discreted range 
	// notice: can't compute cost for pillar
public:
	void		DetectCollision(DualGraph *ptr_subgraph);
	 void		DetectCollision(WF_edge *order_e);
	 
	 void		CreatePrintTable();
	 void		GenerateSampleNormal();

	 double  Cost(vector<GeoV3> reduce);

	 double		DisSegSeg(point start, point end, point target_start, point target_end);
	 double		DisSegPoint(point start, point end, point target);

	 bool		IsColVec(GeoV3 start, GeoV3 end, GeoV3 target);
	 bool		IsColTable(GeoV3 target);

	 GeoV3		ColAngle(point target, point start, point end);
	 gte::Segment<3, float>		Segement_(point target_start, point target_end);

	 int		Divide()			{ return divide_; }
	 int		AvailableAngle()	{ return normal_.size(); }

private:
	/* Interface Data Structure*/
	WireFrame		*ptr_frame_;
	ExtruderCone	*ptr_extruder_;

	int					divide_;
	gte::Plane3<float>	table_;

	WF_edge				*target_e_;
	vector<GeoV3>		normal_;							// normal list
};
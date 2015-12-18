/*
* ==========================================================================
*
*       class: SequenceAnalyzer
*
*    Description:  Collision geometric solid created for collision detect submodule
*				   This inherit class is specified for collision algo. Ver2.0.
*	 Version:  1.0
*	 Created:  Dec/08/2015
*    Update :  Dec/08/2015
*
*	 Author:   Guoxian Song
*	 Company:  GCL@USTC
*	 Note:	   This file use data structure from Geometric Tools Engine, 
*			   a library of source code for computing in the fields of 
*			   mathematics, graphics, image analysis, and physics.
*			   More info please refer to http://www.geometrictools.com/index.html
*	 LOG:	   Ver2.0: Use Cylinder to warp the printing edge, and collision is 
*	judged by the distance between printing edge and candidate edge
* ==========================================================================
*/
#pragma once
#include"Collision\BaseBulk.h"

typedef struct RAngle
{
	GeoV3 u;
	GeoV3 v;
};

class CylinderBulk : public BaseBulk
{
public:
	CylinderBulk(){}
	~CylinderBulk(){}

	/*
	* Main function: Read edge's info data and modify		Nov/08/2015
	*			     feasible range and is_collision
	* @param: start : printing edge's start point, cylinder Bulk generated at this edge
	* @param: end   : printing edge's end   point
	* @param: target_start : collision detect edge's start point 
	* @param: end_point    : collision detect edge's end   point
	*/
	CylinderBulk(point start, point end);
	CylinderBulk(point start, point end, point target_start, point tartget_end);

	void Init();

	gte::Cylinder3<float>	Cylinder_(point a, point b);
	gte::Line<3, float>		CyLine(point a, point b);
	gte::Vector3<float>		Point_(point p);

	bool	IsColCyl(point target, point end);
	bool	IfParalet(point p);
	GeoV3	ColAngle(point target);


	double DisSegPoint(point start, point end, point target);

	bool IfConsider(point start, point end, point target);

public:
	/* Geometric Data from Geometric Tool Engine(gte) */
	gte::Segment3<float> target_edge_;

	/* Interface Data Structure*/
	ExtruderCone  extruder_;

	/* Output Data */
	double range_;
	bool   is_collision_;

	RAngle Rangle_;

	/* Printing edge endpoints */
	point start_;
	point end_;
	gte::Cylinder3<float> cylinder_;
};
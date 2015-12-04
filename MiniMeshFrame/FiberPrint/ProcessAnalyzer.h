#pragma once

#include"FiberPrint\FiberPrintPlugIn.h"
#include"WireFrame\WireFrame.h"
#define STOREPATH "I:\\Users\\Alienware\\Desktop\\Result\\"


typedef  struct Process
{
    bool extruder_state_;
	bool fan_state_;
	point start_;
	point end_;

	// -1 for move; 0 for quick move; 1 for sweep;
	//2 for up; 3 for down; 4 for flat;5 for vertical; 6 for base; 
	//7 fusion up
	 int move_state_; 
	point vector;
};


class  ProcessAnalyzer
{
public:
	ProcessAnalyzer(FiberPrintPlugIn	*ptr_fiberprint);


	ProcessAnalyzer(WireFrame * frame);


	vector<Process*> * Init();

 
	/*generate process start point and end point, based on the prior process;
	  follow the rule:

	  Must:
	  1. angle(normal, z) <extruder angle, so must up print; 
	  2.  e start point must base on existing structure; 

	  Optimization:

	  1. if e has a point same to  the prior process's end_ point, so start point = prior end point.
	  2.if e has a point same to one of  the later order edge point, so the end point = the point.
	*/
	Process* SetPoint(WF_edge *e, int id);


	/* Fan depend on the relationship bewteen prior and current process:
	  mainly check existing point
	  1. fusion don;t need fan
	  2. branch need fan.


	*/
	Process* SetFan( Process* temp);


	//Different move for different speed, angle
	Process* SetExtruderSpeed(Process* temp,int id);



	// If print is  not continuous, consider break; quick up the break string 
	void SetBreak(Process* temp);

	//Consider string thick
	void SetThick();

	vector<Process*> *print_;
	FiberPrintPlugIn	*ptr_fiberprint_;
	vector<point> exist_point_;

	bool IfPointInVector(point p);
	ExtruderCone extruder_;
	double break_height_;

	void Write();
	
};
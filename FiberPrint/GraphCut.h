#pragma once

#include <iostream>

#include "WireFrame\WireFrame.h"
#include "Stiffness\Stiffness.h"
#include "Collision\QuadricCollision.h"
#include "FiberPrintPARM.h"

using namespace std;


class GraphCut
{
public:
	GraphCut();
	virtual ~GraphCut();

public:
	virtual void	MakeLayers();	
	virtual void	PrintOutTimer();

public:
	WireFrame		*ptr_frame_;
	char			*ptr_path_;


	vector<int>		cutting_edge_;


protected:
	/* for debuging */
	bool			debug_;

};


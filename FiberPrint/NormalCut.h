#pragma once

#include <map>

#include "GraphCut.h"

using namespace std;


class NormalCut : public GraphCut
{
public:
	NormalCut();
	NormalCut(WireFrame *ptr_frame, char *ptr_path);
	~NormalCut();

public:
	void	MakeLayers();

private:
	multimap<double, WF_edge*>	sweep_queue_;
};


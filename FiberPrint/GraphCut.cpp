#include "GraphCut.h"


GraphCut::GraphCut()
{
}


GraphCut::~GraphCut()
{
	delete ptr_dualgraph_;
	ptr_dualgraph_ = NULL;

	delete ptr_stiff_;
	ptr_stiff_ = NULL;
}


void GraphCut::MakeLayers()
{
}


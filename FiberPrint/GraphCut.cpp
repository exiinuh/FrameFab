#include "GraphCut.h"


GraphCut::GraphCut()
{
	ptr_dualgraph_ = NULL;
	ptr_stiff_ = NULL;
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


void GraphCut::PrintOutTimer()
{
}

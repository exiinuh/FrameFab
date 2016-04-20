#include "GraphCut.h"


GraphCut::GraphCut()
{
	ptr_frame_ = NULL;
	ptr_dualgraph_ = NULL;
	ptr_stiffness_ = NULL;
	ptr_collision_ = NULL;
	ptr_path_ = NULL;

	debug_ = false;
}


GraphCut::GraphCut(WireFrame *ptr_frame, char *ptr_path)
{
	ptr_frame_ = ptr_frame;
	ptr_dualgraph_ = NULL;
	ptr_stiffness_ = NULL;
	ptr_collision_ = NULL;
	ptr_path_ = ptr_path;

	debug_ = true;
}


GraphCut::~GraphCut()
{
}


void GraphCut::MakeLayers()
{
}


void GraphCut::PrintOutTimer()
{
}

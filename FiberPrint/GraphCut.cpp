#include "GraphCut.h"


GraphCut::GraphCut()
{
	ptr_frame_ = NULL;
	ptr_dualgraph_ = NULL;
	ptr_stiff_ = NULL;
	ptr_collision_ = NULL;
}


GraphCut::GraphCut(WireFrame *ptr_frame, char *ptr_path)
{
	ptr_frame_ = ptr_frame;
	ptr_dualgraph_ = new DualGraph(ptr_frame_);
	ptr_stiff_ = NULL;
	ptr_collision_ = new QuadricCollision(ptr_frame_);
	ptr_path_ = ptr_path;

	debug_ = true;
}


GraphCut::~GraphCut()
{
	delete ptr_dualgraph_;
	ptr_dualgraph_ = NULL;

	delete ptr_stiff_;
	ptr_stiff_ = NULL;

	delete ptr_collision_;
	ptr_collision_ = NULL;
}


void GraphCut::MakeLayers()
{
}


void GraphCut::PrintOutTimer()
{
}

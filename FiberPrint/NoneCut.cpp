#include "NoneCut.h"


NoneCut::NoneCut()
{
}


NoneCut::NoneCut(WireFrame *ptr_frame, FiberPrintPARM *ptr_parm, char *path)
{
	ptr_frame_ = ptr_frame;
	ptr_dualgraph_ = new DualGraph(ptr_frame_);

	path_ = path;
	debug_ = true;
}


NoneCut::~NoneCut()
{
}

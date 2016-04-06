#include "NoneCut.h"


NoneCut::NoneCut()
{
}


NoneCut::NoneCut(WireFrame *ptr_frame, FiberPrintPARM *ptr_parm, char *ptr_path)
{
	ptr_frame_ = ptr_frame;
	ptr_dualgraph_ = new DualGraph(ptr_frame_);

	ptr_path_ = ptr_path;
	debug_ = true;
}


NoneCut::~NoneCut()
{
}

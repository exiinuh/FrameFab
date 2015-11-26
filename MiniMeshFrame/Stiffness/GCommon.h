/*
* ==========================================================================
*
*       file: GCommon.h
*
*    Description:  GCommon.h provides some common configuration for fiberprint project.
*
*	 Version:  1.0
*	 Created:  Nov/25/2015
*
*	 Author:   Yijiang Huang, Xin Hu, Guoxian Song
*	 Company:  GCL@USTC
* ==========================================================================
*/

#ifndef FIBERPRINT_COMMON_H
#define FIBERPRINT_COMMON_H

#define FRAME3DD_PATHMAX 512
#ifndef MAXL
#define MAXL    512
#endif

#define FILENMAX 128

#ifndef VERSION
#define VERSION "20151125+"
#endif

#ifndef F_PI
#define F_PI 3.14159265358979323846264338327950288419716939937510
#endif

// Zvert=1: Z axis is vertical... rotate about Y-axis, then rotate about Z-axis
// Zvert=0: Y axis is vertical... rotate about Z-axis, then rotate about Y-axis
#define Zvert 1	

#endif /* FIBERPRINT_COMMON_H */


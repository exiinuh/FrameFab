#pragma once

#include "SeqAnalyzer.h"

class BFAnalyzer : public SeqAnalyzer
{
public:
	BFAnalyzer();
	BFAnalyzer(
		DualGraph			*ptr_dualgraph,
		QuadricCollision	*ptr_collision,
		Stiffness			*ptr_stiffness,
		FiberPrintPARM		*ptr_parm,
		char				*ptr_path
		)
		:SeqAnalyzer(ptr_dualgraph, ptr_collision, ptr_stiffness,
		ptr_parm, ptr_path){}
	~BFAnalyzer();

public:
	bool	SeqPrint();

private:
	bool	GenerateSeq(int h, int t);
	void	PrintOutQueue(int N);	

public:
	void	PrintOutTimer();

private:
	Timer	BF_analyzer_;
};


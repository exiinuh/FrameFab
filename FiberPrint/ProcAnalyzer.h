#pragma once
#include "SeqAnalyzer.h"


using namespace std;


typedef  struct Process
{
	bool	fan_state_;
	point	start_;
	point	end_;
	std::vector<GeoV3>  normal_;
};


class ProcAnalyzer
{
public:
	ProcAnalyzer();
	ProcAnalyzer(SeqAnalyzer *seqanalyzer, char *path);
	~ProcAnalyzer();

public:
	void		ProcPrint();

private:
	void		ReadLayerQueue();
	void		Write();
	bool		IfPointInVector(point p);

private:
	SeqAnalyzer			*ptr_seqanalyzer_;
	QuadricCollision	*ptr_collision_;
	char				*path_;
	vector<int>			layer_queue_;
	vector<point>		exist_point_;
	vector<WF_edge*>	exist_edge_;
	ExtruderCone		extruder_;

	vector<Process>		process_list_;
	bool				debug_;
	int					support_;
};


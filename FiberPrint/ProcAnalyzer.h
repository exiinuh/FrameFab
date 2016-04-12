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
	void     CollisionColorMap();
	void      CollisionColorMap(int x);
private:
	void		ReadLayerQueue();
	void		Write();
	bool		IfPointInVector(point p);

	

private:
	SeqAnalyzer			*ptr_seqanalyzer_;
	char				*path_;

	vector<int>			layer_queue_;
	vector<point>		exist_point_;
	vector<WF_edge*>	exist_edge_;
	ExtruderCone		extruder_;

	vector<Process>		process_list_;
	bool				debug_;
	int					support_;
<<<<<<< HEAD

	bool IfCoOrientation(GeoV3 a, vector<GeoV3> &b);
	void CheckProcess(Process &a);



=======
>>>>>>> a11a85ba0639cf40e03412dad89f5cab97bae04c
};


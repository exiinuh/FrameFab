#pragma once
#include <cmath>
#include <cstring>

#include "SeqAnalyzer.h"


class FFAnalyzer : public SeqAnalyzer
{
public:
	typedef Eigen::MatrixXd MX;
	typedef Eigen::Matrix3d M3;
	typedef Eigen::VectorXd VX;
	typedef Eigen::Vector3d V3;

public:
	FFAnalyzer();
	FFAnalyzer(
		DualGraph			*ptr_dualgraph,
		QuadricCollision	*ptr_collision,
		Stiffness			*ptr_stiffness,
		FiberPrintPARM		*ptr_parm,
		char				*ptr_path
		)
		:SeqAnalyzer(ptr_dualgraph, ptr_collision, ptr_stiffness,
		ptr_parm, ptr_path){}
	~FFAnalyzer();

public:
	bool			SeqPrint();

private:
	bool			GenerateSeq(int l, int h, int t);
	double			GenerateCost(WF_edge *ei, WF_edge *ej);

public:
	void			PrintOutTimer();
	void			WriteRenderPath(int min_layer, int max_layer, char *ptr_path);

private:
	vector<vector<WF_edge*>>	layers_;			// store dual_node's id for each layers

	double			min_z_;
	double			max_z_;


	Timer			FF_analyzer_;
};


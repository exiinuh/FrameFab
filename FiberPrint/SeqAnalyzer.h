#pragma once
#include <cmath>

#include "ADMMCut.h"
#include "NormalCut.h"
#include "Collision\QuadricCollision.h"
#include "Collision\ResolveAngle.h"


class SeqAnalyzer
{
public:
	typedef Eigen::MatrixXd MX;
	typedef Eigen::Matrix3d M3;
	typedef Eigen::VectorXd VX;
	typedef Eigen::Vector3d V3;

public:
	SeqAnalyzer();
	SeqAnalyzer(
		DualGraph			*ptr_dualgraph,
		QuadricCollision	*ptr_collision,
		Stiffness			*ptr_stiffness,
		FiberPrintPARM		*ptr_parm,
		char				*ptr_path
		);
	virtual ~SeqAnalyzer();

public:
	virtual bool	SeqPrint();
	virtual void	PrintOutTimer();
	virtual void	WriteRenderPath(int min_layer, int max_layer, char *ptr_path);

public:
	void			InputPrintOrder(vector<int> &print_queue);
	void			OutputPrintOrder(vector<WF_edge*> &print_queue);

protected:
	void			Init();

	void			PrintPillars();
	void			UpdateStructure(WF_edge *e);
	void			RecoverStructure(WF_edge *e);
	void			UpdateStateMap(WF_edge *e, vector<vector<lld>> &state_map);
	void			RecoverStateMap(WF_edge *e, vector<vector<lld>> &state_map);
	bool			TestifyStiffness(WF_edge *e);

public:
	WireFrame			*ptr_frame_;
	DualGraph			*ptr_dualgraph_;
	Stiffness			*ptr_stiffness_;
	QuadricCollision	*ptr_collision_;
	char				*ptr_path_;

protected:
	/* maintaining for sequence */
	int							Nd_;
	DualGraph					*ptr_wholegraph_;
	vector<WF_edge*>			print_queue_;
	vector<vector<lld>>			angle_state_;
	VX							D0_;

	/* parameters */
	double				gamma_;						// gamma_	: amplifier factor for adjacency cost
	double				D_tol_;						// Dt_tol	: tolerance of offset in stiffness
	double				Wp_;						// Wp_		: stablity weight for printing cost
	double				Wa_;						// Wa_		: adjacent weight for printing cost
	double				Wi_;						// Wl_		: influence weight for printing cost

	bool				debug_;
	bool				fileout_;

	Timer				upd_struct_;
	Timer				rec_struct_;
	Timer				upd_map_;
	Timer				upd_map_collision_;
	Timer				rec_map_;
	Timer				test_stiff_;
};


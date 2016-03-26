#include "GraphCut.h"


GraphCut::GraphCut()
{
}


GraphCut::~GraphCut()
{
	delete ptr_dualgraph_;
	ptr_dualgraph_ = NULL;

	delete ptr_stiff_;
	ptr_stiff_ = NULL;
}


void GraphCut::MakeLayers()
{
<<<<<<< HEAD
=======
	// Initial Cutting Edge Setting
	InitState();

	debug_ = false;
	int cut_count = 0;
	vector<double> cut_energy;
    vector<double> res_energy;
	do
	{
        /* Recreate dual graph at the beginning of each cut */
		SetStartingPoints(cut_count);
		CreateA();

		ptr_stiff_->CalculateD(D_, x_, 0, 0, cut_count);

		/* for rendering */
		if (cut_count == 0)
		{
			WriteWeight();
			WriteStiffness("offset1.txt", "rotation1.txt");
			getchar();
		}
		if (cut_count == 4)
		{
			WriteStiffness("offset2.txt", "rotation2.txt");
			getchar();
		}
		if (cut_count == 6)
		{
			WriteStiffness("offset3.txt", "rotation3.txt");
			getchar();
		}

        /* set x for intial cut setting */
		SetBoundary();

		/* 
		* energy specify:
		* cut		 energy : | A * x |
		* defomation energy : norm(K D - F)
		*/
		ptr_stiff_->CreateGlobalK(x_);
        ptr_stiff_->CreateF(x_);
		SpMat K_init = *(ptr_stiff_->WeightedK());
		VX	  F_init = *(ptr_stiff_->WeightedF());
	

        double icut_energy = 0;
        VX V_Cut = A_ * x_;
        for (int i = 0; i < Md_; i++)
        {
            icut_energy += abs(V_Cut[i]);
        }
		double ideform_energy = (K_init * D_ - F_init).norm();

		cout << "****************************************" << endl;
		cout << "GraphCut Round : " << cut_count << endl;
		cout << "Initial cut energy before entering ADMM: "        << icut_energy    << endl;
		cout << "Initial Lagrangian energy before entering ADMM: " << ideform_energy << endl;
		cout << "---------------------------------" << endl;

		int rew_count = 0;
		VX x_prev;
		VX D_prev;
        
        /* Output energy list for reweighting process in a single 
           graph cut problem, energy.size() = number of reweighting
           process performed.
           cut energy = |A_ * x_| = sum_{e_ij} w_ij * |x_i - x_j|
           res energy = (K(x)D - F(x)).norm()
        */

        cut_energy.clear();
        res_energy.clear();

        cut_energy.push_back(icut_energy);
        res_energy.push_back(ideform_energy);

		/* ADMM primal residual recording in each reweighting */
		vector<double> ADMM_res;

		do
		{
            /* Reweighting loop for cut */

			int ADMM_count = 0;
			x_prev = x_;
			CreateC(cut_count, rew_count);

			ADMM_res.clear();
			
			do
			{
				cout << "GraphCut Round: " << cut_count << ", reweight iteration:" << rew_count
					<< ", ADMM " << ADMM_count << " iteration." << endl;

				/*-------------------ADMM loop-------------------*/
				CalculateX();

				D_prev = D_;
				CalculateD();

				UpdateLambda();

				/*-------------------Residual Calculation-------------------*/
				SpMat Q_prev;
				SpMat Q_new;
				CalculateQ(D_prev, Q_prev);
				CalculateQ(D_,     Q_new);

                /* Update K reweighted by new x */
				ptr_stiff_->CreateGlobalK(x_);
                ptr_stiff_->CreateF(x_);
				SpMat K_new = *(ptr_stiff_->WeightedK());
                VX    F_new = *(ptr_stiff_->WeightedF());

				dual_res_   = penalty_ * (D_prev - D_).transpose() * K_new.transpose() * Q_prev
					+ lambda_.transpose() * (Q_prev - Q_new);
                primal_res_ = K_new * D_ - F_new;

				/*-------------------Screenplay-------------------*/				
                double new_cut_energy = x_.dot(H1_ * x_);

                cout << "new quadratic func value record: " << new_cut_energy << endl;
                cout << "dual_residual : "                 << dual_res_.norm() << endl;
                cout << "primal_residual(KD-F) : "         << primal_res_.norm() << endl;

                cout << "---------------------" << endl;

				ADMM_res.push_back(primal_res_.norm());

				ADMM_count++;
			} while (!TerminationCriteria(ADMM_count));

            /* One reweighting process ended! */
            /* Output energy and residual */

            double energy = 0;
            VX V_Cut = A_ * x_;
            for (int i = 0; i < Md_; i++)
            {
                energy += ptr_dualgraph_->Weight(i) * abs(V_Cut[i]);
            }

            /*-------------------Screenplay-------------------*/
            double res_tmp = primal_res_.norm();
            cout << "Cut " << cut_count << " Reweight " << rew_count << " completed." << endl;
            cout << "Cut Energy :" << energy  << endl;
            cout << "Res Energy :" << res_tmp << endl;
            cout << "<-<-<-<-<-<-<-<-<-<-<-<-<-<-<-<-<-" << endl;

            cut_energy.push_back(energy);
            res_energy.push_back(res_tmp);

            /* write x distribution to a file */
            //string str_x = "Cut_" + to_string(cut_count) + "_Rew_" + to_string(rew_count) + "_x";
            //Statistics tmp_x(str_x, x_);
            //tmp_x.GenerateVectorFile();

			/* write ADMM primal residual norm to a file*/
			string str_pri_res = "Rew_" + to_string(rew_count) + "_ADMM_PrimalRes_Norm";
			Statistics s_pres(str_pri_res, ADMM_res);
			s_pres.GenerateStdVecFile();

            rew_count++;
        } while (!UpdateR(x_prev, rew_count));

        /* Output reweighting energy history for last cut process */
        string str_eC = "Cut_" + to_string(cut_count) + "_Cut_Energy";
        Statistics s_eC(str_eC, cut_energy);
        s_eC.GenerateStdVecFile();

        string str_eR = "Cut_" + to_string(cut_count) + "_Res_Energy";
        Statistics s_eR(str_eR, res_energy);
        s_eR.GenerateStdVecFile();

        /* Update New Cut information to Rendering (layer_label_) */

		UpdateCut();

        fprintf(stdout, "GraphCut No.%d process is Finished!\n", cut_count);
		cut_count++;
		getchar();

	} while (!CheckLabel(cut_count));
	
	fprintf(stdout, "All done!\n");
}


void GraphCut::CalculateX()
{
	// Construct Hessian Matrix for D-Qp problem
	SpMat Q;
	CalculateQ(D_, Q);

	SpMat H2 = Q.transpose() * Q;
	SpMat H = 2 * H1_ + penalty_ * H2;

	// Construct Linear coefficient for x-Qp problem
	a_ = Q.transpose() * lambda_;
	
	// Inequality constraints A*x <= b
	SpMat A(6 * Fd_, 6 * Fd_);
	A.setZero();
	VX b(6 * Fd_);
	b.setZero();

	// Variable constraints x >= lb, x <= ub
	VX lb(Nd_), ub(Nd_);													
	lb.setZero();
	ub.setOnes();

	qp_->solve(H, a_, A, b, W_, d_, lb, ub, x_, NULL, NULL, debug_);
}


void GraphCut::CalculateQ(const VX _D, SpMat &Q)
{
	// Construct Hessian Matrix for D-Qp problem
	Q.resize(6 * Ns_, Nd_);
	vector<Eigen::Triplet<double>> Q_list;

	for (int i = 0; i < Ns_; i++)
	{
		int u = ptr_dualgraph_->v_orig_id(i);
		WF_edge *edge = ptr_frame_->GetNeighborEdge(u);
		
		while (edge != NULL)
		{
			int e_id = ptr_dualgraph_->e_dual_id(edge->ID());
			if (e_id != -1)
			{
				int v = edge->pvert_->ID();
				int j = ptr_dualgraph_->v_dual_id(v);

				MX eKuu = ptr_stiff_->eKv(edge->ID());
				MX eKeu = ptr_stiff_->eKe(edge->ID());
				VX Fe = ptr_stiff_->Fe(edge->ID());
				VX Di(6);
				VX Dj(6);
                
                if (i < Ns_ && j < Ns_)
                {
                    for (int k = 0; k < 6; k++)
                    {
                        Di[k] = _D[6 * i + k];
                        Dj[k] = _D[6 * j + k];
                    }
                }
                else
                {
                    if (i < Ns_)
                    {
                        for (int k = 0; k < 6; k++)
                        {
                            Di[k] = _D[6 * i + k];
                            Dj[k] = 0;
                        }
                    }

                    if (j < Ns_)
                    {
                        for (int k = 0; k < 6; k++)
                        {
                            Di[k] = 0;
                            Dj[k] = _D[6 * j + k];
                        }
                    }
                }
                VX Gamma = eKuu * Di + eKeu * Dj - Fe;

				for (int k = 0; k < 6; k++)
				{
					Q_list.push_back(Triplet<double>(6 * i + k, e_id, Gamma[k]));
				}
			}

			edge = edge->pnext_;
		}
	}

	Q.setFromTriplets(Q_list.begin(), Q_list.end());
}


void GraphCut::CalculateD()
{
	// Construct Hessian Matrix for D-Qp problem
    // Here, K is continuous-x weighted
	ptr_stiff_->CreateGlobalK(x_);
	SpMat K = *(ptr_stiff_->WeightedK());
	SpMat Q = penalty_ * K.transpose() * K;

	// Construct Linear coefficient for D-Qp problem
	ptr_stiff_->CreateF(x_);
	VX F = *(ptr_stiff_->WeightedF());

	VX a = K.transpose() * lambda_ - penalty_ * K.transpose() * F;
	
	/* 10 degree rotation tolerance, from degree to radians */
	qp_->solve(Q, a, D_, Dt_tol_, Dr_tol_, debug_);	
}


void GraphCut::UpdateLambda()
{
	// Recompute K(x_{k+1}) and F(x_{k+1})
	ptr_stiff_->CreateGlobalK(x_);
	SpMat K = *(ptr_stiff_->WeightedK());

	ptr_stiff_->CreateF(x_);
	VX F = *(ptr_stiff_->WeightedF());

	lambda_ = lambda_ + penalty_ * (K * D_ - F);
}


void GraphCut::UpdateCut()
{
	// Convert previous continuous computed result to discrete 1-0 label
	for (int i = 0; i < M_; i++)
	{
		int e_id = ptr_dualgraph_->e_dual_id(i);
		WF_edge *e = ptr_frame_->GetEdge(i);
		int layer = e->Layer();
		if (e_id == -1)
		{
			e->SetLayer(layer + 1);
		}
		else
		{
			if (x_[e_id] >= 0.5)
			{
				x_[e_id] = 1;
			}
			else
			{
				x_[e_id] = 0;
				e->SetLayer(layer + 1);
			}
		}
	}

	// Update cut
	cutting_edge_.clear();
	VectorXd J(Md_);
	J = A_ * x_;
	for (int i = 0; i < Md_; i++)
	{
		if (J[i] == 1 || J[i] == -1)
		{
			int dual_u = ptr_dualgraph_->u(i);
			int dual_v = ptr_dualgraph_->v(i);
			int u = ptr_dualgraph_->e_orig_id(dual_u);
			int v = ptr_dualgraph_->e_orig_id(dual_v);
			if (x_[dual_u] == 1)
			{
				// u is the lower dual vertex of cutting-edge 
				cutting_edge_.push_back(u);
			}
			else
			{
				// v is the lower dual vertex of cutting-edge 
				cutting_edge_.push_back(v);
			}
		}
	}
}


bool GraphCut::UpdateR(VX &x_prev, int count)
{
    double max_improv = 0;
    for (int i = 0; i < Nd_; i++)
    {
        /* if No significant improvment found */
        double improv = fabs(x_prev[i] - x_[i]) / x_prev[i];
        if (improv > max_improv)
        {
            max_improv = improv;
        }
    }

    cout << "---UpdateR Rew Check---"     << endl;
    cout << "Reweighting Process No."     << count-1    << endl;
    cout << "Max Relative Improvement = " << max_improv << endl;
    cout << "---" << endl;

    vector<DualEdge*> dual_edge = *(GetDualEdgeList());

	for (int i = 0; i < Md_; i++)
	{
		int    u = ptr_dualgraph_->u(i);
		int    v = ptr_dualgraph_->v(i);
        double w = dual_edge[i]->w();
>>>>>>> 2c719846f6006b0658c93a4bf28bf6ac0236a416


}


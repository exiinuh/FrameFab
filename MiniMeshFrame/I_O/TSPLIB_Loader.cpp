#include "TSPLIB_Loader.h"

bool TSPLIB_Loader::loadFromFile(char* filename, int &N, SpMat *CostMatrix)
{
	// N		  : number of cities
	// CostMatrix : a N*N cost matrix as described in symmeric TSP problem

	FILE *tspfile;
	char sbuf[21];
	std::cout << filename << std::endl;

	tspfile = fopen(filename, "r");
	
	assert(tspfile && CostMatrix);

	do
	{
		if (1 != fscanf(tspfile, "%20s ", sbuf))
		{
			return false;
		}
	} while (strncmp(sbuf, "DIMENSION", 9) != 0);
	

	if (1 != fscanf(tspfile, "%d ", &N))
	{
		return false;
	}

	CostMatrix->resize(N, N);
	std::vector<Eigen::Triplet<double>> Cost_list;

	do
	{
		if (1 != fscanf(tspfile, "%20s ", sbuf))
		{
			return false;
		}
	}
	while (strncmp(sbuf, "EDGE_WEIGHT_TYPE", 16) != 0);
	
	std::cout << sbuf << std::endl;

	if (1 != fscanf(tspfile, "%20s ", sbuf))
	{
		return false;
	}

	if (strcmp(sbuf, "EXPLICIT") == 0)
	{
		do
		{
			if (1 != fscanf(tspfile, "%20s ", sbuf))
			{
				return false;
			}
		} while (strncmp(sbuf, "EDGE_WEIGHT_FORMAT", 18) != 0);
		
		if (1 != fscanf(tspfile, "%20s ", sbuf))
		{
			return false;
		}

		if (strcmp(sbuf, "FULL_MATRIX") == 0)
		{
			int* cost;
			int ij, n2;

			do
			{
				if (1 != fscanf(tspfile, "%20s ", sbuf))
				{
					return NULL;
				}
			} while (strncmp(sbuf, "EDGE_WEIGHT_SECTION", 19) != 0);
			
			n2 = N;
			n2 *= n2;
			
			//cost = (int*)malloc(n2*sizeof(int));
			//assert(cost);
			
			for (ij = 0; ij < n2; ij++)
			{
				int i = (ij - (ij % N))/N;
				int j = ij % N;
				int c;

				int ScanFlag = fscanf(tspfile, "%d ", &c);
				
				if (1 != ScanFlag)
				{
					//free(cost);
					return false;
				}
				else
				{
					Cost_list.push_back(Eigen::Triplet<double>(i, j, c));
				}
			}
			CostMatrix->setFromTriplets(Cost_list.begin(), Cost_list.end());

			return true;
		}
		else if (strcmp(sbuf, "LOWER_DIAG_ROW") == 0)
		{
			//int* cost;
			int i, j, n;
			
			do
			{
				if (1 != fscanf(tspfile, "%20s ", sbuf))
				{
					return false;
				}
			} while (strncmp(sbuf, "EDGE_WEIGHT_SECTION", 19) != 0);
			
			n = N;
			
			//cost = (int*)malloc(n*n*sizeof(int));
			//assert(cost);
			
			for (i = 0; i < n; i++)
			{
				for (j = 0; j <= i; j++)
				{
					int c;
					if (1 != fscanf(tspfile, "%d ", &c))
					{
						//free(cost);
						return false;
					}
					Cost_list.push_back(Eigen::Triplet<double>(i, j, c));
					Cost_list.push_back(Eigen::Triplet<double>(j, i, c));
					//cost[IJ(i, j)] = c;
					//cost[IJ(j, i)] = c;
				}
			}
			CostMatrix->setFromTriplets(Cost_list.begin(), Cost_list.end());
			return true;
		}
		else
		{
			printf("Format not supported\n");
			assert(false);
			return false;
		}
	}
	else if (strcmp(sbuf, "EUC_2D") == 0)
	{
		int* cost;
		double *xcoord, *ycoord;
		int i, j, n;
		
		do
		{
			if (1 != fscanf(tspfile, "%20s ", sbuf))
			{
				return NULL;
			}
		} while (strncmp(sbuf, "NODE_COORD_SECTION", 18) != 0);
		
		n = N;
		xcoord = (double*)malloc(n*sizeof(double));
		ycoord = (double*)malloc(n*sizeof(double));
		cost = (int*)malloc(n*n*sizeof(int));
		
		assert(xcoord); 
		assert(ycoord); 
		assert(cost);
		
		for (i = 0; i < n; i++)
		{
			int dummy;
			if (3 != fscanf(tspfile, "%d %lf %lf ", &dummy, &xcoord[i], &ycoord[i]))
			{
				//free(cost);
				return false;
			}
		}
		for (i = 0; i < n; i++)
		{
			for (j = 0; j < n; j++)
			{
				double xd = xcoord[i] - xcoord[j];
				double yd = ycoord[i] - ycoord[j];
				Cost_list.push_back(Eigen::Triplet<double>(i, j, 0.5 + sqrt(xd*xd + yd*yd)));
				//cost[IJ(i, j)] = (int)(0.5 + sqrt(xd*xd + yd*yd));
			}
		}

		CostMatrix->setFromTriplets(Cost_list.begin(), Cost_list.end());
		return true;
	}
	else
	{
		printf("E_W_Type not supported\n");
		return false;
	}
}
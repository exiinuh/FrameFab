#include "StiffnessIO.h"
/*
* StiffnessGetline :
* get line into a character string
*/
int	StiffnessIO::StiffnessGetline(FILE *fp, char *s, int lim)
{
	int c = 0, i = 0;

	while (--lim > 0 && (c = getc(fp)) != EOF && c != '\n')
		s[i++] = c;

	s[i] = '\0';
	return i;
}

/*
* GetlineNoCommen :
* get a line into a character string. from K&R
* get the line only up to one of the following characters:  \n  %  #  ?
* ignore all comma (,) characters
* ignore all double quote (") characters
* ignore all semi-colon (;) characters
*/
void StiffnessIO::GetlineNoComment(
	FILE *fp,   /**< pointer to the file from which to read */
	char *s,    /**< pointer to the string to which to write */
	int lim    /**< the longest anticipated line length  */)
{
	int     c = 0, i = 0;

	while (--lim > 0 && (c = getc(fp)) != EOF 
		&& c != '\n' && c != '%' && c != '#' 
		&& c != '?') 
	{
		if (c != ',' && c != '"' && c != ';')
			s[i++] = c;
		else
			s[i++] = ' ';
	}

	s[i] = '\0';
	
	if (c != '\n')
	{
		while (--lim > 0 && (c = getc(fp)) != EOF && c != '\n')
		{
			/* read the rest of the line, otherwise do nothing */;
		}
	}
	
	if (c == EOF) s[0] = EOF;
	
	return;
}

/*
* ParseInput
* strip comments from the input file, and write a stripped input file
*/
void StiffnessIO::ParseInput(FILE *fp, const char *tpath)
{
	FILE	*fpc;		/* stripped input file pointer	*/
	char	line[256];
	char	errMsg[MAXL];

	fpc = fopen(tpath, "w");

	if (fpc == NULL) 
	{
		sprintf(errMsg, "\n  error: cannot open parsed input data file: '%s' \n", tpath);
		errorMsg(errMsg);
		exit(12);
	}

	//fprintf(fpc, "%d", 111);
	//int ff;
	//rewind(fpc);
	//fscanf(fpc, "%d", &ff);
	//MYOUT << ff << MYEND;

	do 
	{
		GetlineNoComment(fp, line, 256);
		fprintf(fpc, "%s \n", line);
	} while (line[0] != '_' && line[0] != EOF);

	fclose(fpc);
}

/*
* PARSE_OPTIONS -  parse command line options
* command line options over-ride values in the input data file
* 04 Mar 2009, 22 Sep 2009
*/
void StiffnessIO::ParseOptions(
	char IN_file[], char OUT_file[],
	int *shear_flag,
	int *geom_flag,
	int *anlyz_flag,
	double *exagg_flag,
	int *D3_flag,
	int *lump_flag,
	int *modal_flag,
	double *tol_flag,
	double *shift_flag,
	float *pan_flag,
	int *write_matrix,
	int *axial_sign,
	int *condense_flag,
	int *verbose,
	int *debug
	)
{
	char	option;
	char	errMsg[MAXL];
	int		sfrv = 0;		/* *scanf return value	*/

	/* default values */

	*shear_flag = *geom_flag = *anlyz_flag = *lump_flag = *modal_flag = -1;
	*exagg_flag = *tol_flag = *shift_flag = -1.0;
	*D3_flag = 0;
	*pan_flag = -1.0;
	*condense_flag = -1;
	*write_matrix = 0;
	*axial_sign = 1;
	*debug = 0; *verbose = 1;

	strcpy(IN_file, "\0");
	strcpy(OUT_file, "\0");

	/* set up file names for the the input data and the output data */

	printf(" Please enter the  input data file name: ");
	//sfrv = scanf("%s", IN_file);
	sprintf(IN_file, "exE.3dd");
	//if (sfrv != 1) sferr("IN_file");

	printf(" Please enter the output data file name: ");
	//sfrv = scanf("%s", OUT_file);
	sprintf(IN_file, "exE.out");

	//if (sfrv != 1) sferr("OUT_file");

	if (strcmp(IN_file, "\0") == 0) {
		fprintf(stderr, " Please enter the  input data file name: ");
		sfrv = scanf("%s", IN_file);
		if (sfrv != 1) sferr("IN_file");
		fprintf(stderr, " Please enter the output data file name: ");
		sfrv = scanf("%s", OUT_file);
		if (sfrv != 1) sferr("OUT_file");
	}
	if (strcmp(IN_file, "\0") != 0 && strcmp(OUT_file, "\0") == 0) {
		strcpy(OUT_file, IN_file);
		strcat(OUT_file, ".out");
	}
	return;
}

/*
* READ_NODE_DATA  -  read node location data
* 04 Jan 2009
*/
void StiffnessIO::ReadNodeData(FILE *fp, int nN, vec3 &xyz, VX &r)
{
	int	i, j,
		sfrv = 0;		/* *scanf return value	*/
	char	errMsg[MAXL];

	for (i = 0; i < nN; i++)
	{	
		/* read node coordinates	*/
		sfrv = fscanf(fp, "%d", &j);
		if (sfrv != 1) sferr("node number in node data");
		if (j <= 0 || j > nN) 
		{
			sprintf(errMsg, "\nERROR: in node coordinate data, node number out of range\n(node number %d is <= 0 or > %d)\n", j, nN);
			errorMsg(errMsg);
			exit(41);
		}
		float r_tmp;
		sfrv = fscanf(fp, "%lf %lf %lf %f", &xyz[j-1][0], &xyz[j-1][1], &xyz[j-1][2], &r_tmp);
		r[j - 1] = r_tmp;

		if (sfrv != 4) sferr("node coordinates in node data");

		fprintf(stderr,"\nj = %d, pos = (%lf, %lf, %lf), r = %f", j-1, xyz[j-1][0], xyz[j-1][1], xyz[j-1][2], r[j-1]);

		r[j-1] = fabs(r[j-1]);
	}
	return;
}

/*
* OUTPUT_PATH
* return path for output files using either current directory, or FRAME3DD_OUTDIR
* if specified. --
*/
void StiffnessIO::OutputPath(const char *fname, char fullpath[], const int len, char *default_outdir) {
	int res;
	assert(fname != NULL);

	/*			deprecated code, January 15 2010 ...
	if ( fname[0]==sep ) {	in Win32 absolute path starts with C:\ not \ ??
	// absolute output path specified
	//		res = snprintf(fullpath,len,"%s",fname);
	res = sprintf(fullpath,"%s",fname);
	} else {
	*/

	//		fprintf(stderr,"Generating output path for file '%s'\n",fname);
	//		res = snprintf(fullpath,len,"%s%c%s",outdir,sep,fname);
	res = sprintf(fullpath, "%s%s", "F:\\FiberPrintProject\\ResultData\\Frame3dd_data\\", fname);

	/*			closing bracket for deprecated code "if"
	}
	*/

	if (res > len) 
	{
		errorMsg("ERROR: unable to construct output filename: overflow.\n");
		exit(16);
	}
	//	printf("Output file path generated: %s\n",fullpath); /* debug */
}

/*
* READ_FRAME_ELEMENT_DATA  -  read frame element property data
* 04 Jan 2009
*/
void StiffnessIO::ReadFrameElementData(
	FILE *fp,					/**< input data file pointer					*/
	int &nN,					/**< number of nodes							*/
	int &nE,					/**< number of frame elements					*/
	std::vector<V3> xyz,		/**< XYZ coordinates of each node				*/
	VX &r,						/**< rigid radius of each node					*/
	VX &L, VX &Le,				/**< length of each frame element, effective	*/
	VXi &N1, VXi &N2, 			/**< node connectivity							*/
	VX &Ax, VX &Asy, VX &Asz,	/**< section areas								*/
	VX &Jx, VX &Iy, VX &Iz,		/**< section inertias							*/
	VX &E, VX &G,				/**< elastic moduli and shear moduli			*/
	VX &p,						/**< roll angle of each frame element (radians)	*/
	VX &d,						/**< mass density of each frame element			*/
	int verbose
	)
{
	int	n1, n2, i, n, b;
	VXi	epn;
	int epn0 = 0;		/* vector of elements per node */
	int	sfrv = 0;		/* *scanf return value */
	char	errMsg[MAXL];

	epn.resize(nN);

	for (n = 0; n < nN; n++)	epn[n] = 0;

	for (i = 0; i < nE; i++) 
	{		
		/* read frame element properties */
		sfrv = fscanf(fp, "%d", &b);
		MYOUT << b << MYEND;

		if (sfrv != 1) sferr("frame element number in element data");
		
		if (b <= 0 || b > nE)
		{
			sprintf(errMsg, "\n  error in frame element property data: Element number out of range  \n Frame element number: %d  \n", b);
			errorMsg(errMsg);
			exit(51);
		}

		int n1_tmp, n2_tmp;
		sfrv = fscanf(fp, "%d %d", &n1_tmp, &n2_tmp);
		N1[b - 1] = n1_tmp;
		N2[b - 1] = n2_tmp;

		epn[N1[b-1]-1] += 1;        epn[N2[b-1]-1] += 1;

		if (sfrv != 2) sferr("node numbers in frame element data");
		
		if (N1[b-1] <= 0 || N1[b-1] > nN || N2[b-1] <= 0 || N2[b-1] > nN) 
		{
			sprintf(errMsg, "\n  error in frame element property data: node number out of range  \n Frame element number: %d \n", b);
			errorMsg(errMsg);
			exit(52);
		}

		float ax_t = 0, asy_t = 0, asz_t = 0;
		sfrv = fscanf(fp, "%f %f %f", &ax_t, &asy_t, &asz_t);
		Ax[b - 1]  = ax_t;
		Asy[b - 1] = asy_t;
		Asz[b - 1] = asz_t;

		if (sfrv != 3) sferr("section areas in frame element data");
		
		float jx_t, iy_t, iz_t;
		sfrv = fscanf(fp, "%f %f %f", &jx_t, &iy_t, &iz_t);
		Jx[b - 1] = jx_t;
		Iy[b - 1] = iy_t;
		Iz[b - 1] = iz_t;

		if (sfrv != 3) sferr("section inertias in frame element data");

		float e_t, g_t;
		sfrv = fscanf(fp, "%f %f", &e_t, &g_t);
		E[b - 1] = e_t;
		G[b - 1] = g_t;

		if (sfrv != 2) sferr("material moduli in frame element data");
		
		float p_t;
		sfrv = fscanf(fp, "%f", &p_t);
		p[b - 1] = p_t;

		if (sfrv != 1) sferr("roll angle in frame element data");

		p[b-1] = p[b-1] * F_PI / 180.0;	/* convert from degrees to radians */

		float d_t;
		sfrv = fscanf(fp, "%f", &d_t);
		d[b - 1] = d_t;

		if (sfrv != 1) sferr("mass density in frame element data");

		if (Ax[b-1] < 0 || Asy[b-1] < 0 || Asz[b-1] < 0 ||
			Jx[b-1] < 0 || Iy[b-1] < 0 || Iz[b-1] < 0) 
		{
			sprintf(errMsg, "\n  error in frame element property data: section property < 0 \n  Frame element number: %d  \n", b);
			errorMsg(errMsg);
			exit(53);
		}

		if (Ax[b-1] == 0) 
		{
			sprintf(errMsg, "\n  error in frame element property data: cross section area is zero   \n  Frame element number: %d  \n", b);
			errorMsg(errMsg);
			exit(54);
		}

		if ((Asy[b-1] == 0 || Asz[b-1] == 0) && G[b-1] == 0) 
		{
			sprintf(errMsg, "\n  error in frame element property data: a shear area and shear modulus are zero   \n  Frame element number: %d  \n", b);
			errorMsg(errMsg);
			exit(55);
		}

		if (Jx[b-1] == 0) 
		{
			sprintf(errMsg, "\n  error in frame element property data: torsional moment of inertia is zero   \n  Frame element number: %d  \n", b);
			errorMsg(errMsg);
			exit(56);
		}

		if (Iy[b-1] == 0 || Iz[b-1] == 0) 
		{
			sprintf(errMsg, "\n  error: cross section bending moment of inertia is zero   \n  Frame element number : %d  \n", b);
			errorMsg(errMsg);
			exit(57);
		}

		if (E[b-1] <= 0 || G[b-1] <= 0) 
		{
			sprintf(errMsg, "\n  error : material elastic modulus E or G is not positive   \n  Frame element number: %d  \n", b);
			errorMsg(errMsg);
			exit(58);
		}

		if (d[b-1] <= 0) 
		{
			sprintf(errMsg, "\n  error : mass density d is not positive   \n  Frame element number: %d  \n", b);
			errorMsg(errMsg);
			exit(59);
		}

		if (verbose)
		{
			printf("b = %d, ( Ax, Asy, Asz, Jxx, Iyy, Izz, E, G, roll, Density) = ( %f, %f, %f, %f, %f, %f, %f, %f, %f, %f)\n",
				b, Ax[b - 1], Asy[b - 1], Asz[b - 1], Jx[b - 1], Iy[b - 1], Iz[b - 1], E[b - 1], G[b - 1], 
				p[b - 1], d[b - 1]);
		}
	}

	for (b = 1; b <= nE; b++) 
	{		/* calculate frame element lengths */
		n1 = N1[b-1];
		n2 = N2[b-1];

#define SQ(X) ((X)*(X))
		L[b-1] = SQ(xyz[n2-1][0] - xyz[n1-1][0]) +
			SQ(xyz[n2-1][1] - xyz[n1-1][1]) +
			SQ(xyz[n2-1][2] - xyz[n1-1][2]);
#undef SQ

		L[b-1] = sqrt(L[b-1]);
		Le[b-1] = L[b-1] - r[n1-1] - r[n2-1];

		if (n1 == n2 || L[b-1] == 0.0) 
		{
			sprintf(errMsg,
				" Frame elements must start and stop at different nodes\n  frame element %d  N1= %d N2= %d L= %e\n   Perhaps frame element number %d has not been specified.\n  or perhaps the Input Data file is missing expected data.\n",
				b, n1, n2, L[b-1], i);
			errorMsg(errMsg);
			exit(60);
		}

		if (Le[b-1] <= 0.0) {
			sprintf(errMsg, " Node  radii are too large.\n  frame element %d  N1= %d N2= %d L= %e \n  r1= %e r2= %e Le= %e \n",
				b, n1, n2, L[b-1], r[n1-1], r[n2-1], Le[b-1]);
			errorMsg(errMsg);
			exit(61);
		}

		if (verbose)
		{
			printf("b = %d, ( L, Le) = ( %f, %f), ( n1, n2) = ( %d, %d)\n", b, L[b - 1], Le[b - 1], n1, n2);
		}

	}

	for (n = 1; n <= nN; n++) 
	{
		if (epn[n-1] == 0) 
		{
			sprintf(errMsg, "node or frame element property data:\n     node number %3d is unconnected. \n", n);
			sferr(errMsg);
			epn0 += 1;
		}
	}

	if (epn0 > 0) exit(42);

	return;
}

/*
* READ_RUN_DATA  -  read information for analysis
* 29 Dec 2008
*/
void StiffnessIO::ReadRunData(
	FILE *fp,			 /**< input data file pointer						*/
	char OUT_file[],	 /**< output data file name							*/
	int &shear,			 /**< 1: include shear deformations, 0: don't		*/
	int &geom,		   	 /**< 1: include geometric stiffness, 0: don't		*/
	char meshpath[],	 /**< file name for mesh data output				*/
	char plotpath[],	 /**< file name for Gnuplot script					*/
	char infcpath[],	 /**< file name for internal force data				*/
	double &exagg_static,/**< factor for static displ. exaggeration			*/
	double &exagg_flag,	 /**< static exagg. command-line over-ride			*/
	float &scale,		 /**< zoom scale for 3D plotting in gnuplot			*/
	float &dx,			 /**< frame element increment for internal forces	*/
	int &anlyz,			 /**< 1: perform elastic analysis, 0: don't			*/
	int debug			 /**< print debugging information					*/
	)
{
	int	full_len = 0, len = 0, i;
	char	base_file[96] = "EMPTY_BASE";
	char	mesh_file[96] = "EMPTY_MESH";
	int	sfrv = 0;		/* *scanf return value */

	strcpy(base_file, OUT_file);
	while (base_file[len++] != '\0')
		/* the length of the base_file */;
	full_len = len;
	while (base_file[len--] != '.' && len > 0)
		/* find the last '.' in base_file */;
	if (len == 0)	len = full_len;
	base_file[++len] = '\0';	/* end base_file at the last '.' */

	// GnuPlot file
	strcpy(plotpath, base_file);
	strcat(plotpath, ".plt");

	// Internal force file
	strcpy(infcpath, base_file);
	strcat(infcpath, ".if");

	while (base_file[len] != '/' && base_file[len] != '\\' && len > 0)
	{
		len--;	/* find the last '/' or '\' in base_file */
	}
			
	i = 0; 
	
	while (base_file[len] != '\0')
	{
		mesh_file[i++] = base_file[len++];
	}
	
	mesh_file[i] = '\0';
	strcat(mesh_file, "-msh");
	OutputPath(mesh_file, meshpath, FRAME3DD_PATHMAX, NULL);

	if (debug) {
		fprintf(stderr, "OUT_FILE  = %s \n", OUT_file);
		fprintf(stderr, "BASE_FILE = %s \n", base_file);
		fprintf(stderr, "PLOTPATH  = %s \n", plotpath);
		fprintf(stderr, "MESH_FILE = %s \n", mesh_file);
		fprintf(stderr, "MESHPATH  = %s \n", meshpath);
	}

	sfrv = fscanf(fp, "%d %d %lf %f %f", shear, geom, exagg_static, scale, dx);
	if (sfrv != 5) sferr("shear, geom, exagg_static, scale, or dx variables");

	if (*shear != 0 && *shear != 1) {
		errorMsg(" Rember to specify shear deformations with a 0 or a 1 \n after the frame element property info.\n");
		exit(71);
	}

	if (*geom != 0 && *geom != 1) {
		errorMsg(" Rember to specify geometric stiffness with a 0 or a 1 \n after the frame element property info.\n");
		exit(72);
	}

	if (*exagg_static < 0.0) {
		errorMsg(" Remember to specify an exageration factor greater than zero.\n");
		exit(73);
	}

	if (*dx <= 0.0 && *dx != -1) {
		errorMsg(" Remember to specify a frame element increment greater than zero.\n");
		exit(74);
	}


	/* over-ride values from input data file with command-line options */
	if (shear_flag != -1)	*shear = shear_flag;
	if (geom_flag != -1)	*geom = geom_flag;
	if (exagg_flag != -1.0)	*exagg_static = exagg_flag;
	if (anlyz_flag != -1.0)	*anlyz = anlyz_flag;


	return;
}

/*
* READ_REACTION_DATA - Read fixed node displacement boundary conditions
* 29 Dec 2009
*/
void StiffnessIO::ReadReactionData(
	FILE *fp, int DoF, int nN, int &nR, VXi &q, VXi &r, int &sumR, int verbose
	)
{
	int		i, j, l;
	int		sfrv = 0;		/* *scanf return value */
	char	errMsg[MAXL];

	r.setZero();

	sfrv = fscanf(fp, "%d", &nR);	/* read restrained degrees of freedom */
	if (sfrv != 1) sferr("number of reactions in reaction data");

	if (verbose) 
	{
		fprintf(stdout, " number of nodes with reactions ");
		dots(stdout, 21);

		// Number of reaction node
		fprintf(stdout, " nR =%4d ", nR);
	}

	if (nR < 0 || nR > DoF / 6) 
	{
		fprintf(stderr, " number of nodes with reactions ");
		dots(stderr, 21);
		fprintf(stderr, " nR = %3d \n", nR);
		sprintf(errMsg, "\n  error: valid ranges for nR is 0 ... %d \n", DoF / 6);
		errorMsg(errMsg);
		exit(80);
	}

	for (i = 0; i < nR; i++)
	{
		sfrv = fscanf(fp, "%d", &j);
		if (sfrv != 1) sferr("node number in reaction data");

		for (l = 5; l >= 0; l--)
		{
			int r_tmp;
			sfrv = fscanf(fp, "%d", &r_tmp);
			r[6 * j - l - 1] = r_tmp;

			if (sfrv != 1) sferr("reaction value in reaction data");

			if (j > nN)
			{
				sprintf(errMsg, "\n  error in reaction data: node number %d is greater than the number of nodes, %d \n", j, nN);
				errorMsg(errMsg);
				exit(81);
			}

			if (r[6 * j - l - 1] != 0 && r[6 * j - l - 1] != 1)
			{
				sprintf(errMsg, "\n  error in reaction data: Reaction data must be 0 or 1\n   Data for node %d, DoF %d is %d\n", j, 6 - l, r[6 * j - l - 1]);
				errorMsg(errMsg);
				exit(82);
			}
		}
		
		if (verbose)
		{
			printf("j = %d, ", j);
			for (int k = 0; k < 6; k++)
			{
				printf("( x, y, z, xx, yy, zz) = ( %d, %d, %d, %d, %d, %d )\n", r[6 * (j - 1)], r[6 * (j - 1) + 1], r[6 * (j - 1)+2]
					, r[6 * (j - 1) + 3], r[6 * (j - 1) + 4], r[6 * (j - 1)+5]);
			}
		}
		sumR = 0;
		for (l = 5; l >= 0; l--) 	sumR += r[6 * j - l - 1];
		if (sumR == 0) 
		{
			sprintf(errMsg, "\n  error: node %3d has no reactions\n   Remove node %3d from the list of reactions\n   and set nR to %3d \n",
				j, j, nR - 1);
			errorMsg(errMsg);
			exit(83);
		}
	}

	sumR = 0;	
	for (i = 0; i < DoF; i++)
	{
		sumR += r[i];
	}

	if (sumR < 4) {
		sprintf(errMsg, 
			"\n  Warning:  un-restrained structure   %d imposed reactions.\n  At least 4 reactions are required to support static loads.\n"
			, sumR);
		errorMsg(errMsg);
		/*	exit(84); */
	}

	if (sumR >= DoF) {
		sprintf(errMsg, "\n  error in reaction data:  Fully restrained structure\n   %d imposed reactions >= %d degrees of freedom\n"
			, sumR, DoF);
		errorMsg(errMsg);
		exit(85);
	}

	for (i = 0; i < DoF; i++)
	{
		if (r[i])
		{
			q[i] = 0;
		}
		else
		{
			q[i] = 1;
		}
	}

	return;
}

///*
//* READ_AND_ASSEMBLE_LOADS
//* Read load information data, assemble load vectors in global coordinates
//* Returns vector of equivalent loadal forces F_temp and F_mech and
//* a matrix of equivalent element end forces eqF_temp and eqF_mech from
//* distributed internal and temperature loadings.
//* eqF_temp and eqF_mech are computed for the global coordinate system
//* 2008-09-09, 2015-05-15
//*/
//void ReadAndAssembleLoads(
//	FILE *fp,
//	int nN, int nE, int nL, int DoF,
//	vec3 *xyz,
//	double *L, double *Le,
//	int *J1, int *J2,
//	float *Ax, float *Asy, float *Asz,
//	float *Iy, float *Iz, float *E, float *G,
//	float *p,
//	float *d, float *gX, float *gY, float *gZ,
//	int *r,
//	int shear,
//	int *nF, int *nU, int *nW, int *nP, int *nT, int *nD,
//	double **Q,
//	double **F_temp, double **F_mech, double *Fo,
//	float ***U, float ***W, float ***P, float ***T, float **Dp,
//	double ***eqF_mech, // equivalent mech loads, global coord 
//	double ***eqF_temp, // equivalent temp loads, global coord 
//	int verbose
//	){
//	float	hy, hz;			/* section dimensions in local coords */
//
//	float	x1, x2, w1, w2;
//	double	Ln, R1o, R2o, f01, f02;
//
//	/* equivalent element end forces from distributed and thermal loads */
//	double	Nx1, Vy1, Vz1, Mx1 = 0.0, My1 = 0.0, Mz1 = 0.0,
//		Nx2, Vy2, Vz2, Mx2 = 0.0, My2 = 0.0, Mz2 = 0.0;
//	double	Ksy, Ksz, 		/* shear deformatn coefficients	*/
//		a, b,			/* point load locations */
//		t1, t2, t3, t4, t5, t6, t7, t8, t9;	/* 3D coord Xfrm coeffs */
//	int	i, j, l, lc, n, n1, n2;
//	int	sfrv = 0;		/* *scanf return value */
//
//	char	errMsg[MAXL];
//
//	/* initialize load data vectors and matrices to zero */
//	for (j = 1; j <= DoF; j++)
//		Fo[j] = 0.0;
//
//	// mechanism
//	for (j = 1; j <= DoF; j++)
//	for (lc = 1; lc <= nL; lc++)
//		F_temp[lc][j] = F_mech[lc][j] = 0.0;
//
//	// temperature
//	for (i = 1; i <= 12; i++)
//	for (n = 1; n <= nE; n++)
//	for (lc = 1; lc <= nL; lc++)
//		eqF_mech[lc][n][i] = eqF_temp[lc][n][i] = 0.0;
//
//	for (i = 1; i <= DoF; i++)
//	for (lc = 1; lc <= nL; lc++)
//		Dp[lc][i] = 0.0;
//
//	for (i = 1; i <= nE; i++)
//	for (j = 1; j <= 12; j++)
//		Q[i][j] = 0.0;
//
//	for (lc = 1; lc <= nL; lc++)
//	{		/* begin load-case loop */
//
//		if (verbose) {	/*  display the load case number */
//			textColor('y', 'g', 'b', 'x');
//			fprintf(stdout, " load case %d of %d: ", lc, nL);
//			fprintf(stdout, "                                            ");
//			fflush(stdout);
//			color(0);
//			fprintf(stdout, "\n");
//		}
//
//		/* gravity loads applied uniformly to all frame elements ------- */
//		sfrv = fscanf(fp, "%f %f %f", &gX[lc], &gY[lc], &gZ[lc]);
//		if (sfrv != 3) sferr("gX gY gZ values in load data");
//
//		for (n = 1; n <= nE; n++) {
//
//			n1 = J1[n];	n2 = J2[n];
//
//			coord_trans(xyz, L[n], n1, n2,
//				&t1, &t2, &t3, &t4, &t5, &t6, &t7, &t8, &t9, p[n]);
//
//			eqF_mech[lc][n][1] = d[n] * Ax[n] * L[n] * gX[lc] / 2.0;		//  mass density * cross_section * length
//			eqF_mech[lc][n][2] = d[n] * Ax[n] * L[n] * gY[lc] / 2.0;
//			eqF_mech[lc][n][3] = d[n] * Ax[n] * L[n] * gZ[lc] / 2.0;
//
//			eqF_mech[lc][n][4] = d[n] * Ax[n] * L[n] * L[n] / 12.0 *		// rotation
//				((-t4*t8 + t5*t7)*gY[lc] + (-t4*t9 + t6*t7)*gZ[lc]);
//			eqF_mech[lc][n][5] = d[n] * Ax[n] * L[n] * L[n] / 12.0 *
//				((-t5*t7 + t4*t8)*gX[lc] + (-t5*t9 + t6*t8)*gZ[lc]);
//			eqF_mech[lc][n][6] = d[n] * Ax[n] * L[n] * L[n] / 12.0 *
//				((-t6*t7 + t4*t9)*gX[lc] + (-t6*t8 + t5*t9)*gY[lc]);
//
//			eqF_mech[lc][n][7] = d[n] * Ax[n] * L[n] * gX[lc] / 2.0;
//			eqF_mech[lc][n][8] = d[n] * Ax[n] * L[n] * gY[lc] / 2.0;
//			eqF_mech[lc][n][9] = d[n] * Ax[n] * L[n] * gZ[lc] / 2.0;
//
//			eqF_mech[lc][n][10] = d[n] * Ax[n] * L[n] * L[n] / 12.0 *
//				((t4*t8 - t5*t7)*gY[lc] + (t4*t9 - t6*t7)*gZ[lc]);
//			eqF_mech[lc][n][11] = d[n] * Ax[n] * L[n] * L[n] / 12.0 *
//				((t5*t7 - t4*t8)*gX[lc] + (t5*t9 - t6*t8)*gZ[lc]);
//			eqF_mech[lc][n][12] = d[n] * Ax[n] * L[n] * L[n] / 12.0 *
//				((t6*t7 - t4*t9)*gX[lc] + (t6*t8 - t5*t9)*gY[lc]);
//
//			/* debugging ... check eqF data
//			printf("n=%d ", n);
//			for (l=1;l<=12;l++) {
//			if (eqF_mech[lc][n][l] != 0)
//			printf(" eqF %d = %9.2e ", l, eqF_mech[lc][n][l] );
//			}
//			printf("\n");
//			*/
//		}
//
//		/* end gravity loads */
//
//		/* node point loads -------------------------------------------- */
//		sfrv = fscanf(fp, "%d", &nF[lc]);
//		if (sfrv != 1) sferr("nF value in load data");
//		if (verbose) {
//			fprintf(stdout, "  number of loaded nodes ");
//			dots(stdout, 28);	fprintf(stdout, " nF = %3d\n", nF[lc]);
//		}
//		for (i = 1; i <= nF[lc]; i++) {	/* ! global structural coordinates ! */
//			sfrv = fscanf(fp, "%d", &j);
//			if (sfrv != 1) sferr("node value in point load data");
//			if (j < 1 || j > nN) {
//				sprintf(errMsg, "\n  error in node load data: node number out of range ... Node : %d\n   Perhaps you did not specify %d node loads \n  or perhaps the Input Data file is missing expected data.\n", j, nF[lc]);
//				errorMsg(errMsg);
//				exit(121);
//			}
//
//			for (l = 5; l >= 0; l--) {
//				sfrv = fscanf(fp, "%lf", &F_mech[lc][6 * j - l]);
//				if (sfrv != 1) sferr("force value in point load data");
//			}
//
//			if (F_mech[lc][6 * j - 5] == 0 && F_mech[lc][6 * j - 4] == 0 && F_mech[lc][6 * j - 3] == 0 && F_mech[lc][6 * j - 2] == 0 && F_mech[lc][6 * j - 1] == 0 && F_mech[lc][6 * j] == 0)
//				fprintf(stderr, "\n   Warning: All node loads applied at node %d  are zero\n", j);
//		}					/* end node point loads  */
//
//		/* uniformly distributed loads --------------------------------- */
//		sfrv = fscanf(fp, "%d", &nU[lc]);
//		if (sfrv != 1) sferr("nU value in uniform load data");
//		if (verbose) {
//			fprintf(stdout, "  number of uniformly distributed loads ");
//			dots(stdout, 13);	fprintf(stdout, " nU = %3d\n", nU[lc]);
//		}
//		if (nU[lc] < 0 || nU[lc] > nE) {
//			fprintf(stderr, "  number of uniformly distributed loads ");
//			dots(stderr, 13);
//			fprintf(stderr, " nU = %3d\n", nU[lc]);
//			sprintf(errMsg, "\n  error: valid ranges for nU is 0 ... %d \n", nE);
//			errorMsg(errMsg);
//			exit(131);
//		}
//		for (i = 1; i <= nU[lc]; i++) {	/* ! local element coordinates ! */
//			sfrv = fscanf(fp, "%d", &n);
//			if (sfrv != 1) sferr("frame element number in uniform load data");
//			if (n < 1 || n > nE) {
//				sprintf(errMsg, "\n  error in uniform distributed loads: element number %d is out of range\n", n);
//				errorMsg(errMsg);
//				exit(132);
//			}
//			U[lc][i][1] = (double)n;
//			for (l = 2; l <= 4; l++) {
//				sfrv = fscanf(fp, "%f", &U[lc][i][l]);
//				if (sfrv != 1) sferr("load value in uniform load data");
//			}
//
//			if (U[lc][i][2] == 0 && U[lc][i][3] == 0 && U[lc][i][4] == 0)
//				fprintf(stderr, "\n   Warning: All distributed loads applied to frame element %d  are zero\n", n);
//
//			Nx1 = Nx2 = U[lc][i][2] * Le[n] / 2.0;
//			Vy1 = Vy2 = U[lc][i][3] * Le[n] / 2.0;
//			Vz1 = Vz2 = U[lc][i][4] * Le[n] / 2.0;
//			Mx1 = Mx2 = 0.0;
//			My1 = -U[lc][i][4] * Le[n] * Le[n] / 12.0;	My2 = -My1;
//			Mz1 = U[lc][i][3] * Le[n] * Le[n] / 12.0;	Mz2 = -Mz1;
//
//			/* debugging ... check end force values
//			* printf("n=%d Vy=%9.2e Vz=%9.2e My=%9.2e Mz=%9.2e\n",
//			*				n, Vy1,Vz1, My1,Mz1 );
//			*/
//
//			n1 = J1[n];	n2 = J2[n];
//
//			coord_trans(xyz, L[n], n1, n2,
//				&t1, &t2, &t3, &t4, &t5, &t6, &t7, &t8, &t9, p[n]);
//
//			/* debugging ... check coordinate transform coefficients
//			printf("t1=%5.2f t2=%5.2f t3=%5.2f \n", t1, t2, t3 );
//			printf("t4=%5.2f t5=%5.2f t6=%5.2f \n", t4, t5, t6 );
//			printf("t7=%5.2f t8=%5.2f t9=%5.2f \n", t7, t8, t9 );
//			*/
//
//			/* {F} = [T]'{Q} */
//			eqF_mech[lc][n][1] += (Nx1*t1 + Vy1*t4 + Vz1*t7);
//			eqF_mech[lc][n][2] += (Nx1*t2 + Vy1*t5 + Vz1*t8);
//			eqF_mech[lc][n][3] += (Nx1*t3 + Vy1*t6 + Vz1*t9);
//			eqF_mech[lc][n][4] += (Mx1*t1 + My1*t4 + Mz1*t7);
//			eqF_mech[lc][n][5] += (Mx1*t2 + My1*t5 + Mz1*t8);
//			eqF_mech[lc][n][6] += (Mx1*t3 + My1*t6 + Mz1*t9);
//
//			eqF_mech[lc][n][7] += (Nx2*t1 + Vy2*t4 + Vz2*t7);
//			eqF_mech[lc][n][8] += (Nx2*t2 + Vy2*t5 + Vz2*t8);
//			eqF_mech[lc][n][9] += (Nx2*t3 + Vy2*t6 + Vz2*t9);
//			eqF_mech[lc][n][10] += (Mx2*t1 + My2*t4 + Mz2*t7);
//			eqF_mech[lc][n][11] += (Mx2*t2 + My2*t5 + Mz2*t8);
//			eqF_mech[lc][n][12] += (Mx2*t3 + My2*t6 + Mz2*t9);
//
//			/* debugging ... check eqF values
//			printf("n=%d ", n);
//			for (l=1;l<=12;l++) {
//			if (eqF_mech[lc][n][l] != 0)
//			printf(" eqF %d = %9.2e ", l, eqF_mech[lc][n][l] );
//			}
//			printf("\n");
//			*/
//		}				/* end uniformly distributed loads */
//
//		/* trapezoidally distributed loads ----------------------------- */
//		sfrv = fscanf(fp, "%d", &nW[lc]);
//		if (sfrv != 1) sferr("nW value in load data");
//		if (verbose) {
//			fprintf(stdout, "  number of trapezoidally distributed loads ");
//			dots(stdout, 9);	fprintf(stdout, " nW = %3d\n", nW[lc]);
//		}
//		if (nW[lc] < 0 || nW[lc] > 10 * nE) {
//			sprintf(errMsg, "\n  error: valid ranges for nW is 0 ... %d \n", 10 * nE);
//			errorMsg(errMsg);
//			exit(140);
//		}
//		for (i = 1; i <= nW[lc]; i++) {	/* ! local element coordinates ! */
//			sfrv = fscanf(fp, "%d", &n);
//			if (sfrv != 1) sferr("frame element number in trapezoidal load data");
//			if (n < 1 || n > nE) {
//				sprintf(errMsg, "\n  error in trapezoidally-distributed loads: element number %d is out of range\n", n);
//				errorMsg(errMsg);
//				exit(141);
//			}
//			W[lc][i][1] = (double)n;
//			for (l = 2; l <= 13; l++) {
//				sfrv = fscanf(fp, "%f", &W[lc][i][l]);
//				if (sfrv != 1) sferr("value in trapezoidal load data");
//			}
//
//			Ln = L[n];
//
//			/* error checking */
//
//			if (W[lc][i][4] == 0 && W[lc][i][5] == 0 &&
//				W[lc][i][8] == 0 && W[lc][i][9] == 0 &&
//				W[lc][i][12] == 0 && W[lc][i][13] == 0) {
//				fprintf(stderr, "\n   Warning: All trapezoidal loads applied to frame element %d  are zero\n", n);
//				fprintf(stderr, "     load case: %d , element %d , load %d\n ", lc, n, i);
//			}
//
//			if (W[lc][i][2] < 0) {
//				sprintf(errMsg, "\n   error in x-axis trapezoidal loads, load case: %d , element %d , load %d\n  starting location = %f < 0\n",
//					lc, n, i, W[lc][i][2]);
//				errorMsg(errMsg);
//				exit(142);
//			}
//			if (W[lc][i][2] > W[lc][i][3]) {
//				sprintf(errMsg, "\n   error in x-axis trapezoidal loads, load case: %d , element %d , load %d\n  starting location = %f > ending location = %f \n",
//					lc, n, i, W[lc][i][2], W[lc][i][3]);
//				errorMsg(errMsg);
//				exit(143);
//			}
//			if (W[lc][i][3] > Ln) {
//				sprintf(errMsg, "\n   error in x-axis trapezoidal loads, load case: %d , element %d , load %d\n ending location = %f > L (%f) \n",
//					lc, n, i, W[lc][i][3], Ln);
//				errorMsg(errMsg);
//				exit(144);
//			}
//			if (W[lc][i][6] < 0) {
//				sprintf(errMsg, "\n   error in y-axis trapezoidal loads, load case: %d , element %d , load %d\n starting location = %f < 0\n",
//					lc, n, i, W[lc][i][6]);
//				errorMsg(errMsg);
//				exit(142);
//			}
//			if (W[lc][i][6] > W[lc][i][7]) {
//				sprintf(errMsg, "\n   error in y-axis trapezoidal loads, load case: %d , element %d , load %d\n starting location = %f > ending location = %f \n",
//					lc, n, i, W[lc][i][6], W[lc][i][7]);
//				errorMsg(errMsg);
//				exit(143);
//			}
//			if (W[lc][i][7] > Ln) {
//				sprintf(errMsg, "\n   error in y-axis trapezoidal loads, load case: %d , element %d , load %d\n ending location = %f > L (%f) \n",
//					lc, n, i, W[lc][i][7], Ln);
//				errorMsg(errMsg);
//				exit(144);
//			}
//			if (W[lc][i][10] < 0) {
//				sprintf(errMsg, "\n   error in z-axis trapezoidal loads, load case: %d , element %d , load %d\n starting location = %f < 0\n",
//					lc, n, i, W[lc][i][10]);
//				errorMsg(errMsg);
//				exit(142);
//			}
//			if (W[lc][i][10] > W[lc][i][11]) {
//				sprintf(errMsg, "\n   error in z-axis trapezoidal loads, load case: %d , element %d , load %d\n starting location = %f > ending location = %f \n",
//					lc, n, i, W[lc][i][10], W[lc][i][11]);
//				errorMsg(errMsg);
//				exit(143);
//			}
//			if (W[lc][i][11] > Ln) {
//				sprintf(errMsg, "\n   error in z-axis trapezoidal loads, load case: %d , element %d , load %d\n ending location = %f > L (%f) \n", lc, n, i, W[lc][i][11], Ln);
//				errorMsg(errMsg);
//				exit(144);
//			}
//
//			if (shear) {
//				Ksy = (12.0*E[n] * Iz[n]) / (G[n] * Asy[n] * Le[n] * Le[n]);
//				Ksz = (12.0*E[n] * Iy[n]) / (G[n] * Asz[n] * Le[n] * Le[n]);
//			}
//			else	Ksy = Ksz = 0.0;
//
//			/* x-axis trapezoidal loads (along the frame element length) */
//			x1 = W[lc][i][2]; x2 = W[lc][i][3];
//			w1 = W[lc][i][4]; w2 = W[lc][i][5];
//
//			Nx1 = (3.0*(w1 + w2)*Ln*(x2 - x1) - (2.0*w2 + w1)*x2*x2 + (w2 - w1)*x2*x1 + (2.0*w1 + w2)*x1*x1) / (6.0*Ln);
//			Nx2 = (-(2.0*w1 + w2)*x1*x1 + (2.0*w2 + w1)*x2*x2 - (w2 - w1)*x1*x2) / (6.0*Ln);
//
//			/* y-axis trapezoidal loads (across the frame element length) */
//			x1 = W[lc][i][6];  x2 = W[lc][i][7];
//			w1 = W[lc][i][8]; w2 = W[lc][i][9];
//
//			R1o = ((2.0*w1 + w2)*x1*x1 - (w1 + 2.0*w2)*x2*x2 +
//				3.0*(w1 + w2)*Ln*(x2 - x1) - (w1 - w2)*x1*x2) / (6.0*Ln);
//			R2o = ((w1 + 2.0*w2)*x2*x2 + (w1 - w2)*x1*x2 -
//				(2.0*w1 + w2)*x1*x1) / (6.0*Ln);
//
//			f01 = (3.0*(w2 + 4.0*w1)*x1*x1*x1*x1 - 3.0*(w1 + 4.0*w2)*x2*x2*x2*x2
//				- 15.0*(w2 + 3.0*w1)*Ln*x1*x1*x1 + 15.0*(w1 + 3.0*w2)*Ln*x2*x2*x2
//				- 3.0*(w1 - w2)*x1*x2*(x1*x1 + x2*x2)
//				+ 20.0*(w2 + 2.0*w1)*Ln*Ln*x1*x1 - 20.0*(w1 + 2.0*w2)*Ln*Ln*x2*x2
//				+ 15.0*(w1 - w2)*Ln*x1*x2*(x1 + x2)
//				- 3.0*(w1 - w2)*x1*x1*x2*x2 - 20.0*(w1 - w2)*Ln*Ln*x1*x2) / 360.0;
//
//			f02 = (3.0*(w2 + 4.0*w1)*x1*x1*x1*x1 - 3.0*(w1 + 4.0*w2)*x2*x2*x2*x2
//				- 3.0*(w1 - w2)*x1*x2*(x1*x1 + x2*x2)
//				- 10.0*(w2 + 2.0*w1)*Ln*Ln*x1*x1 + 10.0*(w1 + 2.0*w2)*Ln*Ln*x2*x2
//				- 3.0*(w1 - w2)*x1*x1*x2*x2 + 10.0*(w1 - w2)*Ln*Ln*x1*x2) / 360.0;
//
//			Mz1 = -(4.0*f01 + 2.0*f02 + Ksy*(f01 - f02)) / (Ln*Ln*(1.0 + Ksy));
//			Mz2 = -(2.0*f01 + 4.0*f02 - Ksy*(f01 - f02)) / (Ln*Ln*(1.0 + Ksy));
//
//			Vy1 = R1o + Mz1 / Ln + Mz2 / Ln;
//			Vy2 = R2o - Mz1 / Ln - Mz2 / Ln;
//
//			/* z-axis trapezoidal loads (across the frame element length) */
//			x1 = W[lc][i][10]; x2 = W[lc][i][11];
//			w1 = W[lc][i][12]; w2 = W[lc][i][13];
//
//			R1o = ((2.0*w1 + w2)*x1*x1 - (w1 + 2.0*w2)*x2*x2 +
//				3.0*(w1 + w2)*Ln*(x2 - x1) - (w1 - w2)*x1*x2) / (6.0*Ln);
//			R2o = ((w1 + 2.0*w2)*x2*x2 + (w1 - w2)*x1*x2 -
//				(2.0*w1 + w2)*x1*x1) / (6.0*Ln);
//
//			f01 = (3.0*(w2 + 4.0*w1)*x1*x1*x1*x1 - 3.0*(w1 + 4.0*w2)*x2*x2*x2*x2
//				- 15.0*(w2 + 3.0*w1)*Ln*x1*x1*x1 + 15.0*(w1 + 3.0*w2)*Ln*x2*x2*x2
//				- 3.0*(w1 - w2)*x1*x2*(x1*x1 + x2*x2)
//				+ 20.0*(w2 + 2.0*w1)*Ln*Ln*x1*x1 - 20.0*(w1 + 2.0*w2)*Ln*Ln*x2*x2
//				+ 15.0*(w1 - w2)*Ln*x1*x2*(x1 + x2)
//				- 3.0*(w1 - w2)*x1*x1*x2*x2 - 20.0*(w1 - w2)*Ln*Ln*x1*x2) / 360.0;
//
//			f02 = (3.0*(w2 + 4.0*w1)*x1*x1*x1*x1 - 3.0*(w1 + 4.0*w2)*x2*x2*x2*x2
//				- 3.0*(w1 - w2)*x1*x2*(x1*x1 + x2*x2)
//				- 10.0*(w2 + 2.0*w1)*Ln*Ln*x1*x1 + 10.0*(w1 + 2.0*w2)*Ln*Ln*x2*x2
//				- 3.0*(w1 - w2)*x1*x1*x2*x2 + 10.0*(w1 - w2)*Ln*Ln*x1*x2) / 360.0;
//
//			My1 = (4.0*f01 + 2.0*f02 + Ksz*(f01 - f02)) / (Ln*Ln*(1.0 + Ksz));
//			My2 = (2.0*f01 + 4.0*f02 - Ksz*(f01 - f02)) / (Ln*Ln*(1.0 + Ksz));
//
//			Vz1 = R1o - My1 / Ln - My2 / Ln;
//			Vz2 = R2o + My1 / Ln + My2 / Ln;
//
//			/* debugging ... check internal force values
//			printf("n=%d\n Nx1=%9.3f\n Nx2=%9.3f\n Vy1=%9.3f\n Vy2=%9.3f\n Vz1=%9.3f\n Vz2=%9.3f\n My1=%9.3f\n My2=%9.3f\n Mz1=%9.3f\n Mz2=%9.3f\n",
//			n, Nx1,Nx2,Vy1,Vy2,Vz1,Vz2, My1,My2,Mz1,Mz2 );
//			*/
//
//			n1 = J1[n];	n2 = J2[n];
//
//			coord_trans(xyz, Ln, n1, n2,
//				&t1, &t2, &t3, &t4, &t5, &t6, &t7, &t8, &t9, p[n]);
//
//			/* debugging ... check coordinate transformation coefficients
//			printf("t1=%5.2f t2=%5.2f t3=%5.2f \n", t1, t2, t3 );
//			printf("t4=%5.2f t5=%5.2f t6=%5.2f \n", t4, t5, t6 );
//			printf("t7=%5.2f t8=%5.2f t9=%5.2f \n", t7, t8, t9 );
//			*/
//
//			/* {F} = [T]'{Q} */
//			eqF_mech[lc][n][1] += (Nx1*t1 + Vy1*t4 + Vz1*t7);
//			eqF_mech[lc][n][2] += (Nx1*t2 + Vy1*t5 + Vz1*t8);
//			eqF_mech[lc][n][3] += (Nx1*t3 + Vy1*t6 + Vz1*t9);
//			eqF_mech[lc][n][4] += (Mx1*t1 + My1*t4 + Mz1*t7);
//			eqF_mech[lc][n][5] += (Mx1*t2 + My1*t5 + Mz1*t8);
//			eqF_mech[lc][n][6] += (Mx1*t3 + My1*t6 + Mz1*t9);
//
//			eqF_mech[lc][n][7] += (Nx2*t1 + Vy2*t4 + Vz2*t7);
//			eqF_mech[lc][n][8] += (Nx2*t2 + Vy2*t5 + Vz2*t8);
//			eqF_mech[lc][n][9] += (Nx2*t3 + Vy2*t6 + Vz2*t9);
//			eqF_mech[lc][n][10] += (Mx2*t1 + My2*t4 + Mz2*t7);
//			eqF_mech[lc][n][11] += (Mx2*t2 + My2*t5 + Mz2*t8);
//			eqF_mech[lc][n][12] += (Mx2*t3 + My2*t6 + Mz2*t9);
//
//			/* debugging ... check eqF data
//			for (l=1;l<=13;l++) printf(" %9.2e ", W[lc][i][l] );
//			printf("\n");
//			printf("n=%d ", n);
//			for (l=1;l<=12;l++) {
//			if (eqF_mech[lc][n][l] != 0)
//			printf(" eqF %d = %9.3f ", l, eqF_mech[lc][n][l] );
//			}
//			printf("\n");
//			*/
//		}			/* end trapezoidally distributed loads */
//
//		/* internal element point loads -------------------------------- */
//		sfrv = fscanf(fp, "%d", &nP[lc]);
//		if (sfrv != 1) sferr("nP value load data");
//		if (verbose) {
//			fprintf(stdout, "  number of concentrated frame element point loads ");
//			dots(stdout, 2);	fprintf(stdout, " nP = %3d\n", nP[lc]);
//		}
//		if (nP[lc] < 0 || nP[lc] > 10 * nE) {
//			fprintf(stderr, "  number of concentrated frame element point loads ");
//			dots(stderr, 3);
//			fprintf(stderr, " nP = %3d\n", nP[lc]);
//			sprintf(errMsg, "\n  error: valid ranges for nP is 0 ... %d \n", 10 * nE);
//			errorMsg(errMsg);
//			exit(150);
//		}
//		for (i = 1; i <= nP[lc]; i++) {	/* ! local element coordinates ! */
//			sfrv = fscanf(fp, "%d", &n);
//			if (sfrv != 1) sferr("frame element number value point load data");
//			if (n < 1 || n > nE) {
//				sprintf(errMsg, "\n   error in internal point loads: frame element number %d is out of range\n", n);
//				errorMsg(errMsg);
//				exit(151);
//			}
//			P[lc][i][1] = (double)n;
//			for (l = 2; l <= 5; l++) {
//				sfrv = fscanf(fp, "%f", &P[lc][i][l]);
//				if (sfrv != 1) sferr("value in point load data");
//			}
//			a = P[lc][i][5];	b = L[n] - a;
//
//			if (a < 0 || L[n] < a || b < 0 || L[n] < b) {
//				sprintf(errMsg, "\n  error in point load data: Point load coord. out of range\n   Frame element number: %d  L: %lf  load coord.: %lf\n",
//					n, L[n], P[lc][i][5]);
//				errorMsg(errMsg);
//				exit(152);
//			}
//
//			if (shear) {
//				Ksy = (12.0*E[n] * Iz[n]) / (G[n] * Asy[n] * Le[n] * Le[n]);
//				Ksz = (12.0*E[n] * Iy[n]) / (G[n] * Asz[n] * Le[n] * Le[n]);
//			}
//			else	Ksy = Ksz = 0.0;
//
//			Ln = L[n];
//
//			Nx1 = P[lc][i][2] * a / Ln;
//			Nx2 = P[lc][i][2] * b / Ln;
//
//			Vy1 = (1. / (1. + Ksz))    * P[lc][i][3] * b*b*(3.*a + b) / (Ln*Ln*Ln) +
//				(Ksz / (1. + Ksz)) * P[lc][i][3] * b / Ln;
//			Vy2 = (1. / (1. + Ksz))    * P[lc][i][3] * a*a*(3.*b + a) / (Ln*Ln*Ln) +
//				(Ksz / (1. + Ksz)) * P[lc][i][3] * a / Ln;
//
//			Vz1 = (1. / (1. + Ksy))    * P[lc][i][4] * b*b*(3.*a + b) / (Ln*Ln*Ln) +
//				(Ksy / (1. + Ksy)) * P[lc][i][4] * b / Ln;
//			Vz2 = (1. / (1. + Ksy))    * P[lc][i][4] * a*a*(3.*b + a) / (Ln*Ln*Ln) +
//				(Ksy / (1. + Ksy)) * P[lc][i][4] * a / Ln;
//
//			Mx1 = Mx2 = 0.0;
//
//			My1 = -(1. / (1. + Ksy))  * P[lc][i][4] * a*b*b / (Ln*Ln) -
//				(Ksy / (1. + Ksy))* P[lc][i][4] * a*b / (2.*Ln);
//			My2 = (1. / (1. + Ksy))  * P[lc][i][4] * a*a*b / (Ln*Ln) +
//				(Ksy / (1. + Ksy))* P[lc][i][4] * a*b / (2.*Ln);
//
//			Mz1 = (1. / (1. + Ksz))  * P[lc][i][3] * a*b*b / (Ln*Ln) +
//				(Ksz / (1. + Ksz))* P[lc][i][3] * a*b / (2.*Ln);
//			Mz2 = -(1. / (1. + Ksz))  * P[lc][i][3] * a*a*b / (Ln*Ln) -
//				(Ksz / (1. + Ksz))* P[lc][i][3] * a*b / (2.*Ln);
//
//			n1 = J1[n];	n2 = J2[n];
//
//			coord_trans(xyz, Ln, n1, n2,
//				&t1, &t2, &t3, &t4, &t5, &t6, &t7, &t8, &t9, p[n]);
//
//			/* {F} = [T]'{Q} */
//			eqF_mech[lc][n][1] += (Nx1*t1 + Vy1*t4 + Vz1*t7);
//			eqF_mech[lc][n][2] += (Nx1*t2 + Vy1*t5 + Vz1*t8);
//			eqF_mech[lc][n][3] += (Nx1*t3 + Vy1*t6 + Vz1*t9);
//			eqF_mech[lc][n][4] += (Mx1*t1 + My1*t4 + Mz1*t7);
//			eqF_mech[lc][n][5] += (Mx1*t2 + My1*t5 + Mz1*t8);
//			eqF_mech[lc][n][6] += (Mx1*t3 + My1*t6 + Mz1*t9);
//
//			eqF_mech[lc][n][7] += (Nx2*t1 + Vy2*t4 + Vz2*t7);
//			eqF_mech[lc][n][8] += (Nx2*t2 + Vy2*t5 + Vz2*t8);
//			eqF_mech[lc][n][9] += (Nx2*t3 + Vy2*t6 + Vz2*t9);
//			eqF_mech[lc][n][10] += (Mx2*t1 + My2*t4 + Mz2*t7);
//			eqF_mech[lc][n][11] += (Mx2*t2 + My2*t5 + Mz2*t8);
//			eqF_mech[lc][n][12] += (Mx2*t3 + My2*t6 + Mz2*t9);
//		}					/* end element point loads */
//
//		/* thermal loads ----------------------------------------------- */
//		sfrv = fscanf(fp, "%d", &nT[lc]);
//		if (sfrv != 1) sferr("nT value in load data");
//		if (verbose) {
//			fprintf(stdout, "  number of temperature changes ");
//			dots(stdout, 21); fprintf(stdout, " nT = %3d\n", nT[lc]);
//		}
//		if (nT[lc] < 0 || nT[lc] > nE) {
//			fprintf(stderr, "  number of temperature changes ");
//			dots(stderr, 21);
//			fprintf(stderr, " nT = %3d\n", nT[lc]);
//			sprintf(errMsg, "\n  error: valid ranges for nT is 0 ... %d \n", nE);
//			errorMsg(errMsg);
//			exit(160);
//		}
//		for (i = 1; i <= nT[lc]; i++) {	/* ! local element coordinates ! */
//			sfrv = fscanf(fp, "%d", &n);
//			if (sfrv != 1) sferr("frame element number in temperature load data");
//			if (n < 1 || n > nE) {
//				sprintf(errMsg, "\n  error in temperature loads: frame element number %d is out of range\n", n);
//				errorMsg(errMsg);
//				exit(161);
//			}
//			T[lc][i][1] = (double)n;
//			for (l = 2; l <= 8; l++) {
//				sfrv = fscanf(fp, "%f", &T[lc][i][l]);
//				if (sfrv != 1) sferr("value in temperature load data");
//			}
//			a = T[lc][i][2];
//			hy = T[lc][i][3];
//			hz = T[lc][i][4];
//
//			if (hy < 0 || hz < 0) {
//				sprintf(errMsg, "\n  error in thermal load data: section dimension < 0\n   Frame element number: %d  hy: %f  hz: %f\n", n, hy, hz);
//				errorMsg(errMsg);
//				exit(162);
//			}
//
//			Nx2 = a*(1.0 / 4.0)*(T[lc][i][5] + T[lc][i][6] + T[lc][i][7] + T[lc][i][8])*E[n] * Ax[n];
//			Nx1 = -Nx2;
//			Vy1 = Vy2 = Vz1 = Vz2 = 0.0;
//			Mx1 = Mx2 = 0.0;
//			My1 = (a / hz)*(T[lc][i][8] - T[lc][i][7])*E[n] * Iy[n];
//			My2 = -My1;
//			Mz1 = (a / hy)*(T[lc][i][5] - T[lc][i][6])*E[n] * Iz[n];
//			Mz2 = -Mz1;
//
//			n1 = J1[n];	n2 = J2[n];
//
//			coord_trans(xyz, L[n], n1, n2,
//				&t1, &t2, &t3, &t4, &t5, &t6, &t7, &t8, &t9, p[n]);
//
//			/* {F} = [T]'{Q} */
//			eqF_temp[lc][n][1] += (Nx1*t1 + Vy1*t4 + Vz1*t7);
//			eqF_temp[lc][n][2] += (Nx1*t2 + Vy1*t5 + Vz1*t8);
//			eqF_temp[lc][n][3] += (Nx1*t3 + Vy1*t6 + Vz1*t9);
//			eqF_temp[lc][n][4] += (Mx1*t1 + My1*t4 + Mz1*t7);
//			eqF_temp[lc][n][5] += (Mx1*t2 + My1*t5 + Mz1*t8);
//			eqF_temp[lc][n][6] += (Mx1*t3 + My1*t6 + Mz1*t9);
//
//			eqF_temp[lc][n][7] += (Nx2*t1 + Vy2*t4 + Vz2*t7);
//			eqF_temp[lc][n][8] += (Nx2*t2 + Vy2*t5 + Vz2*t8);
//			eqF_temp[lc][n][9] += (Nx2*t3 + Vy2*t6 + Vz2*t9);
//			eqF_temp[lc][n][10] += (Mx2*t1 + My2*t4 + Mz2*t7);
//			eqF_temp[lc][n][11] += (Mx2*t2 + My2*t5 + Mz2*t8);
//			eqF_temp[lc][n][12] += (Mx2*t3 + My2*t6 + Mz2*t9);
//		}				/* end thermal loads	*/
//
//		/* debugging ...  check eqF's prior to asembly
//		for (n=1; n<=nE; n++) {
//		printf("n=%d ", n);
//		for (l=1;l<=12;l++) {
//		if (eqF_mech[lc][n][l] != 0)
//		printf(" eqF %d = %9.2e ", l, eqF_mech[lc][n][l] );
//		}
//		printf("\n");
//		}
//		*/
//
//		// assemble all element equivalent loads into 
//		// separate load vectors for mechanical and thermal loading
//		for (n = 1; n <= nE; n++) {
//			n1 = J1[n];	n2 = J2[n];
//			for (i = 1; i <= 6; i++) F_mech[lc][6 * n1 - 6 + i] += eqF_mech[lc][n][i];
//			for (i = 7; i <= 12; i++) F_mech[lc][6 * n2 - 12 + i] += eqF_mech[lc][n][i];
//			for (i = 1; i <= 6; i++) F_temp[lc][6 * n1 - 6 + i] += eqF_temp[lc][n][i];
//			for (i = 7; i <= 12; i++) F_temp[lc][6 * n2 - 12 + i] += eqF_temp[lc][n][i];
//		}
//
//		/* prescribed displacements ------------------------------------ */
//		sfrv = fscanf(fp, "%d", &nD[lc]);
//		if (sfrv != 1) sferr("nD value in load data");
//		if (verbose) {
//			fprintf(stdout, "  number of prescribed displacements ");
//			dots(stdout, 16);	fprintf(stdout, " nD = %3d\n", nD[lc]);
//		}
//		for (i = 1; i <= nD[lc]; i++) {
//			sfrv = fscanf(fp, "%d", &j);
//			if (sfrv != 1) sferr("node number value in prescribed displacement data");
//			for (l = 5; l >= 0; l--) {
//				sfrv = fscanf(fp, "%f", &Dp[lc][6 * j - l]);
//				if (sfrv != 1) sferr("prescribed displacement value");
//				if (r[6 * j - l] == 0 && Dp[lc][6 * j - l] != 0.0) {
//					sprintf(errMsg, " Initial displacements can be prescribed only at restrained coordinates\n  node: %d  dof: %d  r: %d\n",
//						j, 6 - l, r[6 * j - l]);
//					errorMsg(errMsg);
//					exit(171);
//				}
//			}
//		}
//
//	}					/* end load-case loop */
//
//	return;
//}
//
///*
//* READ_MASS_DATA  -  read element densities and extra inertial mass data	16aug01
//*/
//void StiffnessIO::ReadMassData(
//	FILE *fp,
//	char *OUT_file,
//	int nN, int nE, int *nI, int *nX,
//	float *d, float *EMs,
//	float *NMs, float *NMx, float *NMy, float *NMz,
//	double *L, float *Ax,
//	double *total_mass, double *struct_mass,
//	int *nM, int *Mmethod, int modal_flag,
//	int *lump, int lump_flag,
//	double *tol, double tol_flag, double *shift, double shift_flag,
//	double *exagg_modal,
//	char modepath[],
//	int anim[], float *pan, float pan_flag,
//	int verbose, int debug
//	)
//{
//	/*	double	ms = 0.0; */
//	int	i, j, jnt, m, b, nA;
//	int	full_len = 0, len = 0;
//	int	sfrv = 0;		/* *scanf return value	*/
//
//	char	base_file[96] = "EMPTY_BASE";
//	char	mode_file[96] = "EMPTY_MODE";
//	char	errMsg[MAXL];
//
//	*total_mass = *struct_mass = 0.0;
//
//	sfrv = fscanf(fp, "%d", nM);
//	if (sfrv != 1) sferr("nM value in mass data");
//
//	if (verbose)
//	{
//		fprintf(stdout, " number of dynamic modes ");
//		dots(stdout, 28);
//		fprintf(stdout, " nM = %3d\n", *nM);
//	}
//
//	if (*nM < 1 || sfrv != 1) {
//		*nM = 0;
//		return;
//	}
//
//	sfrv = fscanf(fp, "%d", Mmethod);
//	if (sfrv != 1) sferr("Mmethod value in mass data");
//	if (modal_flag != -1)	*Mmethod = modal_flag;
//
//	if (verbose) {
//		fprintf(stdout, " modal analysis method ");
//		dots(stdout, 30);	fprintf(stdout, " %3d ", *Mmethod);
//		if (*Mmethod == 1) fprintf(stdout, " (Subspace-Jacobi)\n");
//		if (*Mmethod == 2) fprintf(stdout, " (Stodola)\n");
//	}
//
//
//#ifdef MASSDATA_DEBUG
//	FILE	*mf;				// mass data file
//	mf = fopen("MassData.txt", "w");		// open mass data file
//	if ((mf = fopen("MassData.txt", "w")) == NULL) {
//		errorMsg("\n  error: cannot open mass data debug file: 'MassData.txt' \n");
//		exit(29);
//	}
//	fprintf(mf, "%% structural mass data \n");
//	fprintf(mf, "%% element\tAx\t\tlength\t\tdensity\t\tmass \n");
//#endif
//
//	sfrv = fscanf(fp, "%d", lump);
//	if (sfrv != 1) sferr("lump value in mass data");
//	sfrv = fscanf(fp, "%lf", tol);
//	if (sfrv != 1) sferr("tol value in mass data");
//	sfrv = fscanf(fp, "%lf", shift);
//	if (sfrv != 1) sferr("shift value in mass data");
//	sfrv = fscanf(fp, "%lf", exagg_modal);
//	if (sfrv != 1) sferr("exagg_modal value in mass data");
//
//	if (lump_flag != -1)	*lump = lump_flag;
//	if (tol_flag != -1.0)	*tol = tol_flag;
//	if (shift_flag != -1.0)	*shift = shift_flag;
//
//
//	/* number of nodes with extra inertias */
//	sfrv = fscanf(fp, "%d", nI);
//	if (sfrv != 1) sferr("nI value in mass data");
//	if (verbose) {
//		fprintf(stdout, " number of nodes with extra lumped inertia ");
//		dots(stdout, 10);	fprintf(stdout, " nI = %3d\n", *nI);
//	}
//	for (j = 1; j <= *nI; j++) {
//		sfrv = fscanf(fp, "%d", &jnt);
//		if (sfrv != 1) sferr("node value in extra node mass data");
//		if (jnt < 1 || jnt > nN) {
//			sprintf(errMsg, "\n  error in node mass data: node number out of range    Node : %d  \n   Perhaps you did not specify %d extra masses \n   or perhaps the Input Data file is missing expected data.\n",
//				jnt, *nI);
//			errorMsg(errMsg);
//			exit(86);
//		}
//		sfrv = fscanf(fp, "%f %f %f %f",
//			&NMs[jnt], &NMx[jnt], &NMy[jnt], &NMz[jnt]);
//		if (sfrv != 4) sferr("node inertia in extra mass data");
//		*total_mass += NMs[jnt];
//
//		if (NMs[jnt] == 0 && NMx[jnt] == 0 && NMy[jnt] == 0 && NMz[jnt] == 0)
//			fprintf(stderr, "\n  Warning: All extra node inertia at node %d  are zero\n", jnt);
//	}
//
//	/* number of frame elements with extra beam mass */
//	sfrv = fscanf(fp, "%d", nX);
//	if (sfrv != 1) sferr("nX value in mass data");
//	if (verbose) {
//		fprintf(stdout, " number of frame elements with extra mass ");
//		dots(stdout, 11);	fprintf(stdout, " nX = %3d\n", *nX);
//		if (sfrv != 1) sferr("element value in extra element mass data");
//	}
//	for (m = 1; m <= *nX; m++) {
//		sfrv = fscanf(fp, "%d", &b);
//		if (sfrv != 1) sferr("element number in extra element mass data");
//		if (b < 1 || b > nE) {
//			sprintf(errMsg, "\n  error in element mass data: element number out of range   Element: %d  \n   Perhaps you did not specify %d extra masses \n   or perhaps the Input Data file is missing expected data.\n",
//				b, *nX);
//			errorMsg(errMsg);
//			exit(87);
//		}
//		sfrv = fscanf(fp, "%f", &EMs[b]);
//		if (sfrv != 1) sferr("extra element mass value in mass data");
//	}
//
//
//	/* calculate the total mass and the structural mass */
//	for (b = 1; b <= nE; b++) 
//	{
//		*total_mass += d[b] * Ax[b] * L[b] + EMs[b];
//		*struct_mass += d[b] * Ax[b] * L[b];
//
//#ifdef MASSDATA_DEBUG
//		fprintf(mf, " %4d\t\t%12.5e\t%12.5e\t%12.5e\t%12.5e \n",
//			b, Ax[b], L[b], d[b], d[b] * Ax[b] * L[b]);
//#endif
//	}
//
//#ifdef MASSDATA_DEBUG
//	fclose(mf);
//#endif
//
//	for (m = 1; m <= nE; m++) {			/* check inertia data	*/
//		if (d[m] < 0.0 || EMs[m] < 0.0 || d[m] + EMs[m] <= 0.0) {
//			sprintf(errMsg, "\n  error: Non-positive mass or density\n  d[%d]= %f  EMs[%d]= %f\n", m, d[m], m, EMs[m]);
//			errorMsg(errMsg);
//			exit(88);
//		}
//	}
//	/*	for (m=1;m<=nE;m++) ms += EMs[m]; // consistent mass doesn't agree  */
//	/*	if ( ms > 0.0 )	    *lump = 1;    // with concentrated masses, EMs  */
//
//	if (verbose) {
//		fprintf(stdout, " structural mass ");
//		dots(stdout, 36);	fprintf(stdout, "  %12.4e\n", *struct_mass);
//		fprintf(stdout, " total mass ");
//		dots(stdout, 41);	fprintf(stdout, "  %12.4e\n", *total_mass);
//	}
//	sfrv = fscanf(fp, "%d", &nA);
//	if (sfrv != 1) sferr("nA value in mode animation data");
//	if (verbose) {
//		fprintf(stdout, " number of modes to be animated ");
//		dots(stdout, 21);	fprintf(stdout, " nA = %3d\n", nA);
//	}
//	if (nA > 20)
//		fprintf(stderr, " nA = %d, only 100 or fewer modes may be animated\n", nA);
//	for (m = 0; m < 20; m++)	anim[m] = 0;
//	for (m = 1; m <= nA; m++) {
//		sfrv = fscanf(fp, "%d", &anim[m]);
//		if (sfrv != 1) sferr("mode number in mode animation data");
//	}
//
//	sfrv = fscanf(fp, "%f", pan);
//	if (sfrv != 1) sferr("pan value in mode animation data");
//	if (pan_flag != -1.0)	*pan = pan_flag;
//
//	if (verbose) {
//		fprintf(stdout, " pan rate ");
//		dots(stdout, 43); fprintf(stdout, " %8.3f\n", *pan);
//	}
//
//	strcpy(base_file, OUT_file);
//	while (base_file[len++] != '\0')
//		/* the length of the base_file */;
//	full_len = len;
//
//	while (base_file[len--] != '.' && len > 0)
//		/* find the last '.' in base_file */;
//	if (len == 0)	len = full_len;
//	base_file[++len] = '\0';	/* end base_file at the last '.' */
//
//	while (base_file[len] != '/' && base_file[len] != '\\' && len > 0)
//		len--;	/* find the last '/' or '\' in base_file */
//	i = 0;
//	while (base_file[len] != '\0')
//		mode_file[i++] = base_file[len++];
//	mode_file[i] = '\0';
//	strcat(mode_file, "-m");
//	output_path(mode_file, modepath, FRAME3DD_PATHMAX, NULL);
//
//	return;
//}
//
///*
//* READ_CONDENSE   -  read matrix condensation information 	        30aug01
//*/
//void StiffnessIO::ReadCondense(
//	FILE *fp,
//	int nN, int nM,
//	int *nC, int *Cdof,
//	int *Cmethod, int condense_flag, int *c, int *m, int verbose
//	)
//{
//	int	i, j, k, **cm;
//	int	sfrv = 0;		/* *scanf return value */
//	char	errMsg[MAXL];
//
//	*Cmethod = *nC = *Cdof = 0;
//
//	if ((sfrv = fscanf(fp, "%d", Cmethod)) != 1)   {
//		*Cmethod = *nC = *Cdof = 0;
//		if (verbose)
//			fprintf(stdout, " missing matrix condensation data \n");
//		return;
//	}
//
//	if (condense_flag != -1)	*Cmethod = condense_flag;
//
//	if (*Cmethod <= 0)  {
//		if (verbose)
//			fprintf(stdout, " Cmethod = %d : no matrix condensation \n", *Cmethod);
//		*Cmethod = *nC = *Cdof = 0;
//		return;
//	}
//
//	if (*Cmethod > 3) *Cmethod = 1;	/* default */
//	if (verbose) {
//		fprintf(stdout, " condensation method ");
//		dots(stdout, 32);	fprintf(stdout, " %d ", *Cmethod);
//		if (*Cmethod == 1)	fprintf(stdout, " (static only) \n");
//		if (*Cmethod == 2)	fprintf(stdout, " (Guyan) \n");
//		if (*Cmethod == 3)	fprintf(stdout, " (dynamic) \n");
//	}
//
//	if ((sfrv = fscanf(fp, "%d", nC)) != 1)  {
//		*Cmethod = *nC = *Cdof = 0;
//		if (verbose)
//			fprintf(stdout, " missing matrix condensation data \n");
//		return;
//	}
//
//	if (verbose) {
//		fprintf(stdout, " number of nodes with condensed DoF's ");
//		dots(stdout, 15);	fprintf(stdout, " nC = %3d\n", *nC);
//	}
//
//	if ((*nC) > nN) {
//		sprintf(errMsg, "\n  error in matrix condensation data: \n error: nC > nN ... nC=%d; nN=%d;\n The number of nodes with condensed DoF's may not exceed the total number of nodes.\n",
//			*nC, nN);
//		errorMsg(errMsg);
//		exit(90);
//	}
//
//	cm = imatrix(1, *nC, 1, 7);
//
//	for (i = 1; i <= *nC; i++) {
//		sfrv = fscanf(fp, "%d %d %d %d %d %d %d",
//			&cm[i][1],
//			&cm[i][2], &cm[i][3], &cm[i][4], &cm[i][5], &cm[i][6], &cm[i][7]);
//		if (sfrv != 7) sferr("DoF numbers in condensation data");
//		if (cm[i][1] < 1 || cm[i][1] > nN) {		/* error check */
//			sprintf(errMsg, "\n  error in matrix condensation data: \n  condensed node number out of range\n  cj[%d] = %d  ... nN = %d  \n", i, cm[i][1], nN);
//			errorMsg(errMsg);
//			exit(91);
//		}
//	}
//
//	for (i = 1; i <= *nC; i++)  for (j = 2; j <= 7; j++)  if (cm[i][j]) (*Cdof)++;
//
//	k = 1;
//	for (i = 1; i <= *nC; i++) {
//		for (j = 2; j <= 7; j++) {
//			if (cm[i][j]) {
//				c[k] = 6 * (cm[i][1] - 1) + j - 1;
//				++k;
//			}
//		}
//	}
//
//	for (i = 1; i <= *Cdof; i++) {
//		sfrv = fscanf(fp, "%d", &m[i]);
//		if (sfrv != 1 && *Cmethod == 3) {
//			sferr("mode number in condensation data");
//			sprintf(errMsg, "condensed mode %d = %d", i, m[i]);
//			errorMsg(errMsg);
//		}
//		if ((m[i] < 0 || m[i] > nM) && *Cmethod == 3) {
//			sprintf(errMsg, "\n  error in matrix condensation data: \n  m[%d] = %d \n The condensed mode number must be between   1 and %d (modes).\n",
//				i, m[i], nM);
//			errorMsg(errMsg);
//			exit(92);
//		}
//	}
//
//	free_imatrix(cm, 1, *nC, 1, 7);
//	return;
//}
//
///*
//* STATIC_MESH  - create mesh data of deformed and undeformed mesh  22 Feb 1999
//* use gnuplot
//* useful gnuplot options: unset xtics ytics ztics border view key
//* This function illustrates how to read the internal force output data file.
//* The internal force output data file contains all the information required
//* to plot deformed meshes, internal axial force, internal shear force, internal
//* torsion, and internal bending moment diagrams.
//*/
//void StiffnessIO::GnuPltStaticMesh(
//	char IN_file[],
//	char infcpath[], char meshpath[], char plotpath[],
//	char *title, int nN, int nE, int nL, int lc, int DoF,
//	vec3 *xyz, double *L,
//	int *N1, int *N2, float *p, double *D,
//	double exagg_static, int D3_flag, int anlyz, float dx, float scale
//	)
//{
//	FILE	*fpif = NULL, *fpm = NULL;
//	double	mx, my, mz; /* coordinates of the frame element number labels */
//	char	fnif[FILENMAX], meshfl[FILENMAX],
//		D2 = '#', D3 = '#',	/* indicates plotting in 2D or 3D	*/
//		errMsg[MAXL],
//		ch = 'a';
//	int	sfrv = 0,		/* *scanf return value			*/
//		frel, nx,	/* frame element number, number of increments */
//		n1, n2;		/* node numbers			*/
//	float	x1, y1, z1,	/* coordinates of node n1		*/
//		x2, y2, z2;	/* coordinates of node n2		*/
//	int	j = 0, m = 0, n = 0,
//		X = 0, Y = 0, Z = 0,
//		lw = 1;		/*  line width of deformed mesh		*/
//	time_t  now;		/* modern time variable type		*/
//
//	(void)time(&now);
//
//	// write gnuplot plotting script commands
//
//	for (j = 1; j <= nN; j++) { // check for three-dimensional frame 
//		if (xyz[j].x != 0.0) X = 1;
//		if (xyz[j].y != 0.0) Y = 1;
//		if (xyz[j].z != 0.0) Z = 1;
//	}
//	if ((X && Y && Z) || D3_flag) {
//		D3 = ' '; D2 = '#';
//	}
//	else {
//		D3 = '#'; D2 = ' ';
//	}
//
//	if (lc <= 1) {	// open plotting script file for writing
//		if ((fpm = fopen(plotpath, "w")) == NULL) {
//			sprintf(errMsg, "\n  error: cannot open gnuplot script file: %s \n", plotpath);
//			errorMsg(errMsg);
//			exit(23);
//		}
//	}
//	else {	// open plotting script file for appending
//		if ((fpm = fopen(plotpath, "a")) == NULL) {
//			sprintf(errMsg, "\n  error: cannot open gnuplot script file: %s \n", plotpath);
//			errorMsg(errMsg);
//			exit(24);
//		}
//	}
//
//	// file name for deformed mesh data for load case "lc" 
//	if (lc >= 1 && anlyz)	sprintf(meshfl, "%sf.%03d", meshpath, lc);
//
//	// write header, plot-setup cmds, node label, and element label data
//
//	if (lc <= 1) {	// header & node number & element number labels
//
//		fprintf(fpm, "# FRAME3DD ANALYSIS RESULTS  http://frame3dd.sf.net/");
//		fprintf(fpm, " VERSION %s \n", VERSION);
//		fprintf(fpm, "# %s\n", title);
//		fprintf(fpm, "# %s", ctime(&now));
//		fprintf(fpm, "# G N U P L O T   S C R I P T   F I L E \n");
//		/* fprintf(fpm,"#  X=%d , Y=%d , Z=%d, D3=%d  \n", X,Y,Z,D3_flag); */
//
//		fprintf(fpm, "set autoscale\n");
//		fprintf(fpm, "unset border\n");
//		fprintf(fpm, "set pointsize 1.0\n");
//		fprintf(fpm, "set xtics; set ytics; set ztics; \n");
//		fprintf(fpm, "unset zeroaxis\n");
//		fprintf(fpm, "unset key\n");
//		fprintf(fpm, "unset label\n");
//		fprintf(fpm, "set size ratio -1    # 1:1 2D axis scaling \n");
//		fprintf(fpm, "# set view equal xyz # 1:1 3D axis scaling \n");
//
//		fprintf(fpm, "# NODE NUMBER LABELS\n");
//		for (j = 1; j <= nN; j++)
//			fprintf(fpm, "set label ' %d' at %12.4e, %12.4e, %12.4e\n",
//			j, xyz[j].x, xyz[j].y, xyz[j].z);
//
//		fprintf(fpm, "# ELEMENT NUMBER LABELS\n");
//		for (m = 1; m <= nE; m++) {
//			n1 = N1[m];	n2 = N2[m];
//			mx = 0.5 * (xyz[n1].x + xyz[n2].x);
//			my = 0.5 * (xyz[n1].y + xyz[n2].y);
//			mz = 0.5 * (xyz[n1].z + xyz[n2].z);
//			fprintf(fpm, "set label ' %d' at %12.4e, %12.4e, %12.4e\n",
//				m, mx, my, mz);
//		}
//
//		// 3D plot setup commands
//
//		fprintf(fpm, "%c set parametric\n", D3);
//		fprintf(fpm, "%c set view 60, 70, %5.2f \n", D3, scale);
//		fprintf(fpm, "%c set view equal xyz # 1:1 3D axis scaling \n", D3);
//		fprintf(fpm, "%c unset key\n", D3);
//		fprintf(fpm, "%c set xlabel 'x'\n", D3);
//		fprintf(fpm, "%c set ylabel 'y'\n", D3);
//		fprintf(fpm, "%c set zlabel 'z'\n", D3);
//		//	 fprintf(fpm,"%c unset label\n", D3 );
//
//	}
//
//	// different plot title for each load case
//
//	fprintf(fpm, "set title \"%s\\n", title);
//	fprintf(fpm, "analysis file: %s ", IN_file);
//	if (anlyz) {
//		fprintf(fpm, "  deflection exaggeration: %.1f ", exagg_static);
//		fprintf(fpm, "  load case %d of %d \"\n", lc, nL);
//	}
//	else {
//		fprintf(fpm, "  data check only \"\n");
//	}
//	fprintf(fpm, "unset clip; \nset clip one; set clip two\n");
//	fprintf(fpm, "set xyplane 0 \n"); // requires Gnuplot >= 4.6
//
//	// 2D plot command
//
//	fprintf(fpm, "%c plot '%s' u 2:3 t 'undeformed mesh' w lp ",
//		D2, meshpath);
//	if (!anlyz) fprintf(fpm, "lw %d lt 1 pt 6 \n", lw);
//	else fprintf(fpm, "lw 1 lt 5 pt 6, '%s' u 1:2 t 'load case %d of %d' w l lw %d lt 3\n", meshfl, lc, nL, lw);
//
//	// 3D plot command
//
//	fprintf(fpm, "%c splot '%s' u 2:3:4 t 'load case %d of %d' w lp ",
//		D3, meshpath, lc, nL);
//	if (!anlyz) fprintf(fpm, " lw %d lt 1 pt 6 \n", lw);
//	else fprintf(fpm, " lw 1 lt 5 pt 6, '%s' u 1:2:3 t 'load case %d of %d' w l lw %d lt 3\n", meshfl, lc, nL, lw);
//
//	if (lc < nL && anlyz)	fprintf(fpm, "pause -1\n");
//
//	fclose(fpm);
//
//	// write undeformed mesh data
//
//	if (lc <= 1) {
//		// open the undeformed mesh data file for writing
//		if ((fpm = fopen(meshpath, "w")) == NULL) {
//			sprintf(errMsg, "\n  error: cannot open gnuplot undeformed mesh data file: %s\n", meshpath);
//			errorMsg(errMsg);
//			exit(21);
//		}
//
//		fprintf(fpm, "# FRAME3DD ANALYSIS RESULTS  http://frame3dd.sf.net/");
//		fprintf(fpm, " VERSION %s \n", VERSION);
//		fprintf(fpm, "# %s\n", title);
//		fprintf(fpm, "# %s", ctime(&now));
//		fprintf(fpm, "# U N D E F O R M E D   M E S H   D A T A   (global coordinates)\n");
//		fprintf(fpm, "# Node        X            Y            Z \n");
//
//		for (m = 1; m <= nE; m++) {
//			n = N1[m];	// i = 6*(n-1);
//			fprintf(fpm, "%5d %12.4e %12.4e %12.4e \n",
//				n, xyz[n].x, xyz[n].y, xyz[n].z);
//			n = N2[m];	// i = 6*(n-1);
//			fprintf(fpm, "%5d %12.4e %12.4e %12.4e",
//				n, xyz[n].x, xyz[n].y, xyz[n].z);
//			fprintf(fpm, "\n\n\n");
//		}
//		fclose(fpm);
//	}
//
//	if (!anlyz) return; 	// no deformed mesh
//
//	// write deformed mesh data
//
//	// open the deformed mesh data file for writing 
//	if ((fpm = fopen(meshfl, "w")) == NULL) {
//		sprintf(errMsg, "\n  error: cannot open gnuplot deformed mesh data file %s \n", meshfl);
//		errorMsg(errMsg);
//		exit(22);
//	}
//
//	fprintf(fpm, "# FRAME3DD ANALYSIS RESULTS  http://frame3dd.sf.net/");
//	fprintf(fpm, " VERSION %s \n", VERSION);
//	fprintf(fpm, "# %s\n", title);
//	fprintf(fpm, "# L O A D  C A S E   %d  of   %d \n", lc, nL);
//	fprintf(fpm, "# %s", ctime(&now));
//	fprintf(fpm, "# D E F O R M E D   M E S H   D A T A ");
//	fprintf(fpm, "  deflection exaggeration: %.1f\n", exagg_static);
//	fprintf(fpm, "#       X-dsp        Y-dsp        Z-dsp\n");
//
//	// open the interior force data file for reading 
//	if (dx > 0.0 && anlyz) {
//		// file name for internal force data for load case "lc" 
//		sprintf(fnif, "%s%02d", infcpath, lc);
//		if ((fpif = fopen(fnif, "r")) == NULL) {
//			sprintf(errMsg, "\n  error: cannot open interior force data file: %s \n", fnif);
//			errorMsg(errMsg);
//			exit(20);
//		}
//	}
//
//	for (m = 1; m <= nE; m++) {	// write deformed shape data for each element
//
//		ch = 'a';
//
//		fprintf(fpm, "\n# element %5d \n", m);
//		if (dx < 0.0 && anlyz) {
//			cubic_bent_beam(fpm,
//				N1[m], N2[m], xyz, L[m], p[m], D, exagg_static);
//		}
//		if (dx > 0.0 && anlyz) {
//			while (ch != '@')	ch = getc(fpif);
//			sfrv = fscanf(fpif, "%d %d %d %f %f %f %f %f %f %d",
//				&frel, &n1, &n2, &x1, &y1, &z1, &x2, &y2, &z2, &nx);
//			if (sfrv != 10) sferr(fnif);
//			if (frel != m || N1[m] != n1 || N2[m] != n2) {
//				fprintf(stderr, " error in static_mesh parsing\n");
//				fprintf(stderr, "  frel = %d; m = %d; nx = %d \n", frel, m, nx);
//			}
//			/* debugging ... check mesh data
//			printf("  frel = %3d; m = %3d; n1 =%4d; n2 = %4d; nx = %3d L = %f \n", frel,m,n1,n2,nx,L[m] );
//			*/
//			while (ch != '~')	ch = getc(fpif);
//			force_bent_beam(fpm, fpif, fnif, nx,
//				N1[m], N2[m], xyz, L[m], p[m], D, exagg_static);
//		}
//
//	}
//
//	if (dx > 0.0 && anlyz) fclose(fpif);
//
//	fclose(fpm);
//
//	return;
//}
//
///*
//* CUBIC_BENT_BEAM  -  computes cubic deflection functions from end deflections
//* and end rotations.  Saves deflected shapes to a file.  These bent shapes
//* are exact for mode-shapes, and for frames loaded at their nodes.
//* 15 May 2009
//*/
//void StiffnessIO::GnuPltCubicBentBeam(
//	FILE *fpm, int n1, int n2, vec3 *xyz,
//	double L, float p, double *D, double exagg
//	){
//	double	t1, t2, t3, t4, t5, t6, t7, t8, t9, 	/* coord xfmn	*/
//		u1, u2, u3, u4, u5, u6, u7, u8, u9, u10, u11, u12,
//		*a, *b, **A,
//		s, v, w, dX, dY, dZ;
//	int	i1, i2, pd;
//	char	errMsg[MAXL];
//
//	A = dmatrix(1, 4, 1, 4);
//	a = dvector(1, 4);
//	b = dvector(1, 4);
//
//	coord_trans(xyz, L, n1, n2,
//		&t1, &t2, &t3, &t4, &t5, &t6, &t7, &t8, &t9, p);
//
//	i1 = 6 * (n1 - 1);	i2 = 6 * (n2 - 1);
//
//	/* compute end deflections in local coordinates */
//
//	u1 = exagg*(t1*D[i1 + 1] + t2*D[i1 + 2] + t3*D[i1 + 3]);
//	u2 = exagg*(t4*D[i1 + 1] + t5*D[i1 + 2] + t6*D[i1 + 3]);
//	u3 = exagg*(t7*D[i1 + 1] + t8*D[i1 + 2] + t9*D[i1 + 3]);
//
//	u4 = exagg*(t1*D[i1 + 4] + t2*D[i1 + 5] + t3*D[i1 + 6]);
//	u5 = exagg*(t4*D[i1 + 4] + t5*D[i1 + 5] + t6*D[i1 + 6]);
//	u6 = exagg*(t7*D[i1 + 4] + t8*D[i1 + 5] + t9*D[i1 + 6]);
//
//	u7 = exagg*(t1*D[i2 + 1] + t2*D[i2 + 2] + t3*D[i2 + 3]);
//	u8 = exagg*(t4*D[i2 + 1] + t5*D[i2 + 2] + t6*D[i2 + 3]);
//	u9 = exagg*(t7*D[i2 + 1] + t8*D[i2 + 2] + t9*D[i2 + 3]);
//
//	u10 = exagg*(t1*D[i2 + 4] + t2*D[i2 + 5] + t3*D[i2 + 6]);
//	u11 = exagg*(t4*D[i2 + 4] + t5*D[i2 + 5] + t6*D[i2 + 6]);
//	u12 = exagg*(t7*D[i2 + 4] + t8*D[i2 + 5] + t9*D[i2 + 6]);
//
//	/* curve-fitting problem for a cubic polynomial */
//
//	a[1] = u2;		b[1] = u3;
//	a[2] = u8;   		b[2] = u9;
//	a[3] = u6;		b[3] = -u5;
//	a[4] = u12;		b[4] = -u11;
//
//	u7 += L;
//	A[1][1] = 1.0;   A[1][2] = u1;   A[1][3] = u1*u1;   A[1][4] = u1*u1*u1;
//	A[2][1] = 1.0;   A[2][2] = u7;   A[2][3] = u7*u7;   A[2][4] = u7*u7*u7;
//	A[3][1] = 0.0;   A[3][2] = 1.;   A[3][3] = 2.*u1;   A[3][4] = 3.*u1*u1;
//	A[4][1] = 0.0;   A[4][2] = 1.;   A[4][3] = 2.*u7;   A[4][4] = 3.*u7*u7;
//	u7 -= L;
//
//	lu_dcmp(A, 4, a, 1, 1, &pd);		/* solve for cubic coef's */
//
//	if (!pd) {
//		sprintf(errMsg, " n1 = %d  n2 = %d  L = %e  u7 = %e \n", n1, n2, L, u7);
//		errorMsg(errMsg);
//		exit(30);
//	}
//
//	lu_dcmp(A, 4, b, 0, 1, &pd);		/* solve for cubic coef's */
//
//	// debug ... if deformed mesh exageration is too big, some elements
//	// may not be plotted.  
//	//fprintf( fpm, "# u1=%e  L+u7=%e, dx = %e \n",
//	//				u1, fabs(L+u7), fabs(L+u7-u1)/10.0); 
//	for (s = u1; fabs(s) <= 1.01*fabs(L + u7); s += fabs(L + u7 - u1) / 10.0) {
//
//		/* deformed shape in local coordinates */
//		v = a[1] + a[2] * s + a[3] * s*s + a[4] * s*s*s;
//		w = b[1] + b[2] * s + b[3] * s*s + b[4] * s*s*s;
//
//		/* deformed shape in global coordinates */
//		dX = t1*s + t4*v + t7*w;
//		dY = t2*s + t5*v + t8*w;
//		dZ = t3*s + t6*v + t9*w;
//
//		fprintf(fpm, " %12.4e %12.4e %12.4e\n",
//			xyz[n1].x + dX, xyz[n1].y + dY, xyz[n1].z + dZ);
//	}
//	fprintf(fpm, "\n\n");
//
//	free_dmatrix(A, 1, 4, 1, 4);
//	free_dvector(a, 1, 4);
//	free_dvector(b, 1, 4);
//
//	return;
//}
//
//
///*
//* FORCE_BENT_BEAM  -  reads internal frame element forces and deflections
//* from the internal force and deflection data file.
//* Saves deflected shapes to a file.  These bent shapes are exact.
//* Note: It would not be difficult to adapt this function to plot
//* internal axial force, shear force, torques, or bending moments.
//* 9 Jan 2010
//*/
//void StiffnessIO::GnuPltForceBentBeam(
//	FILE *fpm, FILE *fpif, char fnif[], int nx, int n1, int n2, vec3 *xyz,
//	double L, float p, double *D, double exagg
//	){
//	double	t1, t2, t3, t4, t5, t6, t7, t8, t9; 	/* coord xfmn	*/
//	double	xi, dX, dY, dZ;
//	float	x, Nx, Vy, Vz, Tx, My, Mz, Dx, Dy, Dz, Rx;
//	double	Lx, Ly, Lz;
//	int	n;
//	int	sfrv = 0;		/* *scanf return value	*/
//
//	Lx = xyz[n2].x - xyz[n1].x;
//	Ly = xyz[n2].y - xyz[n1].y;
//	Lz = xyz[n2].z - xyz[n1].z;
//
//	coord_trans(xyz, L, n1, n2,
//		&t1, &t2, &t3, &t4, &t5, &t6, &t7, &t8, &t9, p);
//
//	x = -1.0;
//	n = 0;
//	for (xi = 0; xi <= 1.01*L && n < nx; xi += 0.10*L) {
//
//		while (x < xi && n < nx) {
//			/* read the deformed shape in local coordinates */
//			sfrv = fscanf(fpif, "%f %f %f %f %f %f %f %f %f %f %f",
//				&x, &Nx, &Vy, &Vz, &Tx, &My, &Mz, &Dx, &Dy, &Dz, &Rx);
//			//		    printf("x = %12.4f\n", x );		/* debug */
//			if (sfrv != 11) sferr(fnif);
//			++n;
//		}
//
//		/* exaggerated deformed shape in global coordinates */
//		dX = exagg * (t1*Dx + t4*Dy + t7*Dz);
//		dY = exagg * (t2*Dx + t5*Dy + t8*Dz);
//		dZ = exagg * (t3*Dx + t6*Dy + t9*Dz);
//
//		fprintf(fpm, " %12.4e %12.4e %12.4e\n",
//			xyz[n1].x + (x / L)*Lx + dX,
//			xyz[n1].y + (x / L)*Ly + dY,
//			xyz[n1].z + (x / L)*Lz + dZ);
//
//		//		printf("...  x = %7.3f  n = %3d  Dx = %10.3e   Dy = %10.3e   Dz = %10.3e \n", x,n,Dx,Dy,Dz ); /* debug */
//		//		printf("                           dX = %10.3e   dY = %10.3e   dZ = %10.3e \n", dX,dY,dZ ); /* debug */
//
//	}
//
//	fprintf(fpm, "\n\n");
//
//	return;
//}

void StiffnessIO::Debug(int verbose)
{
	// path define
	char	IN_file[FILENMAX],	// the input  data filename
		OUT_file[FILENMAX],	// the output data filename
		title[MAXL],		// the title of the analysis
		errMsg[MAXL],		// the text of an error message
		meshpath[FRAME3DD_PATHMAX] = "EMPTY_MESH", // mesh data path
		plotpath[FRAME3DD_PATHMAX] = "EMPTY_PLOT", // plot file path
		infcpath[FRAME3DD_PATHMAX] = "EMPTY_INFC", // int  file path
		modepath[FRAME3DD_PATHMAX] = "EMPTY_MODE"; // mode data path
	char strippedInputFile[FRAME3DD_PATHMAX] = "EMPTY_TEMP"; // temp data path

	FILE	*fp;		// input and output file

	vec3	xyz;		// X,Y,Z node coordinates (global)

	VX	rj,					// node size radius, for finite sizes
		Ax, Asy, Asz,		// cross section areas, incl. shear
		Jx, Iy, Iz,			// section inertias		
		E, G,				// elastic modulus and shear moduli
		p;					// roll of each member, radians	
	std::vector<MX> U,			// uniform distributed member loads
					W,			// trapizoidal distributed member loads
					P,			// member concentrated loads	
					T;			// member temperature  loads

	MX	Dp;			// prescribed node displacements
	VX	d, EMs,			// member densities and extra inertia
		NMs, 			// mass of a node
		NMx, NMy, NMz,	// inertia of a node in global coord	
		gX,			// gravitational acceleration in global X 
		gY,			// gravitational acceleration in global Y
		gZ;			// gravitational acceleration in global Z

	int pan = 1.0,				// >0: pan during animation; 0: don't
		scale = 1.0,			// zoom scale for 3D plotting in Gnuplot
		dx = 1.0;				// x-increment for internal force data

	MX			K;					// equilibrium stiffness matrix
	double		traceK = 0.0;		// trace of the global stiffness matrix
	MX			M;					// global mass matrix
	double		traceM = 0.0;		// trace of the global mass matrix

	std::vector<MX>		eqF_mech,	// equivalent end forces from mechenism loads global
						eqF_temp;	// equivalent end forces from temp loads global
	std::vector<VX>		F_mech,		// mechanical load vectors, all load cases	
						F_temp,		// thermal load vectors, all load cases
						F, 			// total load vectors for a load case
						R,			// total reaction force vector
						dR,			// incremental reaction force vector 
						D,			// displacement vector
						dD,			// incremental displacement vector
						//dDdD = 0.0,	// dD' * dD
						dF;			// equilibrium error in nonlinear anlys
	VX			L,			// node-to-node length of each element
				Le;			// effcve lngth, accounts for node size

	MX				Q;					// local member node end-forces
	double			tol = 1.0e-9,		// tolerance for modal convergence
					shift = 0.0,		// shift-factor for rigid-body-modes
					struct_mass,		// mass of structural system	
					total_mass;			// total structural mass and extra mass 
	
	double	*f = NULL,			// resonant frequencies	
			**V = NULL,			// resonant mode-shapes
			rms_resid = 1.0,	// root mean square of residual displ. error
			error = 1.0,		// rms equilibrium error and reactions
			Cfreq = 0.0,		// frequency used for Guyan condensation
			**Kc, **Mc,			// condensed stiffness and mass matrices
			exagg_static = 10,	// exaggerate static displ. in mesh data
			exagg_modal = 10;	// exaggerate modal displ. in mesh data

	// peak internal forces, moments, and displacments
	// in each frame element and each load case 
	double	**pkNx, **pkVy, **pkVz, **pkTx, **pkMy, **pkMz,
		**pkDx, **pkDy, **pkDz, **pkRx, **pkSy, **pkSz;

	int	nN = 0,		// number of Nodes
		nE = 0,		// number of frame Elements
		nL = 0, lc = 0,	// number of Load cases
		DoF = 0, i, j,	// number of Degrees of Freedom
		nR = 0;		// number of restrained nodes
	VXi nD,			// number of prescribed nodal displ'nts
		nF,			// number of loaded nodes
		nU,			// number of members w/ unifm dist loads
		nW,			// number of members w/ trapz dist loads
		nP,			// number of members w/ conc point loads
		nT;			// number of members w/ temp. changes
	int	nI = 0,		// number of nodes w/ extra inertia
		nX = 0,		// number of elemts w/ extra mass
		nC = 0;		// number of condensed nodes
	VXi N1, N2;	// begin and end node numbers

	int	shear = 0,	// indicates shear deformation
	geom = 0,		// indicates  geometric nonlinearity
	anlyz = 1;		// 1: stiffness analysis, 0: data check	
	
	VXi	q, r;
	int	sumR,		// reaction data, total no. of reactions
		nM = 0,		// number of desired modes
		Mmethod,	// 1: Subspace Jacobi, 2: Stodola
		nM_calc,	// number of modes to calculate
		lump = 1,	// 1: lumped, 0: consistent mass matrix
		iter = 0,	// number of iterations	
		ok = 1,		// number of (-ve) diag. terms of L D L'
		anim[128],	// the modes to be animated
		Cdof = 0,	// number of condensed degrees o freedom
		Cmethod = 0,	// matrix condensation method
		*c = NULL,		// vector of DoF's to condense
		*m = NULL,		// vector of modes to condense
		filetype = 0,	// 1 if .CSV, 2 if file is Matlab
		debug = 0,		// 1: debugging screen output, 0: none
		axial_strain_warning = 0, // 0: "ok", 1: strain > 0.001
		ExitCode = 0;	// error code returned by Frame3DD

	int	shear_flag = -1,		//   over-ride input file value	
		geom_flag = -1,			//   over-ride input file value
		anlyz_flag = -1,		//   over-ride input file value
		D3_flag = -1,			//   over-ride 3D plotting check
		lump_flag = -1,			//   over-ride input file value
		modal_flag = -1,		//   over-ride input file value	
		write_matrix = -1,		//   write stiffness and mass matrix
		axial_sign = -1,		//   suppress 't' or 'c' in output data
		condense_flag = -1;		// over-ride input file value	

	int	sfrv = 0;		// *scanf return value for err checking

	double	exagg_flag = -1.0,	// over-ride input file value
			tol_flag = -1.0,	// over-ride input file value
			shift_flag = -1.0;	// over-ride input file value

	float	pan_flag = -1.0; // over-ride input file value

	char	extn[16];	// Input Data file name extension

	ParseOptions(IN_file, OUT_file,
		&shear_flag, &geom_flag, &anlyz_flag, &exagg_flag,
		&D3_flag,
		&lump_flag, &modal_flag, &tol_flag, &shift_flag,
		&pan_flag, &write_matrix, &axial_sign, &condense_flag,
		&verbose, &debug);

	//This is deprecated
	//string in_file = "F:\\FiberPrintProject\\TestData\\Frame3ddData\\" + string(IN_file);
	//fp = fopen(in_file.c_str(), "r");

	/* open the input data file */

	fp = fopen("F:\\FiberPrintProject\\TestData\\Frame3ddData\\exE.3dd", "r");
	if (fp == NULL)
	{ 
		/* open input data file */
		sprintf(errMsg, "\n ERROR: cannot open input data file '%s'\n", IN_file);
		errorMsg(errMsg);
		exit(11);
	}

	//OutputPath("frame3dd.3dd", strippedInputFile, FRAME3DD_PATHMAX, NULL);
	sprintf(strippedInputFile, "F:\\FiberPrintProject\\TestData\\Frame3ddData\\frame3dd.3dd");

	ParseInput(fp, strippedInputFile);	/* strip comments from input data */
	fclose(fp);

	if ((fp = fopen(strippedInputFile, "r")) == NULL) 
	{ /* open stripped input file */
		sprintf(errMsg, "\n ERROR: cannot open stripped input data file '%s'\n", strippedInputFile);
		errorMsg(errMsg);
		exit(13);
	}

	StiffnessGetline(fp, title, MAXL);
	if (verbose) 
	{	
		/*  display analysis title */
		fprintf(stdout, "\n");
		fprintf(stdout, " ** %s ** \n", title);
		color(0);
		fprintf(stdout, "\n");
	}
	
	if (verbose) 
	{	/*  display analysis title */
		fprintf(stdout, " ** %s ** \n", title);
	}

	sfrv = fscanf(fp, "%d", &nN);		/* number of nodes	*/

	if (verbose) 
	{	
		/* display nN */
		fprintf(stdout, " number of nodes ");
		dots(stdout, 36);	
		fprintf(stdout, " nN =%4d ", nN);
	}

	rj.resize(nN);		/* rigid radius around each node */
	rj.setZero();

	xyz.resize(nN) ;	/* node coordinates */

	ReadNodeData(fp, nN, xyz, rj);
	if (verbose){ printf("\nNode Reading completed.\n");}

	DoF = 6 * nN;		/* total number of degrees of freedom	*/

	q.resize(DoF);	/* allocate memory for reaction data ... */
	r.resize(DoF);	/* allocate memory for reaction data ... */
	
	ReadReactionData(fp, DoF, nN, nR, q, r, sumR, verbose);
	if (verbose){ printf(" \nReaction Data Reading complete\n"); }

	sfrv = fscanf(fp, "%d", &nE);	/* number of frame elements	*/
	if (sfrv != 1)	sferr("nE value for number of frame elements");

	if (verbose) 
	{	
		/* display nE */
		fprintf(stdout, " number of frame elements");
		dots(stdout, 28);
		fprintf(stdout, " nE =%4d\n", nE);
	}

	if (nN > nE + 1)
	{	/* not enough elements */
		fprintf(stderr, "\n  warning: %d nodes and %d members...", nN, nE);
		fprintf(stderr, " not enough elements to connect all nodes.\n");
	}

	/* allocate memory for frame elements ... */
	L.resize(nE);	/* length of each element		*/
	Le.resize(nE);	/* effective length of each element	*/

	N1.resize(nE);	/* node #1 of each element		*/
	N2.resize(nE);	/* node #2 of each element		*/
	N1.setZero();
	N2.setZero();

	Ax.resize(nE);	/* cross section area of each element	*/
	Asy.resize(nE);	/* shear area in local y direction 	*/
	Asz.resize(nE);	/* shear area in local z direction	*/
	Jx.resize(nE);	/* torsional moment of inertia 		*/
	Iy.resize(nE);	/* bending moment of inertia about y-axis */
	Iz.resize(nE);	/* bending moment of inertia about z-axis */

	E.resize(nE);	/* frame element Young's modulus	*/
	G.resize(nE);	/* frame element shear modulus		*/
	p.resize(nE);	/* element rotation angle about local x axis */
	d.resize(nE);	/* element mass density			*/

	ReadFrameElementData(fp, nN, nE, xyz, rj, L, Le, N1, N2,
		Ax, Asy, Asz, Jx, Iy, Iz, E, G, p, d, verbose);
	if (verbose) 	fprintf(stdout, " Reading Frame Element Data complete\n");

	ReadRunData(fp, OUT_file, &shear, shear_flag, &geom, geom_flag,
		meshpath, plotpath, infcpath,
		&exagg_static, exagg_flag, &scale, &dx,
		&anlyz, anlyz_flag, debug);
	getchar();
}

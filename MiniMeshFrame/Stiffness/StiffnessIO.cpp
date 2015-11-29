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

///*
//* PARSE_OPTIONS -  parse command line options
//* command line options over-ride values in the input data file
//* 04 Mar 2009, 22 Sep 2009
//*/
//void StiffnessIO::ParseOptions(
//	char IN_file[], char OUT_file[],
//	int *shear_flag,
//	int *geom_flag,
//	int *anlyz_flag,
//	double *exagg_flag,
//	int *D3_flag,
//	int *lump_flag,
//	int *modal_flag,
//	double *tol_flag,
//	double *shift_flag,
//	float *pan_flag,
//	int *write_matrix,
//	int *axial_sign,
//	int *condense_flag,
//	int *verbose,
//	int *debug
//	)
//{
//	char	option;
//	char	errMsg[MAXL];
//	int		sfrv = 0;		/* *scanf return value	*/
//
//	/* default values */
//
//	*shear_flag = *geom_flag = *anlyz_flag = *lump_flag = *modal_flag = -1;
//	*exagg_flag = *tol_flag = *shift_flag = -1.0;
//	*D3_flag = 0;
//	*pan_flag = -1.0;
//	*condense_flag = -1;
//	*write_matrix = 0;
//	*axial_sign = 1;
//	*debug = 0; *verbose = 1;
//
//	strcpy(IN_file, "\0");
//	strcpy(OUT_file, "\0");
//
//	/* set up file names for the the input data and the output data */
//
//	printf(" Please enter the  input data file name: ");
//	//sfrv = scanf("%s", IN_file);
//	sprintf(IN_file, "exE.3dd");
//	//if (sfrv != 1) sferr("IN_file");
//
//	printf(" Please enter the output data file name: ");
//	//sfrv = scanf("%s", OUT_file);
//	sprintf(IN_file, "exE.out");
//
//	//if (sfrv != 1) sferr("OUT_file");
//
//	if (strcmp(IN_file, "\0") == 0) {
//		fprintf(stderr, " Please enter the  input data file name: ");
//		sfrv = scanf("%s", IN_file);
//		if (sfrv != 1) sferr("IN_file");
//		fprintf(stderr, " Please enter the output data file name: ");
//		sfrv = scanf("%s", OUT_file);
//		if (sfrv != 1) sferr("OUT_file");
//	}
//	if (strcmp(IN_file, "\0") != 0 && strcmp(OUT_file, "\0") == 0) {
//		strcpy(OUT_file, IN_file);
//		strcat(OUT_file, ".out");
//	}
//	return;
//}


/*
* OUTPUT_PATH
* return path for output files using specific output directory
*/
void StiffnessIO::OutputPath(const char *fname, char fullpath[], const int len, char *default_outdir) {
	int res;
	assert(fname != NULL);

	const char *outdir;
	outdir = getenv("FRAME3DD_OUTDIR");
	if (outdir == NULL) 
	{
		if (default_outdir == NULL)
			outdir = TempDir();
		else
			outdir = default_outdir;
	}

	res = sprintf(fullpath, "%s%c%s", outdir, '\\', fname);

	if (res > len) {
		errorMsg("ERROR: unable to construct output filename: overflow.\n");
		exit(16);
	}

	//res = sprintf(fullpath, "%s%s", "F:\\FiberPrintProject\\ResultData\\Frame3dd_data\\", fname);

	printf("Output file path generated: %s\n",fullpath); /* debug */
}


/*
* READ_RUN_DATA  -  read information for analysis
* 29 Dec 2008
*/
void StiffnessIO::ReadRunData(
	char OUT_file[],	 /**< output data file name							*/
	char meshpath[],	 /**< file name for mesh data output				*/
	char plotpath[],	 /**< file name for Gnuplot script					*/
	int  debug
	)
{
	int	full_len = 0, len = 0, i;
	char	base_file[96] = "EMPTY_BASE";
	char	mesh_file[96] = "EMPTY_MESH";
	int	sfrv = 0;		/* *scanf return value */

	strcpy(base_file, OUT_file);
	while (base_file[len++] != '\0')
	{
		/* the length of the base_file */;
		full_len = len;
	}

	while (base_file[len--] != '.' && len > 0)
	{
		/* find the last '.' in base_file */;
		if (len == 0)	len = full_len;
	}
		
	base_file[++len] = '\0';	/* end base_file at the last '.' */

	// GnuPlot file
	strcpy(plotpath, base_file);
	strcat(plotpath, ".plt");

	//// Internal force file
	//strcpy(infcpath, base_file);
	//strcat(infcpath, ".if");

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

	if (debug) 
	{
		fprintf(stderr, "OUT_FILE  = %s \n", OUT_file);
		fprintf(stderr, "BASE_FILE = %s \n", base_file);
		fprintf(stderr, "PLOTPATH  = %s \n", plotpath);
		fprintf(stderr, "MESH_FILE = %s \n", mesh_file);
		fprintf(stderr, "MESHPATH  = %s \n", meshpath);
	}

	return;
}


/*
* GnuPltStaticMesh  - create mesh data of deformed and undeformed mesh  25/Nov/2015
* use gnuplot
* useful gnuplot options: unset xtics ytics ztics border view key
* This function illustrates how to read the internal force output data file.
* The internal force output data file contains all the information required
* to plot deformed meshes, internal axial force, internal shear force, internal
* torsion, and internal bending moment diagrams.
*/
void StiffnessIO::GnuPltStaticMesh(
	char OUT_file[],
	char meshpath[], char plotpath[],
	char *title, int nN, int nE, int nL, int lc, int DoF,
	vec3 &xyz, VX &L,
	VXi &N1, VXi &N2, VX &p, VX &D,
	double exagg_static, int D3_flag, int anlyz, float scale
	)
{
	FILE	*fpif = NULL, *fpm = NULL;
	double	mx, my, mz; /* coordinates of the frame element number labels */
	char	fnif[FILENMAX], meshfl[FILENMAX],
		D2 = '#', D3 = '#',	/* indicates plotting in 2D or 3D	*/
		errMsg[MAXL],
		ch = 'a';
	int	sfrv = 0,		/* *scanf return value						  */
		frel, nx,		/* frame element number, number of increments */
		n1, n2;			/* node numbers								  */
	float	x1, y1, z1,	/* coordinates of node n1		*/
		x2, y2, z2;		/* coordinates of node n2		*/
	int	j = 0, m = 0, n = 0,
		X = 0, Y = 0, Z = 0,
		lw = 1;		/*  line width of deformed mesh		*/
	time_t  now;		/* modern time variable type		*/

	(void)time(&now);

	// write gnuplot plotting script commands

	for (j = 0; j < nN; j++) 
	{ 
		// check for three-dimensional frame 
		if (xyz[j][0] != 0.0) X = 1;
		if (xyz[j][1] != 0.0) Y = 1;
		if (xyz[j][2] != 0.0) Z = 1;
	}
	
	if ((X && Y && Z) || D3_flag) 
	{
		D3 = ' '; D2 = '#';
	}
	else 
	{
		D3 = '#'; D2 = ' ';
	}

	// open plotting script file for writing
	if ((fpm = fopen(plotpath, "w")) == NULL) 
	{
		sprintf(errMsg, "\n  error: cannot open gnuplot script file: %s \n", plotpath);
		errorMsg(errMsg);
		exit(23);
	}

	// file name for deformed mesh data 
	sprintf(meshfl, "%sf.", meshpath);

	// write header, plot-setup cmds, node label, and element label data

	// header & node number & element number labels

	fprintf(fpm, "# FIBERPRINT FRAME STRUCTUAL ANALYSIS RESULTS  GCL@USTC");
	fprintf(fpm, " VERSION %s \n", VERSION);
	fprintf(fpm, "# %s\n", title);
	fprintf(fpm, "# %s", ctime(&now));
	fprintf(fpm, "# G N U P L O T   S C R I P T   F I L E \n");
	/* fprintf(fpm,"#  X=%d , Y=%d , Z=%d, D3=%d  \n", X,Y,Z,D3_flag); */

	fprintf(fpm, "set autoscale\n");
	fprintf(fpm, "unset border\n");
	fprintf(fpm, "set pointsize 1.0\n");
	fprintf(fpm, "set xtics; set ytics; set ztics; \n");
	fprintf(fpm, "unset zeroaxis\n");
	fprintf(fpm, "unset key\n");
	fprintf(fpm, "unset label\n");
	fprintf(fpm, "set size ratio -1    # 1:1 2D axis scaling \n");
	fprintf(fpm, "# set view equal xyz # 1:1 3D axis scaling \n");

	fprintf(fpm, "# NODE NUMBER LABELS\n");
	for (j = 0; j < nN; j++)
		fprintf(fpm, "set label ' %d' at %12.4e, %12.4e, %12.4e\n",
		j, xyz[j][0], xyz[j][1], xyz[j][2]);

	fprintf(fpm, "# ELEMENT NUMBER LABELS\n");
	for (m = 0; m < nE; m++) 
	{
		n1 = N1[m];	n2 = N2[m];
		mx = 0.5 * (xyz[n1][0] + xyz[n2][0]);
		my = 0.5 * (xyz[n1][1] + xyz[n2][1]);
		mz = 0.5 * (xyz[n1][2] + xyz[n2][2]);
		fprintf(fpm, "set label ' %d' at %12.4e, %12.4e, %12.4e\n",
			m, mx, my, mz);

		// 3D plot setup commands

		fprintf(fpm, "%c set parametric\n", D3);
		fprintf(fpm, "%c set view 60, 70, %5.2f \n", D3, scale);
		fprintf(fpm, "%c set view equal xyz # 1:1 3D axis scaling \n", D3);
		fprintf(fpm, "%c unset key\n", D3);
		fprintf(fpm, "%c set xlabel 'x'\n", D3);
		fprintf(fpm, "%c set ylabel 'y'\n", D3);
		fprintf(fpm, "%c set zlabel 'z'\n", D3);
		//	 fprintf(fpm,"%c unset label\n", D3 );

	}

	// different plot title for each load case

	fprintf(fpm, "set title \"%s\\n", title);
	//fprintf(fpm, "analysis file: %s ", IN_file);

	if (anlyz) 
	{
		fprintf(fpm, "  deflection exaggeration: %.1f ", exagg_static);
	}

	fprintf(fpm, "unset clip; \nset clip one; set clip two\n");
	fprintf(fpm, "set xyplane 0 \n"); // requires Gnuplot >= 4.6

	// 2D plot command

	fprintf(fpm, "%c plot '%s' u 2:3 t 'undeformed mesh' w lp ",
		D2, meshpath);
	if (!anlyz) fprintf(fpm, "lw %d lt 1 pt 6 \n", lw);
	else fprintf(fpm, "lw 1 lt 5 pt 6, '%s' u 1:2 t 'load case %d of %d' w l lw %d lt 3\n", meshfl, 1, 1, lw);

	// 3D plot command

	fprintf(fpm, "%c splot '%s' u 2:3:4 t 'load case %d of %d' w lp ",
		D3, meshpath, 1, 1);
	if (!anlyz) fprintf(fpm, " lw %d lt 1 pt 6 \n", lw);
	else fprintf(fpm, " lw 1 lt 5 pt 6, '%s' u 1:2:3 t 'load case %d of %d' w l lw %d lt 3\n", meshfl, 1, 1, lw);

	if (anlyz)	fprintf(fpm, "pause -1\n");

	fclose(fpm);

	// write undeformed mesh data

	// open the undeformed mesh data file for writing
	if ((fpm = fopen(meshpath, "w")) == NULL) 
	{
		sprintf(errMsg, "\n  error: cannot open gnuplot undeformed mesh data file: %s\n", meshpath);
		errorMsg(errMsg);
		exit(21);
	}

	fprintf(fpm, "# FIBERPRINT FRAME STRUCTURAL ANALYSIS RESULTS  GCL@USTC");
	fprintf(fpm, " VERSION %s \n", VERSION);
	fprintf(fpm, "# %s\n", title);
	fprintf(fpm, "# %s", ctime(&now));
	fprintf(fpm, "# U N D E F O R M E D   M E S H   D A T A   (global coordinates)\n");
	fprintf(fpm, "# Node        X            Y            Z \n");

	for (m = 0; m < nE; m++) 
	{
		n = N1[m];	// i = 6*(n-1);
		fprintf(fpm, "%5d %12.4e %12.4e %12.4e \n",
			n, xyz[n][0], xyz[n][1], xyz[n][2]);
		n = N2[m];	// i = 6*(n-1);
		fprintf(fpm, "%5d %12.4e %12.4e %12.4e",
			n, xyz[n][0], xyz[n][1], xyz[n][2]);
		fprintf(fpm, "\n\n\n");
	}
	fclose(fpm);

	if (!anlyz) return; 	// no deformed mesh

	// write deformed mesh data

	// open the deformed mesh data file for writing 
	if ((fpm = fopen(meshfl, "w")) == NULL) 
	{
		sprintf(errMsg, "\n  error: cannot open gnuplot deformed mesh data file %s \n", meshfl);
		errorMsg(errMsg);
		exit(22);
	}

	fprintf(fpm, "# FIBERPRINT FRAME STRUCTURAL ANALYSIS RESULTS  GCL@USTC");
	fprintf(fpm, " VERSION %s \n", VERSION);
	fprintf(fpm, "# %s\n", title);
	fprintf(fpm, "# L O A D  C A S E   %d  of   %d \n", lc, nL);
	fprintf(fpm, "# %s", ctime(&now));
	fprintf(fpm, "# D E F O R M E D   M E S H   D A T A ");
	fprintf(fpm, "  deflection exaggeration: %.1f\n", exagg_static);
	fprintf(fpm, "#       X-dsp        Y-dsp        Z-dsp\n");

	// open the interior force data file for reading 
	//if (anlyz) 
	//{
	//	// file name for internal force data for load case "lc" 
	//	sprintf(fnif, "%s%02d", infcpath, lc);
	//	if ((fpif = fopen(fnif, "r")) == NULL) 
	//	{
	//		sprintf(errMsg, "\n  error: cannot open interior force data file: %s \n", fnif);
	//		errorMsg(errMsg);
	//		exit(20);
	//	}
	//}

	for (m = 0; m < nE; m++) 
	{	
		// write deformed shape data for each element

		ch = 'a';

		fprintf(fpm, "\n# element %5d \n", m);
		if (anlyz) 
		{
			GnuPltCubicBentBeam(fpm,
				N1[m], N2[m], xyz, L[m], p[m], D, exagg_static);
		}
		//if (dx > 0.0 && anlyz) {
		//	while (ch != '@')	ch = getc(fpif);
		//	sfrv = fscanf(fpif, "%d %d %d %f %f %f %f %f %f %d",
		//		&frel, &n1, &n2, &x1, &y1, &z1, &x2, &y2, &z2, &nx);
		//	if (sfrv != 10) sferr(fnif);
		//	if (frel != m || N1[m] != n1 || N2[m] != n2) {
		//		fprintf(stderr, " error in static_mesh parsing\n");
		//		fprintf(stderr, "  frel = %d; m = %d; nx = %d \n", frel, m, nx);
		//	}
		//	/* debugging ... check mesh data
		//	printf("  frel = %3d; m = %3d; n1 =%4d; n2 = %4d; nx = %3d L = %f \n", frel,m,n1,n2,nx,L[m] );
		//	*/
		//	while (ch != '~')	ch = getc(fpif);
		//	force_bent_beam(fpm, fpif, fnif, nx,
		//		N1[m], N2[m], xyz, L[m], p[m], D, exagg_static);
		//}

	}

	//if (dx > 0.0 && anlyz) fclose(fpif);

	fclose(fpm);

	return;
}

/*
* GnuPltCubicBentBeam  -  computes cubic deflection functions from end deflections
* and end rotations.  Saves deflected shapes to a file.  These bent shapes
* are exact for mode-shapes, and for frames loaded at their nodes.
* Nov/25/2015
*/
void StiffnessIO::GnuPltCubicBentBeam(
	FILE *fpm, int n1, int n2, vec3 &xyz,
	double L, float p, VX &D, double exagg
	)
{
	double	t1, t2, t3, t4, t5, t6, t7, t8, t9, 	/* coord transf matrix entries	*/
		u1, u2, u3, u4, u5, u6, u7, u8, u9, u10, u11, u12,
		s, v, w, dX, dY, dZ;
	int	i1, i2;
	int info;
	char	errMsg[MAXL];

	MX A(4,4);
	VX a(4);
	VX b(4);

	trsf_.CreateTransMatrix(xyz, L, n1, n2,
		t1, t2, t3, t4, t5, t6, t7, t8, t9, p);

	i1 = 6 * (n1 - 1);	i2 = 6 * (n2 - 1);

	/* compute end deflections in local coordinates */

	u1 = exagg*(t1*D[i1 + 0] + t2*D[i1 + 1] + t3*D[i1 + 2]);
	u2 = exagg*(t4*D[i1 + 0] + t5*D[i1 + 1] + t6*D[i1 + 2]);
	u3 = exagg*(t7*D[i1 + 0] + t8*D[i1 + 1] + t9*D[i1 + 2]);

	u4 = exagg*(t1*D[i1 + 3] + t2*D[i1 + 4] + t3*D[i1 + 5]);
	u5 = exagg*(t4*D[i1 + 3] + t5*D[i1 + 4] + t6*D[i1 + 5]);
	u6 = exagg*(t7*D[i1 + 3] + t8*D[i1 + 4] + t9*D[i1 + 5]);

	u7 = exagg*(t1*D[i2 + 0] + t2*D[i2 + 1] + t3*D[i2 + 2]);
	u8 = exagg*(t4*D[i2 + 0] + t5*D[i2 + 1] + t6*D[i2 + 2]);
	u9 = exagg*(t7*D[i2 + 0] + t8*D[i2 + 1] + t9*D[i2 + 2]);

	u10 = exagg*(t1*D[i2 + 3] + t2*D[i2 + 4] + t3*D[i2 + 5]);
	u11 = exagg*(t4*D[i2 + 3] + t5*D[i2 + 4] + t6*D[i2 + 5]);
	u12 = exagg*(t7*D[i2 + 3] + t8*D[i2 + 4] + t9*D[i2 + 5]);

	/* curve-fitting problem for a cubic polynomial */

	a[0] = u2;		b[0] = u3;
	a[1] = u8;   	b[1] = u9;
	a[2] = u6;		b[2] = -u5;
	a[3] = u12;		b[3] = -u11;

	u7 += L;
	A(0,0) = 1.0;   A(0,1) = u1;   A(0,2) = u1*u1;   A(0,3) = u1*u1*u1;
	A(1,0) = 1.0;   A(1,1) = u7;   A(1,2) = u7*u7;   A(1,3) = u7*u7*u7;
	A(2,0)= 0.0;    A(2,1) = 1.;   A(2,2) = 2.*u1;   A(2,3) = 3.*u1*u1;
	A(3,0) = 0.0;   A(3,1) = 1.;   A(3,2) = 2.*u7;   A(3,3) = 3.*u7*u7;
	u7 -= L;

	solver_.LUDecomp(A, 4, a, 1, 1, info);		/* solve for cubic coef's */

	if (!info)
	{
		sprintf(errMsg, " n1 = %d  n2 = %d  L = %e  u7 = %e \n", n1, n2, L, u7);
		errorMsg(errMsg);
		exit(30);
	}

	solver_.LUDecomp(A, 4, b, 0, 1, info);		/* solve for cubic coef's */

	// debug ... if deformed mesh exageration is too big, some elements
	// may not be plotted.  
	//fprintf( fpm, "# u1=%e  L+u7=%e, dx = %e \n",
	//				u1, fabs(L+u7), fabs(L+u7-u1)/10.0); 
	for (s = u1; fabs(s) <= 1.01*fabs(L + u7); s += fabs(L + u7 - u1) / 10.0) 
	{

		/* deformed shape in local coordinates */
		v = a[0] + a[1] * s + a[2] * s*s + a[3] * s*s*s;
		w = b[0] + b[1] * s + b[2] * s*s + b[3] * s*s*s;

		/* deformed shape in global coordinates */
		dX = t1*s + t4*v + t7*w;
		dY = t2*s + t5*v + t8*w;
		dZ = t3*s + t6*v + t9*w;

		fprintf(fpm, " %12.4e %12.4e %12.4e\n",
			xyz[n1][0]+ dX, xyz[n1][1] + dY, xyz[n1][2] + dZ);
	}
	fprintf(fpm, "\n\n");


	return;
}


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

void StiffnessIO::WriteInputData(DualGraph *ptr_dualgraph, FiberPrintPARM *ptr_parm, int cut_count)
{
	FILE	*fp;
	char OUT_file[FILENMAX];
	char OUT_path[FILENMAX];
	string title_s = "FiberPrint Test File -- Cut" + to_string(cut_count) + " -- static analysis (N,mm,Ton)\n";
	char errMsg[512];

	string str = "FiberTest_Cut" + to_string(cut_count) + ".3dd";
	sprintf_s(OUT_file, "%s", str.c_str());

	OutputPath(OUT_file, OUT_path, FRAME3DD_PATHMAX, NULL);

	if ((fp = fopen(OUT_path, "w")) == NULL)
	{
		sprintf_s(errMsg, "\n ERROR: cannot open .3dd transfer data file '%s'", OUT_path);
		errorMsg(errMsg);
		exit(11);
	}

	fprintf(fp, title_s.c_str());
	fprintf(fp, "\n");

	int nN = ptr_dualgraph->SizeOfFaceList();
	int nE = ptr_dualgraph->SizeOfVertList();
	WireFrame *ptr_wf = ptr_dualgraph->ptr_frame_;
	std::vector<WF_edge*> wf_edge_list   = *ptr_wf->GetEdgeList();
	std::vector<DualFace*> dual_face_list = *ptr_dualgraph->GetFaceList();

	double r = ptr_parm->radius_;
	double nr = 0.0;
	double density = ptr_parm->density_;
	double g = ptr_parm->g_;
	double G = ptr_parm->shear_modulus_;
	double E = ptr_parm->youngs_modulus_;
	double v = ptr_parm->poisson_ratio_;

	// Write node data
	fprintf(fp, "%d					# number of nodes\n", nN);
	fprintf(fp, "#.node  x       y       z       r\n");
	fprintf(fp, "#        mm      mm      mm      m\n\n");
	for (int i = 0; i < nN; i++)
	{
		// using face id in dualgraph to represent node in orig graph
		int id = dual_face_list[i]->orig_id();
		fprintf(fp, "%d	 %.4f  %.4f  %.4f  %.4f\n", i + 1,
			ptr_wf->GetPosition(id).x(), ptr_wf->GetPosition(id).y(), ptr_wf->GetPosition(id).z(),0.0);
	}

	// Write nodes with reactions
	int nR = 0;		// Number of restrained nodes
	std::vector<int>	res_index;
	for (int i = 0; i < nN; i++)
	{
		int id = dual_face_list[i]->orig_id();
		if (ptr_wf->isFixed(id))
		{
			nR++;
			res_index.push_back(i);
		}
	}
	fprintf(fp, "\n");
	fprintf(fp, "%d					# number of nodes with reaction\n", nR);
	fprintf(fp, "#.node  x  y  z  xx  yy  zz			1=fixed, 0=free\n");
	for (int i = 0; i < nR; i++)
	{
		fprintf(fp, "%d  1  1  1  1  1  1\n", res_index[i]);
	}

	// Write Frame Element data
	double	Ax = F_PI * r * r;
	double	Asy = Ax * (6 + 12 * v + 6 * v*v) / (7 + 12 * v + 4 * v*v);
	double	Asz = Asy;
	double	Jxx = 0.5 * M_PI * r * r * r * r;
	double	Iyy = Jxx / 2;
	double	Izz = Iyy;

	fprintf(fp, "\n");
	fprintf(fp, "%d					# number of frame element\n", nE);
	fprintf(fp, "#.e n1 n2 Ax    Asy     Asz     Jxx     Iyy     Izz     E		G		roll	density\n");
	fprintf(fp, "#   .  .  mm^2  mm^2    mm^2    mm^4    mm^4    mm^4    MPa		MPa		deg		T/mm^3\n");
	for (int i = 0; i < nE; i++)
	{
		int e_id = ptr_dualgraph->e_orig_id(i);
		int orig_id   = wf_edge_list[e_id]->pvert_->ID();
		int orig_id_2 = wf_edge_list[e_id]->ppair_->pvert_->ID();
		
		int id   = ptr_dualgraph->v_dual_id(orig_id);
		int id_1 = ptr_dualgraph->v_dual_id(orig_id_2);

		fprintf(fp, "%d %d %d	%f	%f	%f	%f	%f	%f	%f	%f	%f	%.14f\n",
			i + 1, id + 1, id_1 + 1,
			Ax, Asy, Asz, Jxx, Iyy, Izz, E, G, 0.0, density);
	}

	printf("\n\n");

	// parse option for stiffness matrix
	fprintf(fp, "%d				# 1: include shear deformation\n", 1);
	fprintf(fp, "%d				# 1: include geometric stiffness\n", 0);
	fprintf(fp, "%.1f				# exaggerate static mesh deformation\n", 10.0);
	fprintf(fp, "%.1f				# zoom scale for 3D plotting\n", 2.5);
	fprintf(fp, "%d				# x-axis increment for internal forces, mm\n", -1);
	fprintf(fp, "				# if dx is -1 then internal force calculation are skipped\n");

	// load case parsing
	fprintf(fp, "\n");
	fprintf(fp, "%d				# number of static load cases\n", 1);
	fprintf(fp, "				# Begin static Load Case 1 of 1\n");

	fprintf(fp, "\n");
	fprintf(fp, "# gravitational acceleration for self-weight loading (global)\n");
	fprintf(fp, "#.gX			 gY			     gZ\n");
	fprintf(fp, "#.mm/s^2		 mm/s^2			 mm/s^2\n");
	fprintf(fp, "  0			 0				 %.6f\n", g);

	fprintf(fp,"\n");
	fprintf(fp, "%d				# number of loaded nodes\n",				0);
	fprintf(fp, "%d				# number of uniform loads\n",				0);
	fprintf(fp, "%d				# number of trapezoidal loads\n",			0);
	fprintf(fp, "%d				# number of internal concentrated loads\n", 0);
	fprintf(fp, "%d				# number of temperature loads\n",			0);

	fprintf(fp, "%d				# number of nodes with prescribed displacement\n", nR);
	fprintf(fp, "#.node  X-disp1  Y-disp1  Z-disp1  X-rot'n  Y-rot'n  Z-rot'n\n");
	fprintf(fp, "#       mm		mm	   mm	radian	radian	radian\n");
	for (int i = 0; i < nR; i++)
	{
		fprintf(fp, "%d  0  0  0  0  0  0\n", res_index[i]);
	}

	fprintf(fp, "				# End static Load Case 1 of 1\n");

	// Dynamic Modes
	fprintf(fp, "%d				#number of dynamic modes\n",0);
	fprintf(fp, "# End of transfer data file for fiber test");
	
	fclose(fp);
}

/*
* SaveUpperMatrix - save a symmetric matrix of dimension [1..n][1..n]	Nov/26/2015
* to the named file, use only upper-triangular part
*/
void StiffnessIO::SaveUpperMatrix(char filename[], const MX &A, int n)
{
	FILE    *fp_m;
	int     i, j;
	time_t	now;

	if ((fp_m = fopen(filename, "w")) == NULL) 
	{
		printf(" error: cannot open file: %s \n", filename);
		exit(1016);
	}

	(void)time(&now);
	fprintf(fp_m, "%% filename: %s - %s\n", filename, ctime(&now));
	fprintf(fp_m, "%% type: matrix \n");
	fprintf(fp_m, "%% rows: %d\n", n);
	fprintf(fp_m, "%% columns: %d\n", n);
	for (i = 0; i < n; i++) 
	{
		for (j = 0; j < n; j++) 
		{
			if (i > j) 
			{
				if (fabs(A(j,i)) > 1.e-99)
				{
					fprintf(fp_m, "%21.12e", A(j,i));
				}
				else
				{
					fprintf(fp_m, "    0                ");
				}
			}
			else 
			{
				if (fabs(A(i, j)) > 1.e-99)
				{
					fprintf(fp_m, "%21.12e", A(i,j));
				}
				else
				{
					fprintf(fp_m, "    0                ");
				}
			}
		}
		fprintf(fp_m, "\n");
	}
	fclose(fp_m);
	return;

}

void StiffnessIO::SaveDisplaceVector(char filename[], const VX &D, int n, DualGraph *ptr_dual_graph)
{
	FILE    *fp;
	int     i, j;
	time_t	now;
	int nN = n / 6;
	vector<DualFace*> dual_face_list = *ptr_dual_graph->GetFaceList();
	VX tmp_D(D.size());

	if ((fp = fopen(filename, "w")) == NULL)
	{
		printf(" error: cannot open file: %s \n", filename);
		exit(1016);
	}

	(void)time(&now);
	fprintf(fp, "%% filename: %s - %s\n", filename, ctime(&now));
	fprintf(fp, "%% type: vector \n");
	fprintf(fp, "%% rows: %d\n", n);

	fprintf(fp, "\n");
	fprintf(fp, "NODE DISPLACEMENTS		(global)\n");
	fprintf(fp, "#.node		X-dsp	Y-dsp	Z-dsp	X-rot	Y-rot	Z-rot\n");
	for (i = 0; i < nN; i++)
	{
		/* i is the dual face id, convert it back to wf_vert id*/
		int v_id = dual_face_list[i]->orig_id();
		for (j = 0; j < 6; j++)
		{
			tmp_D[6 * v_id + j] = D[6 * i + j];
		}
	}

	for (i = 0; i < nN; i++)
	{
		fprintf(fp, "%d		%.6f		%.6f		%.6f		%.6f		%.6f		%.6f\n",
			i+1, tmp_D[6 * i], tmp_D[6 * i + 1], tmp_D[6 * i + 2], tmp_D[6 * i + 3], tmp_D[6 * i + 4], tmp_D[6 * i + 5]);
	}

	fclose(fp);
	return;

}

void StiffnessIO::Debug(int verbose)
{
	char OUT_file[FILENMAX],
		meshpath[FILENMAX],
		plotpath[FILENMAX];
	char *title = "fibTest";
	int debug = 1;

	sprintf_s(OUT_file, "%s", "fibTest.obj");

	ReadRunData(OUT_file, meshpath, plotpath, debug);

	getchar();
}

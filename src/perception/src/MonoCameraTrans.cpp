#include "MonoCameraTrans.h"


/* This cube root routine handles negative arguments (unlike cbrt).  */
double CBRT(double x) {
	if (x == 0)
		return 0;
	else if (x > 0)
		return pow(x, 1.0 / 3.0);
	else
		return -pow(-x, 1.0 / 3.0);
}

/*
 * This routine converts from undistorted to distorted sensor coordinates.
 * The technique involves algebraically solving the cubic polynomial

 * Ru = Rd * (1 + kappa1 * Rd**2)

 * using the Cardan method.

 * Note: for kappa1 < 0 the distorted sensor plane extends out to a maximum
 * barrel distortion radius of  Rd = sqrt (-1/(3 * kappa1)).

 * To see the logic used in this routine try graphing the above polynomial
 * for positive and negative kappa1's
 */
void MonoCameraTrans::undistorted_to_distorted_sensor_coord(double Xu, double Yu, double &Xd, double &Yd) {
	double Ru, Rd, lambda, c, d, Q, R, D, S, T, sinT, cosT;

	if (((Xu == 0) && (Yu == 0)) || (cc.kappa1 == 0)) {
		Xd = Xu;
		Yd = Yu;
		return;
	}

	Ru = hypot(Xu, Yu);	/* SQRT(Xu*Xu+Yu*Yu) */

	c = 1 / cc.kappa1;
	d = -c * Ru;

	Q = c / 3;
	R = -d / 2;
	D = CUB(Q) + SQR(R);

	if (D >= 0) {		/* one real root */
		D = SQRT(D);
		S = CBRT(R + D);
		T = CBRT(R - D);
		Rd = S + T;

		if (Rd < 0) {
			Rd = SQRT(-1 / (3 * cc.kappa1));
			//	 fprintf (stderr, "\nWarning: undistorted image point to distorted image point mapping limited by\n");
			//	 fprintf (stderr, " maximum barrel distortion radius of %lf\n", Rd);
			//	 fprintf (stderr, " (Xu = %lf, Yu = %lf) -> (Xd = %lf, Yd = %lf)\n\n",
			//		  Xu, Yu, Xu * Rd / Ru, Yu * Rd / Ru);
		}
	}
	else {			/* three real roots */
		D = SQRT(-D);
		S = CBRT(hypot(R, D));
		T = atan2(D, R) / 3;
		SINCOS(T, sinT, cosT);

		/* the larger positive root is 2*S*cos(T)   */
		/* the smaller positive root is -S*cos(T) + SQRT(3)*S*sin(T) */
		/* the negative root is   -S*cos(T) - SQRT(3)*S*sin(T) */
		Rd = -S * cosT + SQRT3 * S * sinT;	/* use the smaller positive root */
	}

	lambda = Rd / Ru;

	Xd = Xu * lambda;
	Yd = Yu * lambda;
}

void MonoCameraTrans::distorted_to_undistorted_sensor_coord(double Xd, double Yd, double &Xu, double &Yu) {
	double distortion_factor;

	/* convert from distorted to undistorted sensor plane coordinates */
	distortion_factor = 1 + cc.kappa1 * (SQR(Xd) + SQR(Yd));
	Xu = Xd * distortion_factor;
	Yu = Yd * distortion_factor;
}

/* This routine takes the position of a point in world coordinates [mm]
 * and determines the position of its image in image coordinates [pix].
 */
int MonoCameraTrans::world_coord_to_image_coord(double xw, double yw, double zw, double &Xf, double &Yf) {
	double xc, yc, zc, Xu, Yu, Xd, Yd;

	/* convert from world coordinates to camera coordinates */
	xc = cc.r1 * xw + cc.r2 * yw + cc.r3 * zw + cc.Tx;
	yc = cc.r4 * xw + cc.r5 * yw + cc.r6 * zw + cc.Ty;
	zc = cc.r7 * xw + cc.r8 * yw + cc.r9 * zw + cc.Tz;
	if (zc < 0)
		return 0;

	/* convert from camera coordinates to undistorted sensor plane coordinates */
	Xu = cc.f * xc / zc;
	Yu = cc.f * yc / zc;

	/* convert from undistorted to distorted sensor plane coordinates */
	undistorted_to_distorted_sensor_coord (Xu, Yu, Xd, Yd);

	/* convert from distorted sensor plane coordinates to image coordinates */
	Xf = Xd * cp.sx / cp.dpx + cp.Cx;
	Yf = Yd / cp.dpy + cp.Cy;
	return 1;
}

/*
 * This routine performs an inverse perspective projection to determine	*
 * the position of a point in world coordinates that corresponds to a 	*
 * given position in image coordinates.  To constrain the inverse	*
 * projection to a single point the routine requires a Z world	 	*
 * coordinate for the point in addition to the X and Y image coordinates.*
 */
void MonoCameraTrans::image_coord_to_world_coord(double Xfd, double Yfd, double zw, double &xw, double &yw) {
	double Xd, Yd, Xu, Yu, common_denominator;

	/* convert from image to distorted sensor coordinates */
	Xd = cp.dpx * (Xfd - cp.Cx) / cp.sx;
	Yd = cp.dpy * (Yfd - cp.Cy);

	/* convert from distorted sensor to undistorted sensor plane coordinates */
	distorted_to_undistorted_sensor_coord(Xd, Yd, Xu, Yu);

	/* calculate the corresponding xw and yw world coordinates	 */
	/* (these equations were derived by simply inverting	 */
	/* the perspective projection equations using Macsyma)	 */
	common_denominator = ((cc.r1 * cc.r8 - cc.r2 * cc.r7) * Yu +
		(cc.r5 * cc.r7 - cc.r4 * cc.r8) * Xu -
		cc.f * cc.r1 * cc.r5 + cc.f * cc.r2 * cc.r4);

	xw = (((cc.r2 * cc.r9 - cc.r3 * cc.r8) * Yu +
		(cc.r6 * cc.r8 - cc.r5 * cc.r9) * Xu -
		cc.f * cc.r2 * cc.r6 + cc.f * cc.r3 * cc.r5) * zw +
		(cc.r2 * cc.Tz - cc.r8 * cc.Tx) * Yu +
		(cc.r8 * cc.Ty - cc.r5 * cc.Tz) * Xu -
		cc.f * cc.r2 * cc.Ty + cc.f * cc.r5 * cc.Tx) / common_denominator;

	yw = -(((cc.r1 * cc.r9 - cc.r3 * cc.r7) * Yu +
		(cc.r6 * cc.r7 - cc.r4 * cc.r9) * Xu -
		cc.f * cc.r1 * cc.r6 + cc.f * cc.r3 * cc.r4) * zw +
		(cc.r1 * cc.Tz - cc.r7 * cc.Tx) * Yu +
		(cc.r7 * cc.Ty - cc.r4 * cc.Tz) * Xu -
		cc.f * cc.r1 * cc.Ty + cc.f * cc.r4 * cc.Tx) / common_denominator;
}


void MonoCameraTrans::load_cp_cc_data(FILE *fp, camera_parameters *cp, calibration_constants *cc) {
	double sa, ca, sb, cb, sg, cg;
	int i;

	fscanf(fp, "%lf", &(cp->Ncx));
	fscanf(fp, "%lf", &(cp->Nfx));
	fscanf(fp, "%lf", &(cp->dx));
	fscanf(fp, "%lf", &(cp->dy));
	fscanf(fp, "%lf", &(cp->dpx));
	fscanf(fp, "%lf", &(cp->dpy));
	fscanf(fp, "%lf", &(cp->Cx));
	fscanf(fp, "%lf", &(cp->Cy));
	fscanf(fp, "%lf", &(cp->sx));

	fscanf(fp, "%lf", &(cc->f));
	fscanf(fp, "%lf", &(cc->kappa1));
	fscanf(fp, "%lf", &(cc->Tx));
	fscanf(fp, "%lf", &(cc->Ty));
	fscanf(fp, "%lf", &(cc->Tz));
	fscanf(fp, "%lf", &(cc->Rx));
	fscanf(fp, "%lf", &(cc->Ry));
	fscanf(fp, "%lf", &(cc->Rz));

	SINCOS(cc->Rx, sa, ca);
	SINCOS(cc->Ry, sb, cb);
	SINCOS(cc->Rz, sg, cg);

	cc->r1 = cb * cg;
	cc->r2 = cg * sa * sb - ca * sg;
	cc->r3 = sa * sg + ca * cg * sb;
	cc->r4 = cb * sg;
	cc->r5 = sa * sb * sg + ca * cg;
	cc->r6 = ca * sb * sg - cg * sa;
	cc->r7 = -sb;
	cc->r8 = cb * sa;
	cc->r9 = ca * cb;

	fscanf(fp, "%lf", &(cc->p1));
	fscanf(fp, "%lf", &(cc->p2));

	fscanf(fp, "%lf", &(vxx));
	fscanf(fp, "%lf", &(vxy));
	fscanf(fp, "%lf", &(vxz));
	fscanf(fp, "%lf", &(vyx));
	fscanf(fp, "%lf", &(vyy));
	fscanf(fp, "%lf", &(vyz));
	fscanf(fp, "%lf", &(vzx));
	fscanf(fp, "%lf", &(vzy));
	fscanf(fp, "%lf", &(vzz));
	fscanf(fp, "%lf", &(opx));
	fscanf(fp, "%lf", &(opy));
	fscanf(fp, "%lf", &(opz));
	fscanf(fp, "%d", &(i));
	coplanar = i ? 1 : 0;
	if (fscanf(fp, "%d", &i) == EOF)
		return;
	IMAGEWID = i;
	if (fscanf(fp, "%d", &i) == EOF)
		return;
	IMAGELEN = i;
}

bool MonoCameraTrans::LoadCameraCalib(const char *filename)
{
	FILE *fp;

	fp = fopen(filename, "r");
	if (!fp)
		return false;

	load_cp_cc_data(fp, &cp, &cc);

	fclose(fp);
	return true;
}

void MonoCameraTrans::ImageP2VehicleP(Point3d & vp, double imgX, double imgY) {
	imgY = IMAGELEN - 1 - imgY;
	double x, y, z;
	double Xw, Yw, Zw;
	int	cnt = 0;

	if (!coplanar) {
		z = vp.z - 10000;
		image_coord_to_world_coord(imgX, imgY, z, x, y);
		vp.x = x - 10000;
		vp.y = y - 10000;
		return;
	}

	vp.x = -1;
	vp.y = -1;

	z = vp.z;

	while (cnt < 10) {
		image_coord_to_world_coord(imgX, imgY, z, x, y);

		y -= 1000;
		x -= 1000;
		z = -z;

		Xw = vxx * x + vyx * y + vzx * z + opx;
		Yw = vxy * x + vyy * y + vzy * z + opy;
		Zw = vxz * x + vyz * y + vzz * z + opz;

		if (fabs(Zw - vp.z) < 0.01) {
			vp.x = x / 1000;
			vp.y = y / 1000;
			return;
		}

		Zw = vp.z;
		z = (Xw - opx)*vzx + (Yw - opy)*vzy + (Zw - opz)*vzz;
		z = -z;

		cnt++;
	}

	if (cnt >= 10)
	{
		vp.y /= 1000;
		vp.x /= 1000;
		return;
	}
}

void MonoCameraTrans::VehicleP2ImageP(const Point3d &vp, double &imgX, double &imgY) {
	double Xw = vp.x;
	double Yw = vp.y;
	double Zw = vp.z;
	Xw *= 1000;//convert the unit to millimeter
	Yw *= 1000;
	Zw *= 1000;

	double	x, y, z;

	if (coplanar) {
		x = (Xw - opx)*vxx + (Yw - opy)*vxy + (Zw - opz)*vxz;
		y = (Xw - opx)*vyx + (Yw - opy)*vyy + (Zw - opz)*vyz;
		z = (Xw - opx)*vzx + (Yw - opy)*vzy + (Zw - opz)*vzz;
		z = -z;
	}
	else {
		x = Xw + 10000;
		y = Yw + 10000;
		z = Zw + 10000;
	}

	world_coord_to_image_coord(x, y, z, imgX, imgY);

	imgX = imgX;
	imgY = IMAGELEN - 1 - imgY;
}

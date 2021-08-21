#include "CoordinateTrans.h"
#include <fstream>
#include <iostream>
#include <cmath>

void rMatrixmulti(Matrix &r, Matrix &rt)
{
	double	rin[3][3];
	int		i, j;

	for (i = 0; i<3; i++)
		for (j = 0; j<3; j++)
			rin[i][j] = r[i][j];

	for (i = 0; i<3; i++)
		for (j = 0; j<3; j++) {
			r[i][j] = rin[i][0] * rt[0][j] +
				rin[i][1] * rt[1][j] +
				rin[i][2] * rt[2][j];
		}
}

void createRotMatrix_ZYX(Matrix &rt, double rotateX, double rotateY, double rotateZ) {
	double sinx, siny, sinz, cosx, cosy, cosz;
	Matrix rr;
	int		i, j;

	sinx = sin(rotateX);
	siny = sin(rotateY);
	sinz = sin(rotateZ);
	cosx = cos(rotateX);
	cosy = cos(rotateY);
	cosz = cos(rotateZ);

	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			rt[i][j] = i == j ? 1 : 0;
		}
	}

	if (rotateZ != 0.0) {
		/*	R3 :
		cosz  -sinz   0.0
		sinz  cosz   0.0
		0.0   0.0   1.0
		*/
		rr[0][0] = cosz;
		rr[0][1] = -sinz;
		rr[0][2] = 0.0;
		rr[1][0] = sinz;
		rr[1][1] = cosz;
		rr[1][2] = 0.0;
		rr[2][0] = 0.0;
		rr[2][1] = 0.0;
		rr[2][2] = 1.0;
		rMatrixmulti(rt, rr);
	}

	if (rotateY != 0.0) {
		/*	R2 :
		cosy   0.0  siny
		0.0   1.0   0.0
		-siny   0.0  cosy
		*/
		rr[0][0] = cosy;
		rr[0][1] = 0.0;
		rr[0][2] = siny;
		rr[1][0] = 0.0;
		rr[1][1] = 1.0;
		rr[1][2] = 0.0;
		rr[2][0] = -siny;
		rr[2][1] = 0.0;
		rr[2][2] = cosy;
		rMatrixmulti(rt, rr);
	}

	if (rotateX != 0.0) {
		/*	R1 :
		1.0   0.0   0.0
		0.0  cosx  -sinx
		0.0  sinx  cosx
		*/
		rr[0][0] = 1.0;
		rr[0][1] = 0.0;
		rr[0][2] = 0.0;
		rr[1][0] = 0.0;
		rr[1][1] = cosx;
		rr[1][2] = -sinx;
		rr[2][0] = 0.0;
		rr[2][1] = sinx;
		rr[2][2] = cosx;
		rMatrixmulti(rt, rr);
	}
}

void createRotMatrix_XYZ(Matrix &rt, double rotateX, double rotateY, double rotateZ)
{
	double sinx, siny, sinz, cosx, cosy, cosz;
	Matrix rr;

	sinx = sin(rotateX);
	siny = sin(rotateY);
	sinz = sin(rotateZ);
	cosx = cos(rotateX);
	cosy = cos(rotateY);
	cosz = cos(rotateZ);

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			rt[i][j] = i == j ? 1 : 0;
		}
	}

	if (rotateX != 0.0) {
		/*	R1 :
		1.0   0.0   0.0
		0.0  cosx  -sinx
		0.0  sinx  cosx
		*/
		rr[0][0] = 1.0;
		rr[0][1] = 0.0;
		rr[0][2] = 0.0;
		rr[1][0] = 0.0;
		rr[1][1] = cosx;
		rr[1][2] = -sinx;
		rr[2][0] = 0.0;
		rr[2][1] = sinx;
		rr[2][2] = cosx;
		rMatrixmulti(rt, rr);
	}

	if (rotateY != 0.0) {
		/*	R2 :
		cosy   0.0 siny
		0.0   1.0   0.0
		-siny   0.0  cosy
		*/
		rr[0][0] = cosy;
		rr[0][1] = 0.0;
		rr[0][2] = siny;
		rr[1][0] = 0.0;
		rr[1][1] = 1.0;
		rr[1][2] = 0.0;
		rr[2][0] = -siny;
		rr[2][1] = 0.0;
		rr[2][2] = cosy;
		rMatrixmulti(rt, rr);
	}

	if (rotateZ != 0.0) {
		/*	R3 :
		cosz  -sinz   0.0
		sinz cosz   0.0
		0.0   0.0   1.0
		*/
		rr[0][0] = cosz;
		rr[0][1] = -sinz;
		rr[0][2] = 0.0;
		rr[1][0] = sinz;
		rr[1][1] = cosz;
		rr[1][2] = 0.0;
		rr[2][0] = 0.0;
		rr[2][1] = 0.0;
		rr[2][2] = 1.0;
		rMatrixmulti(rt, rr);
	}
}

void shiftPoint3d(Point3d &pt, Point3d &sh) {
	Point3d p;

	p.x = pt.x + sh.x;
	p.y = pt.y + sh.y;
	p.z = pt.z + sh.z;
	pt.x = p.x;
	pt.y = p.y;
	pt.z = p.z;
}

void rotatePoint3d(Point3d &pt, Matrix &a) {
	Point3d	p;

	p.x = a[0][0] * pt.x + a[0][1] * pt.y + a[0][2] * pt.z;
	p.y = a[1][0] * pt.x + a[1][1] * pt.y + a[1][2] * pt.z;
	p.z = a[2][0] * pt.x + a[2][1] * pt.y + a[2][2] * pt.z;
	pt.x = p.x;
	pt.y = p.y;
	pt.z = p.z;
}

bool CoordinateTrans::LoadCalib(std::string filename) {
	if (filename == "") {
		std::cerr << "calib load error\n" << std::endl;
		return false;
	}

	std::ifstream calibfile;
	calibfile.open(filename.c_str());
	if (!calibfile.is_open()) {
		std::cerr << "calib load error\n" << std::endl;
		return false;
	}

	std::string header;
	calibfile >> header >> trans.ang.x >> trans.ang.y >> trans.ang.z;
	calibfile >> header >> trans.shv.x >> trans.shv.y >> trans.shv.z;

	createRotMatrix_XYZ(trans.rot, trans.ang.x, trans.ang.y, trans.ang.z);
	createRotMatrix_ZYX(trans.invrot, -trans.ang.x, -trans.ang.y, -trans.ang.z);
	trans.invshv.x = -trans.shv.x;
	trans.invshv.y = -trans.shv.y;
	trans.invshv.z = -trans.shv.z;

	calibfile.close();
	return true;
}

void CoordinateTrans::VehicleP2LocalP(const Point3d &src, Point3d &dst) {
	Point3d pt;
	pt.x = src.x;
	pt.y = src.y;
	pt.z = src.z;
	shiftPoint3d(pt, trans.invshv);
	rotatePoint3d(pt, trans.invrot);
	dst.x = pt.x;
	dst.y = pt.y;
	dst.z = pt.z;
}

void CoordinateTrans::LocalP2VehicleP(const Point3d &src, Point3d &dst) {
	Point3d pt;
	pt.x = src.x;
	pt.y = src.y;
	pt.z = src.z;
	rotatePoint3d(pt, trans.rot);
	shiftPoint3d(pt, trans.shv);
	dst.x = pt.x;
	dst.y = pt.y;
	dst.z = pt.z;
}

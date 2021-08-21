#pragma once

#include "CoordinateTrans.h"

#include <cstdio>
#include <cstdlib>
#include <cmath>

#define SQR(a) ((a)*(a))
#define CUB(a) ((a)*(a)*(a))
#define SQRT(x) sqrt(fabs(x))
#define SQRT3 1.732050807568877293527446341505872366943

/* Some math libraries may not have the sincos() routine */
#ifdef _SINCOS_
void sincos();
#define SINCOS(x,s,c)   sincos(x,&s,&c)
#else
double sin(), cos();
#define SINCOS(x,s,c)   s=sin(x);c=cos(x)
#endif


/** 
 * @brief 相机参数，具体含义见标定文件说明
 * 
 * Camera parameters are usually the fixed parameters of the given camera 	
 * system, typically obtained from manufacturers specifications. 	
 * Cy and Cy (the center of radial lens distortion), may be calibrated 
 * separately or as part of the coplanar/noncoplanar calibration. 
 * The same with sx (x scale uncertainty factor).  
 * 
 */
class camera_parameters {
public:
	double Ncx; /**< [sel]  Number of sensor elements in camera's x direction */
	double Nfx; /**< [pix]  Number of pixels in frame grabber's x direction */
	double dx; /**< [mm/sel]  X dimension of camera's sensor element (in mm) */
	double dy; /**< [mm/sel]  Y dimension of camera's sensor element (in mm) */
	double dpx; /**< [mm/pix]  Effective X dimension of pixel in frame grabber */
	double dpy; /**< [mm/pix]  Effective Y dimension of pixel in frame grabber */
	double Cx; /**< [pix]  Z axis intercept of camera coordinate system */
	double Cy; /**< [pix]  Z axis intercept of camera coordinate system */
	double sx; /**< []  Scale factor to compensate for any error in dpx */
};

/** 
 * @brief 相机常数，具体含义见标定文件说明
 * Calibration constants are the model constants that are determined from the 	*
 * calibration data. 
 *
 */
class calibration_constants {
public:
	double f; /**< [mm] */
	double kappa1; /**< [1/mm^2] */
	double p1; /**< [1/mm] */
	double p2; /**< [1/mm] */
	double Tx; /**< [mm] */
	double Ty; /**< [mm] */
	double Tz; /**< [mm] */
	double Rx; /**< [rad] */
	double Ry; /**< [rad] */
	double Rz; /**< [rad] */
	double r1; /**< [] */
	double r2; /**< [] */
	double r3; /**< [] */
	double r4; /**< [] */
	double r5; /**< [] */
	double r6; /**< [] */
	double r7; /**< [] */
	double r8; /**< [] */
	double r9; /**< [] */
};

/**
 * @brief 用于单目相机坐标转换的类，存储单目相机的内外参数
 *
 */
class MonoCameraTrans {
public:
 /**
	 * @brief 加载一组单目相机的标定文件
	 *
	 * @param filename 标定文件名
	 * @return true 加载标定文件正常
	 * @return false 加载标定文件出错
	 */
	bool LoadCameraCalib(const char *filename);

	/**
	 * @brief 图像坐标系的点转换到车体坐标系（假设 Z 值已知）
	 *
	 * @param dst 转化到车体坐标系下的三维点
	 * @param imgX 图像坐标系的 X 坐标（列）
	 * @param imgY 图像坐标系的 Y 坐标（行）
	 */
	void ImageP2VehicleP(Point3d& dst, double imgX, double imgY);

	/**
	 * @brief 车体坐标系的点转换到图像坐标系
	 *
	 * @param src 车体坐标系下的三维点
	 * @param imgX 图像坐标系的 X 坐标
	 * @param imgY 图像坐标系的 Y 坐标
	 */
	void VehicleP2ImageP(const Point3d& src, double &imgX, double &imgY);

private:
	/**
	 * @brief 未畸变图像的点装换到畸变图像的点
	 * 
	 * @param Xu 未畸变图像的点 X 坐标 
	 * @param Yu 未畸变图像的点 Y 坐标 
	 * @param Xd 畸变图像的点 X 坐标
	 * @param Yd 畸变图像的点 Y 坐标
	 */
	void undistorted_to_distorted_sensor_coord(double Xu, double Yu, double &Xd, double &Yd);

	/**
	 * @brief 畸变图像的点装换到未畸变图像的点
	 * 
	 * @param Xd 畸变图像的点 X 坐标
	 * @param Yd 畸变图像的点 Y 坐标
	 * @param Xu 未畸变图像的点 X 坐标 
	 * @param Yu 未畸变图像的点 Y 坐标 
	 */
	void distorted_to_undistorted_sensor_coord(double Xd, double Yd, double &Xu, double &Yu);

	/**
	 * @brief 相机视角下的世界坐标系转换到图像坐标系
	 * 
	 * @param xw 三维坐标 X
	 * @param yw 三维坐标 Y
	 * @param zw 三维坐标 Z
	 * @param Xf 图像坐标系 X
	 * @param Yf 图像坐标系 Y
	 * @return int 大于 0，则转换成功
	 */
	int world_coord_to_image_coord(double xw, double yw, double zw, double &Xf, double &Yf);

	/**
	 * @brief 图像坐标系转换到相机视角下的世界坐标系
	 * 
	 * @param Xfd 图像坐标系 X
	 * @param Yfd 图像坐标系 Y
	 * @param zw 三维坐标 Z
	 * @param xw 三维坐标 X
	 * @param yw 三维坐标 Y
	 */
	void image_coord_to_world_coord(double Xfd, double Yfd, double zw, double &xw, double &yw);

	/**
	 * @brief 加载相机参数
	 * 
	 * @param fp 文件描述符
	 * @param cp 相机参数
	 * @param cc 相机常数
	 */
	void load_cp_cc_data(FILE *fp, camera_parameters *cp, calibration_constants *cc);

	double vxx, vxy, vxz; /**< 见标定文件描述 */
	double vyx, vyy, vyz; /**< 见标定文件描述 */
	double vzx, vzy, vzz; /**< 见标定文件描述 */
	double opx, opy, opz; /**< 见标定文件描述 */

	int coplanar = 0;

	double IMAGEWID; /**< 图像宽度 */
	double IMAGELEN; /**< 图像高度 */

	camera_parameters cp;
	calibration_constants cc;

};


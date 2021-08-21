#pragma once

#include <string>

/**
 * @brief 三维矩阵
 * 
 */
typedef double Matrix[3][3];

/**
 * @brief 三维点
 * 
 */
class Point3d {
public:
	double x, y, z;
	Point3d() {}
	Point3d(double x, double y, double z)
		:x(x), y(y), z(z) {}
};

/**
 * @brief 存储坐标转换信息
 * 
 */
class TransInfo {
public:
	char name[20];
	Point3d ang;

	Point3d shv;
	Point3d invshv;

	Matrix rot;
	Matrix invrot;

	bool valid;
};

/**
 * @brief 计算三维矩阵的逆矩阵
 * @param r 原矩阵
 * @param rt 原矩阵的逆矩阵
 * 
 */
void rMatrixmulti(Matrix &r, Matrix &rt);

/**
 * @brief 创建旋转矩阵
 * 
 * @param rt 旋转矩阵
 * @param rotateX 绕 X 轴旋转的旋转，单位为弧度
 * @param rotateY 绕 Y 轴旋转的旋转，单位为弧度
 * @param rotateZ 绕 Z 轴旋转的旋转，单位为弧度
 * 
 * rt <- Rz * Ry * Rx
 */
void createRotMatrix_ZYX(Matrix &rt, double rotateX, double rotateY, double rotateZ);

/**
 * @brief 创建旋转矩阵，为 createRotMatrix_ZYX 的逆矩阵
 * 
 * @param rotateX 绕 X 轴旋转的旋转，单位为弧度
 * @param rotateY 绕 Y 轴旋转的旋转，单位为弧度
 * @param rotateZ 绕 Z 轴旋转的旋转，单位为弧度
 * 
 * rt <- Rx * Ry * Rz
 */
void createRotMatrix_XYZ(Matrix &rt, double rotateX, double rotateY, double rotateZ);

/**
 * @brief 对一个三维点进行平移操作
 * 
 * @param pt 三维点
 * @param sh 平移向量
 * 
 */
void shiftPoint3d(Point3d &pt, Point3d &sh);

/**
 * @brief 对一个三维点进行旋转操作
 * 
 * @param pt 三维点
 * @param a 旋转矩阵
 */
void rotatePoint3d(Point3d &pt, Matrix &a);

/**
 * @brief 用于坐标转换的类，存储一组坐标转换关系
 * 
 */
class CoordinateTrans
{
public:
	/**
	 * @brief 加载一组坐标转换的标定文件
	 * 
	 * @param filename 标定文件名
	 * @return true 加载标定文件正常
	 * @return false 加载标定文件出错
	 */
	bool LoadCalib(std::string filename);

	/**
	 * @brief 把传感器坐标系下的点变换到车体坐标系下
	 * 
	 * @param src 传感器坐标系下的点
	 * @param dst 车体坐标系下的点
	 */
	void LocalP2VehicleP(const Point3d& src, Point3d& dst);

	/**
	 * @brief 把车体坐标系下的点变换到传感器坐标系下
	 * 
	 * @param src 传感器坐标系下的点
	 * @param dst 车体坐标系下的点
	 */
	void VehicleP2LocalP(const Point3d& src, Point3d& dst);

private:
	TransInfo trans; /**< 坐标转换关系 */
};
#include "perception/GetLocalCoord.h"
#include "perception/BoundingBox.h"
#include "perception/Position.h"
#include "perception/Detection_result.h"
#include "perception/Detection_results.h"
#include <pcl/point_cloud.h>
#include "CoordinateTrans.h"
#include "MonoCameraTrans.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <opencv2/core.hpp> 
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <ctime>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <rosgraph_msgs/Clock.h>
#include "std_msgs/String.h"
#include "Eigen/Dense"
const float k1 = 0.3;
const float k2 = 0.4;
const float k3 = 0.3;


struct Detected_Obj
  {
    jsk_recognition_msgs::BoundingBox bounding_box_;
    pcl::PointXYZ min_point_;
    pcl::PointXYZ max_point_;
    pcl::PointXYZ centroid_;
  };

struct clusterInfo
{
    float height_ = 0.0;
    float max_height_ = 0.0;
    float min_height_ = 10e6;
    float depth_ = 0.0;
    int member_num_ = 0;
    float score = 0.0;
    float x = 0;
    float y = 0;
    float z = 0;
    float min_imgx = 0;
    float min_imgy = 0;
    float max_imgx = 0;
    float max_imgy = 0;
    int indice_num = 0;
};

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
) //自定义点云类型，包含反射强度，ringID和点时间


using POINT = VelodynePointXYZIRT;
using CLOUD = pcl::PointCloud<POINT>;
using CLOUD_PTR = CLOUD::Ptr;

struct LidarPoint { // single lidar point in space
    double x,y,z,r,imgx,imgy; // x,y,z in [m], r is point reflectivity
};

struct BoundingBoxWithPcl { // bounding box around a classified object (contains both 2D and 3D data)

    int boxID; // unique identifier for this bounding box
    LidarPoint coord;

    cv::Rect roi; // 2D region-of-interest in image coordinates
    int classID; // ID based on class file provided to YOLO framework
    double confidence; // classification trust

    std::vector<LidarPoint> lidarPoints; // Lidar 3D points which project into 2D image roi
    std::vector<cv::KeyPoint> keypoints; // keypoints enclosed by 2D roi
    std::vector<cv::DMatch> kptMatches; // keypoint matches enclosed by 2D roi
};

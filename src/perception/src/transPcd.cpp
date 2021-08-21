#include "ros/ros.h"
#include "getLocalCoord.h"
#include <cv_bridge/cv_bridge.h>
#include "math.h"
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

using namespace perception;
using namespace std;
using namespace jsk_recognition_msgs;
sensor_msgs::PointCloud2 pl2_msg;
ros::Publisher pub_scan_, pub_bounding_boxs_;
ros::Subscriber sub_vel;

bool transPcd(const sensor_msgs::PointCloud2::ConstPtr &pcd)
{
    sensor_msgs::PointCloud2 ros_pcl = *pcd;

    /// 第一个例子以转换一个32线Velodyne的原始激光点的三维坐标 到 图像二维坐标为例
    CoordinateTrans velo32CTrans; /**< Velodyne 到车体的转换关系 */
    MonoCameraTrans monoTrans;    /**< 相机坐标到车体的转换关系 */

    cout << "111" << endl;
    /// 加载标定配置文件
    monoTrans.LoadCameraCalib("/home/young/Raw-001/Raw-001-Calib/Raw-001-Camera.camera");
    velo32CTrans.LoadCalib("/home/young/Raw-001/Raw-001-Calib/Raw-001-HDL32-E.txt");

    pcl::PointCloud<pcl::PointXYZI> cloud_pcl_xyzi;
    pcl::fromROSMsg(ros_pcl, cloud_pcl_xyzi);
    cout << "222" << endl;
    pcl::PointCloud<pcl::PointXYZI> veh_pcd;

    sensor_msgs::PointCloud2 ros_veh_pcd;
    cout << "333" << endl;

    vector<LidarPoint> lidarPointVector;
    cout << "444" << endl;
    CLOUD_PTR temp_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr trans_pcd = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (int i = 0; i < cloud_pcl_xyzi.points.size(); i++)
    {
        // cout << cloud_pcl_xyzi.points[i] << endl;

        Point3d veloLocalPt; /**< 从*.pcap文件解析出来的一个激光点（HDL32局部坐标系下）*/
        veloLocalPt.x = -cloud_pcl_xyzi.points[i].x;
        veloLocalPt.y = -cloud_pcl_xyzi.points[i].y;
        veloLocalPt.z = cloud_pcl_xyzi.points[i].z;
        cout << "555" << endl;
        Point3d vehiclePt; /**< 转换到车体局部坐标系的点 */
        velo32CTrans.LocalP2VehicleP(veloLocalPt, vehiclePt);

        double imgx, imgy;
        monoTrans.VehicleP2ImageP(vehiclePt, imgx, imgy); /**< 车体坐标系到图像坐标 */
        cout << "666" << endl;
        // POINT point;
        // point.x = vehiclePt.x;
        // point.y = vehiclePt.y;
        // point.z = vehiclePt.z;
        // point.intensity = cloud_pcl_xyzi.points[i].intensity;
        // temp_cloud_->points.push_back(point);

        pcl::PointXYZ point;
        point.x = vehiclePt.x;
        point.y = vehiclePt.y;
        point.z = vehiclePt.z;

        if (point.y < 0 || point.z < 0.3)
            continue;
        cout << "777" << endl;

        // point.x = veloLocalPt.x;
        // point.y = veloLocalPt.y;
        // point.z = veloLocalPt.z;

        // point.intensity = cloud_pcl_xyzi.points[i].intensity;

        trans_pcd->points.push_back(point);
        cout << "bbb" << endl;
        cv::Point pt;
        pt.x = imgx;
        pt.y = imgy;
        LidarPoint lidarPoint;
        lidarPoint.x = vehiclePt.x;
        lidarPoint.y = vehiclePt.y;
        lidarPoint.z = vehiclePt.z;
        lidarPoint.r = cloud_pcl_xyzi.points[i].intensity;
        lidarPoint.imgx = imgx;
        lidarPoint.imgy = imgy;
        lidarPointVector.push_back(lidarPoint);
    } // eof loop over all points
    cout << "888" << endl;

    /////////////   POINT CLOUD CLUSTER
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(trans_pcd);
    cout << "999" << endl;

    std::vector<pcl::PointIndices> local_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.2); // 2cm
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(250);
    ec.setSearchMethod(tree);
    ec.setInputCloud(trans_pcd);
    ec.extract(local_indices);
    std::vector<Detected_Obj> obj_list;
    cout << "aaa" << endl;


    cout << " local_indices.size:  " <<  local_indices.size() << endl;
    for (size_t i = 0; i < local_indices.size(); i++)
    {
        // the structure to save one detected object
        Detected_Obj obj_info;

        float min_x = std::numeric_limits<float>::max();
        float max_x = -std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float max_y = -std::numeric_limits<float>::max();
        float min_z = std::numeric_limits<float>::max();
        float max_z = -std::numeric_limits<float>::max();
        cout << "bbb" << endl;

        for (auto pit = local_indices[i].indices.begin(); pit != local_indices[i].indices.end(); ++pit)
        {
            //fill new colored cluster point by point
            pcl::PointXYZ p;
            p.x = trans_pcd->points[*pit].x;
            p.y = trans_pcd->points[*pit].y;
            p.z = trans_pcd->points[*pit].z;

            obj_info.centroid_.x += p.x;
            obj_info.centroid_.y += p.y;
            obj_info.centroid_.z += p.z;

            if (p.x < min_x)
                min_x = p.x;
            if (p.y < min_y)
                min_y = p.y;
            if (p.z < min_z)
                min_z = p.z;
            if (p.x > max_x)
                max_x = p.x;
            if (p.y > max_y)
                max_y = p.y;
            if (p.z > max_z)
                max_z = p.z;
        }

        //min, max points
        obj_info.min_point_.x = min_x;
        obj_info.min_point_.y = min_y;
        obj_info.min_point_.z = min_z;

        obj_info.max_point_.x = max_x;
        obj_info.max_point_.y = max_y;
        obj_info.max_point_.z = max_z;

        //calculate centroid
        if (local_indices[i].indices.size() > 0)
        {
            obj_info.centroid_.x /= local_indices[i].indices.size();
            obj_info.centroid_.y /= local_indices[i].indices.size();
            obj_info.centroid_.z /= local_indices[i].indices.size();
        }

        //calculate bounding box
        double length_ = obj_info.max_point_.x - obj_info.min_point_.x;
        double width_ = obj_info.max_point_.y - obj_info.min_point_.y;
        double height_ = obj_info.max_point_.z - obj_info.min_point_.z;

        obj_info.bounding_box_.header = ros_pcl.header;

        obj_info.bounding_box_.pose.position.x = obj_info.min_point_.x + length_ / 2;
        obj_info.bounding_box_.pose.position.y = obj_info.min_point_.y + width_ / 2;
        obj_info.bounding_box_.pose.position.z = obj_info.min_point_.z + height_ / 2;

        obj_info.bounding_box_.dimensions.x = ((length_ < 0) ? -1 * length_ : length_);
        obj_info.bounding_box_.dimensions.y = ((width_ < 0) ? -1 * width_ : width_);
        obj_info.bounding_box_.dimensions.z = ((height_ < 0) ? -1 * height_ : height_);

        obj_list.push_back(obj_info);

    }

    jsk_recognition_msgs::BoundingBoxArray bbox_array;

    for (size_t i = 0; i < obj_list.size(); i++)
    {
        bbox_array.boxes.push_back(obj_list[i].bounding_box_);
    }
    bbox_array.header = ros_pcl.header;

    pub_bounding_boxs_.publish(bbox_array);


    pcl::toROSMsg(*trans_pcd, pl2_msg);
    pl2_msg.header = ros_pcl.header;
    pub_scan_.publish(pl2_msg);
    cout << "999" << endl;
    temp_cloud_.reset(new CLOUD());

    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "transform_pcd");
    ros::NodeHandle n;

    pub_scan_ = n.advertise<sensor_msgs::PointCloud2>("trans_velodyne_points", 1000);
    pub_bounding_boxs_ = n.advertise<jsk_recognition_msgs::BoundingBoxArray>("bbox", 1000);
    sub_vel = n.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 1000, transPcd);
    ros::spin();

    return 0;
}

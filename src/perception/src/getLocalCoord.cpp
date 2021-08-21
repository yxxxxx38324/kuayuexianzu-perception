#include "ros/ros.h"
#include "getLocalCoord.h"
#include <cv_bridge/cv_bridge.h>
#include "math.h"
using namespace perception;
using namespace std;
/// 第一个例子以转换一个32线Velodyne的原始激光点的三维坐标 到 图像二维坐标为例
CoordinateTrans velo32CTrans; /**< Velodyne 到车体的转换关系 */
MonoCameraTrans monoTrans;    /**< 相机坐标到车体的转换关系 */



/**
  *brief:基于一个bbox中的点云进行聚类，并且返回每个类在原点云中所对应的索引。
  *author:Yang Xiaoxiao
  *date:20210819
  **/
std::vector<pcl::PointIndices> get_cluster_indices(std::vector<LidarPoint> lidarPoints)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr trans_pcd = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (auto point : lidarPoints)
    {
        pcl::PointXYZ pcl_point;
        pcl_point.x = point.x;
        pcl_point.y = point.y;
        pcl_point.z = 0;//point.z;
        trans_pcd->points.push_back(pcl_point);
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(trans_pcd);

    std::vector<pcl::PointIndices> local_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.2); // 2cm
    ec.setMinClusterSize(2);
    ec.setMaxClusterSize(100);
    ec.setSearchMethod(tree);
    ec.setInputCloud(trans_pcd);
    ec.extract(local_indices);

    return local_indices;
}

/**
  *brief:基于聚类输出的每个类，对每个类进行打分，并输出最后的最佳类，作为被打在目标身上的点云。
  *author:Yang Xiaoxiao
  *date:20210819
  **/
clusterInfo choose_cluster_indices(std::vector<pcl::PointIndices> local_indices, std::vector<LidarPoint> lidarPoints)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr trans_pcd = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (auto point : lidarPoints)
    {
        pcl::PointXYZ pcl_point;
        pcl_point.x = point.x;
        pcl_point.y = point.y;
        pcl_point.z = point.z;
        trans_pcd->points.push_back(pcl_point);
    }   

    vector<clusterInfo> allClusterInfo;
    clusterInfo clusterSumInfo;
    //loop for each cluster in one bbox
    for (size_t i = 0; i < local_indices.size(); i++)
    {
        // the structure to save one detected object
        Detected_Obj obj_info;

        float min_imgx = std::numeric_limits<float>::max();
        float min_imgy = std::numeric_limits<float>::max();
        float max_imgx = -std::numeric_limits<float>::max();
        float max_imgy = -std::numeric_limits<float>::max();

        clusterInfo cluster_info;
        for (auto pit = local_indices[i].indices.begin(); pit != local_indices[i].indices.end(); ++pit)
        {
            //fill new colored cluster point by point
            pcl::PointXYZ p;
            p.x = trans_pcd->points[*pit].x;
            p.y = trans_pcd->points[*pit].y;
            p.z = trans_pcd->points[*pit].z;

            double imgx, imgy;
            Point3d point3d;
            point3d.x = p.x;
            point3d.y = p.y;
            point3d.z = p.z;
        
            monoTrans.VehicleP2ImageP(point3d, imgx, imgy);

            min_imgx = min_imgx < imgx ? min_imgx : imgx;
            min_imgy = min_imgy < imgy ? min_imgy : imgy;
            max_imgx = max_imgx > imgx ? max_imgx : imgx;
            max_imgy = max_imgy > imgy ? max_imgy : imgy;

            obj_info.centroid_.x += p.x;
            obj_info.centroid_.y += p.y;
            obj_info.centroid_.z += p.z;
        }

        //calculate centroid
        if (local_indices[i].indices.size() > 0)
        {
            obj_info.centroid_.x /= local_indices[i].indices.size();
            obj_info.centroid_.y /= local_indices[i].indices.size();
            obj_info.centroid_.z /= local_indices[i].indices.size();
        }

        cluster_info.depth_ = sqrt(pow(obj_info.centroid_.x, 2) + pow(obj_info.centroid_.y, 2) + pow(obj_info.centroid_.z, 2));
        cluster_info.member_num_ = local_indices[i].indices.size();
        cluster_info.height_ = obj_info.centroid_.z;
        cluster_info.x = obj_info.centroid_.x;
        cluster_info.y = obj_info.centroid_.y;
        cluster_info.z = obj_info.centroid_.z;

        cluster_info.min_imgx = min_imgx;
        cluster_info.min_imgy = min_imgy;
        cluster_info.max_imgx = max_imgx;
        cluster_info.max_imgy = max_imgy;

        cluster_info.indice_num = i;

        clusterSumInfo.depth_ += cluster_info.depth_;
        clusterSumInfo.member_num_ += cluster_info.member_num_;
        clusterSumInfo.min_height_ = cluster_info.height_<clusterSumInfo.min_height_?cluster_info.height_:clusterSumInfo.min_height_;
        clusterSumInfo.max_height_ = cluster_info.height_>clusterSumInfo.max_height_?cluster_info.height_:clusterSumInfo.max_height_;

        allClusterInfo.push_back(cluster_info);
        
        //calculate bounding box
        double length_ = obj_info.max_point_.x - obj_info.min_point_.x;
        double width_ = obj_info.max_point_.y - obj_info.min_point_.y;
        double height_ = obj_info.max_point_.z - obj_info.min_point_.z;
    }
    
    for(auto clusterInfo: allClusterInfo){
        clusterInfo.score = k1 * (1 - clusterInfo.depth_/clusterSumInfo.depth_) + 
                            k2 * (clusterInfo.member_num_/clusterSumInfo.member_num_) +
                            k3 * ((clusterInfo.height_ - clusterSumInfo.min_height_)/(clusterSumInfo.max_height_ - clusterSumInfo.min_height_));
    }

    float maxScore = 0;
    clusterInfo best_cluster;
    for(auto p = allClusterInfo.begin(); p != allClusterInfo.end(); ++p)
    {
        if(p->score > maxScore)
        {
            maxScore = p->score;
            best_cluster = *p;
        }
    }
    return best_cluster;
}

/**
  *brief:此服务的回调函数，输出带有位置信息的检测结果。
  *author:Yang Xiaoxiao
  *date:20210816
  **/
bool get_local_coord(GetLocalCoord::Request &req,
                     GetLocalCoord::Response &res)
{
    Detection_results all_det_res = req.det_ress;
    sensor_msgs::PointCloud2 ros_pcl = req.pcl;

    /// 加载标定配置文件
    monoTrans.LoadCameraCalib("/home/young/Raw-001/Raw-001-Calib/Raw-001-Camera.camera");
    velo32CTrans.LoadCalib("/home/young/Raw-001/Raw-001-Calib/Raw-001-HDL32-E.txt");

    pcl::PointCloud<pcl::PointXYZI> cloud_pcl_xyzi;
    pcl::fromROSMsg(ros_pcl, cloud_pcl_xyzi);

    pcl::PointCloud<pcl::PointXYZI> veh_pcd;

    sensor_msgs::PointCloud2 ros_veh_pcd;

    // showLidarTopview(cloud_pcl_xyzi);

    //获取落入每个bbox中的点云集合
    int bbox_num = all_det_res.detection_results.size();
    vector<BoundingBoxWithPcl> bboxWithPclVector(bbox_num);

    vector<LidarPoint> lidarPointVector;

    CLOUD_PTR temp_cloud_;
    for (int i = 0; i < cloud_pcl_xyzi.points.size(); i++)
    {
        // cout << cloud_pcl_xyzi.points[i] << endl;

        Point3d veloLocalPt; /**< 从*.pcap文件解析出来的一个激光点（HDL32局部坐标系下）*/
        veloLocalPt.x = -cloud_pcl_xyzi.points[i].x;
        veloLocalPt.y = -cloud_pcl_xyzi.points[i].y;
        veloLocalPt.z = cloud_pcl_xyzi.points[i].z;

        Point3d vehiclePt; /**< 转换到车体局部坐标系的点 */
        velo32CTrans.LocalP2VehicleP(veloLocalPt, vehiclePt);

        double imgx, imgy;
        monoTrans.VehicleP2ImageP(vehiclePt, imgx, imgy); /**< 车体坐标系到图像坐标 */

        cv::Point pt;
        pt.x = imgx;
        pt.y = imgy;
        LidarPoint lidarPoint;

        if (vehiclePt.y < 0 || vehiclePt.z < 0.3)
            continue;

        lidarPoint.x = vehiclePt.x;
        lidarPoint.y = vehiclePt.y;
        lidarPoint.z = vehiclePt.z;
        lidarPoint.r = cloud_pcl_xyzi.points[i].intensity;
        lidarPoint.imgx = imgx;
        lidarPoint.imgy = imgy;
        for (int i = 0; i < all_det_res.detection_results.size(); i++)
        {
            Detection_result det_res = all_det_res.detection_results[i];
            bboxWithPclVector[i].boxID = i;
            bboxWithPclVector[i].roi.x = float(det_res.bounding_box.point1.x * (768.0 / 480.0));
            bboxWithPclVector[i].roi.y = float(det_res.bounding_box.point1.y * (1024.0 / 640.0));
            bboxWithPclVector[i].roi.width = float((det_res.bounding_box.point2.x - det_res.bounding_box.point1.x) * (1024.0 / 640.0));
            bboxWithPclVector[i].roi.height = float((det_res.bounding_box.point4.y - det_res.bounding_box.point1.y) * (768.0 / 480.0));

            // check wether point is within current bounding box
            if (bboxWithPclVector[i].roi.contains(pt))
            {
                bboxWithPclVector[i].lidarPoints.push_back(lidarPoint);
            }
        }
        lidarPointVector.push_back(lidarPoint);
    } // eof loop over all points

    float max_dis = 0;
    for (auto point : lidarPointVector)
    {
        float r = sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
        if (r > max_dis)
        {
            max_dis = r;
        }
    }

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(req.img, sensor_msgs::image_encodings::BGR8);
    cv::Mat img = cv_ptr->image;
    // cv::Mat img_resize;
    // cv::resize(img, img_resize, cv::Size(640, 480));
    cout << "The size of bboxWithPclVector: " << bboxWithPclVector.size() << endl;
    for (int i = 0; i < bboxWithPclVector.size(); i++)
    {
        cout << "bboxWithPclVector[" << i << "].lidarPoints:  " << bboxWithPclVector[i].lidarPoints.size() << endl;

        for(auto point : bboxWithPclVector[i].lidarPoints){
            float r = sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
            int red = min(255, (int)(255 * abs((r - max_dis) / max_dis)));
            int green = min(255, (int)(255 * (1 - abs((r - max_dis) / max_dis))));
            cv::circle(img, cv::Point(point.imgx, point.imgy), 2, cv::Scalar(0, green, red), -1);
            // cv::circle(img, cv::Point(point.imgx, point.imgy), 5, cv::Scalar(0, 255, 0), -1);
        }
        std::vector<pcl::PointIndices> cluster_indices = get_cluster_indices(bboxWithPclVector[i].lidarPoints);
        clusterInfo best_cluster = choose_cluster_indices(cluster_indices, bboxWithPclVector[i].lidarPoints);
        cv::rectangle(img, cv::Point(bboxWithPclVector[i].roi.x, bboxWithPclVector[i].roi.y),
                      cv::Point(bboxWithPclVector[i].roi.x + bboxWithPclVector[i].roi.width,
                                bboxWithPclVector[i].roi.y + bboxWithPclVector[i].roi.height),
                      cv::Scalar(255, 0, 0), 3, 1, 0);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr trans_pcd = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        for (auto point : bboxWithPclVector[i].lidarPoints)
        {
            pcl::PointXYZ pcl_point;
            pcl_point.x = point.x;
            pcl_point.y = point.y;
            pcl_point.z = point.z;
            trans_pcd->points.push_back(pcl_point);
        }   
        cout<<"cluster_indices[best_cluster.indice_num].indices.size():   " << cluster_indices[best_cluster.indice_num].indices.size() << endl;
        cout<<"cluster_indices.size():   " << cluster_indices.size() << endl;
        for (auto pit = cluster_indices[best_cluster.indice_num].indices.begin(); pit != cluster_indices[best_cluster.indice_num].indices.end(); ++pit)
        {
            //fill new colored cluster point by point
            pcl::PointXYZ p;
            p.x = trans_pcd->points[*pit].x;
            p.y = trans_pcd->points[*pit].y;
            p.z = trans_pcd->points[*pit].z;

            double imgx, imgy;
            Point3d point3d;
            point3d.x = p.x;
            point3d.y = p.y;
            point3d.z = p.z;
        
            monoTrans.VehicleP2ImageP(point3d, imgx, imgy);

            cv::circle(img, cv::Point(imgx, imgy), 3, cv::Scalar(255, 255, 255), -1);
        }

        cout<<"best_cluster.min_imgx:  "<<best_cluster.min_imgx<<endl;
        cout<<"best_cluster.min_imgy:  "<<best_cluster.min_imgy<<endl;
        cout<<"best_cluster.max_imgx:  "<<best_cluster.max_imgx<<endl;
        cout<<"best_cluster.max_imgy:  "<<best_cluster.max_imgy<<endl;

        cv::rectangle(img, cv::Point(best_cluster.min_imgx, best_cluster.min_imgy), cv::Point(best_cluster.max_imgx, best_cluster.max_imgy), cv::Scalar(0,0,255),3,1,0);

        all_det_res.detection_results[i].position.latitude = best_cluster.x;
        all_det_res.detection_results[i].position.longtitude = best_cluster.y;
    }

    // for (auto point : lidarPointVector)
    // {
    //     float r = sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
    //     int red = min(255, (int)(255 * abs((r - max_dis) / max_dis)));
    //     int green = min(255, (int)(255 * (1 - abs((r - max_dis) / max_dis))));
    //     cv::circle(img, cv::Point(point.imgx, point.imgy), 2, cv::Scalar(0, green, red), -1);
    // }

    // cout<<"4444444"<<endl;
    cv::imshow("img", img);
    cv::waitKey();
    res.detResultWithPosition = all_det_res;

    // res.position.latitude = 1;
    // res.position.longtitude = 1;

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_local_coord_server");
    ros::NodeHandle n;

    ros::Publisher pub_scan_ = n.advertise<sensor_msgs::PointCloud2>("velodyne_points", 1000);
    ros::ServiceServer service = n.advertiseService("get_local_coord", get_local_coord);
    ROS_INFO("Ready to get local coord server.");
    ros::spin();

    return 0;
}
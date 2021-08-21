/**
  *pcap_convert.h
  *brief:将pcap中的HDL-32数据转换为pointcloud2的msg发布出来
  *author:Yang Chenglin
  *date:20210728
  **/
#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <ctime>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <rosgraph_msgs/Clock.h>
#include "std_msgs/String.h"
#include "Eigen/Dense"
#include "pcap.h"

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

class pcap_convert
{
public:
    pcap_convert(ros::NodeHandle &nh, ros::NodeHandle &nh_priv);
    ~pcap_convert();

    std::string pcap_file_; //文件名
    int packet_num_per_scan_; //每一帧点云中包含的packet数量

    int readpacket(int key); //根据传入的令牌key值来读取packet数据
private:
    pcap_t *pcap_;
    char errbuf_[PCAP_ERRBUF_SIZE];

    bool pub_virtual_clock_;

    ros::Rate loop_rate_;

    float last_azimuth_; //上一个数据组的角度值
    Eigen::VectorXf firing_seq_; //存储32个线束对应的角度
    Eigen::VectorXf sin_lookup_table_;
    Eigen::VectorXf cos_lookup_table_; //为了提升运算速度，使用预先计算好的正弦余弦表
    Eigen::VectorXf ring_id_;

    int cur_packet_num_ = 0;
    int cur_point_num_ = 0;

    int32_t cur_time_sec_;
    int32_t cur_time_nsec_;

    long time_unix;
    long time_us;

    float bottom_angle_ = 30.67;
    float ang_res_y = 41.33/float(32-1);

    POINT temp_point_;
    CLOUD_PTR temp_cloud_;

    ros::Publisher pub_scan_;
    ros::Publisher pub_clock_;
    
    void readblock(const u_char *start); //处理packet内的每一个block
    void publish_scan();
    void reset();
    int unix_convert(long time_in);
};
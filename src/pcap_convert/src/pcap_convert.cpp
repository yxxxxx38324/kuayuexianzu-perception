/**
  *pcap_convert.h
  *brief:将pcap中的HDL-32数据转换为pointcloud2的msg发布出来
  *author:Yang Chenglin
  *date:20210728
  **/
#include "pcap_convert.h"

pcap_convert::pcap_convert(ros::NodeHandle &nh, ros::NodeHandle &nh_priv) : firing_seq_(32), sin_lookup_table_(32), cos_lookup_table_(32), loop_rate_(10), ring_id_(32)
{
    pcap_ = NULL;
    nh_priv.param<std::string>("pcap_file", pcap_file_, "/home/ycl/kyxz2020/Raw-022/Raw-022-HDL32.pcap");
    nh_priv.param<int>("packet_num_per_scan", packet_num_per_scan_, 181);
    nh_priv.param<bool>("pub_virtual_clock", pub_virtual_clock_, false);

    temp_cloud_.reset(new CLOUD());

    pub_scan_ = nh.advertise<sensor_msgs::PointCloud2>("velodyne_points", 1000);
    pub_clock_ = nh.advertise<rosgraph_msgs::Clock>("/clock", 100);

    std::cout << "vector initialization" << std::endl;

    std::cout << "vector enumbered" << std::endl;
    firing_seq_ << -30.67, -9.33, -29.33, -8.00, -28.00, -6.67, -26.67, -5.33,
        -25.33, -4.00, -24.00, -2.67, -22.67, -1.33, -21.33, 0.00,
        -20.00, 1.33, -18.67, 2.67, -17.33, 4.00, -16.00, 5.33, -14.67,
        6.67, -13.33, 8.00, -12.00, 9.33, -10.67, 10.67; //输入各个firing对应的角度

    sin_lookup_table_ << sin(-30.67 * M_PI / 180.0), sin(-9.33 * M_PI / 180.0), sin(-29.33 * M_PI / 180.0), sin(-8.00 * M_PI / 180.0),
        sin(-28.00 * M_PI / 180.0), sin(-6.67 * M_PI / 180.0), sin(-26.67 * M_PI / 180.0), sin(-5.33 * M_PI / 180.0),
        sin(-25.33 * M_PI / 180.0), sin(-4.00 * M_PI / 180.0), sin(-24.00 * M_PI / 180.0), sin(-2.67 * M_PI / 180.0),
        sin(-22.67 * M_PI / 180.0), sin(-1.33 * M_PI / 180.0), sin(-21.33 * M_PI / 180.0), sin(0.00 * M_PI / 180.0),
        sin(-20.00 * M_PI / 180.0), sin(1.33 * M_PI / 180.0), sin(-18.67 * M_PI / 180.0), sin(2.67 * M_PI / 180.0),
        sin(-17.33 * M_PI / 180.0), sin(4.00 * M_PI / 180.0), sin(-16.00 * M_PI / 180.0), sin(5.33 * M_PI / 180.0),
        sin(-14.67 * M_PI / 180.0), sin(6.67 * M_PI / 180.0), sin(-13.33 * M_PI / 180.0), sin(8.00 * M_PI / 180.0),
        sin(-12.00 * M_PI / 180.0), sin(9.33 * M_PI / 180.0), sin(-10.67 * M_PI / 180.0), sin(10.67 * M_PI / 180.0);

    cos_lookup_table_ << cos(-30.67 * M_PI / 180.0), cos(-9.33 * M_PI / 180.0), cos(-29.33 * M_PI / 180.0), cos(-8.00 * M_PI / 180.0),
        cos(-28.00 * M_PI / 180.0), cos(-6.67 * M_PI / 180.0), cos(-26.67 * M_PI / 180.0), cos(-5.33 * M_PI / 180.0),
        cos(-25.33 * M_PI / 180.0), cos(-4.00 * M_PI / 180.0), cos(-24.00 * M_PI / 180.0), cos(-2.67 * M_PI / 180.0),
        cos(-22.67 * M_PI / 180.0), cos(-1.33 * M_PI / 180.0), cos(-21.33 * M_PI / 180.0), cos(0.00 * M_PI / 180.0),
        cos(-20.00 * M_PI / 180.0), cos(1.33 * M_PI / 180.0), cos(-18.67 * M_PI / 180.0), cos(2.67 * M_PI / 180.0),
        cos(-17.33 * M_PI / 180.0), cos(4.00 * M_PI / 180.0), cos(-16.00 * M_PI / 180.0), cos(5.33 * M_PI / 180.0),
        cos(-14.67 * M_PI / 180.0), cos(6.67 * M_PI / 180.0), cos(-13.33 * M_PI / 180.0), cos(8.00 * M_PI / 180.0),
        cos(-12.00 * M_PI / 180.0), cos(9.33 * M_PI / 180.0), cos(-10.67 * M_PI / 180.0), cos(10.67 * M_PI / 180.0);

    ROS_INFO("Opening PCAP file: \"%s\"", pcap_file_.c_str());
    if ((pcap_ = pcap_open_offline(pcap_file_.c_str(), errbuf_)) == NULL)
    {
        ROS_FATAL("Error occurs when opening PCAP");
        return;
    } //在构造函数中打开PCAP文件存储在类内，后续读取由调用类的外部代码负责
}

pcap_convert::~pcap_convert()
{
}

/**
  *brief:控制类从pcap文件中读取一个packet，并完成点云的转换和赋值，返回值判断这个packet是否有效
  *author:Yang Chenglin
  *date:20210728
  **/
int pcap_convert::readpacket(int key)
{
    struct pcap_pkthdr *header;
    const u_char *pkt_data;
    int res;
    if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0)
    {
        if (header->caplen != header->len)
        {
            ROS_WARN("cap_len is not equal to len, packet abandoned");
            return 0;
        }
        time_unix = header->ts.tv_sec; //直接得到的UNIX时间戳
        time_us = header->ts.tv_usec;

        int time_sec = unix_convert(time_unix); //获取秒数据的高位
        int time_end = time_us / 1000;          //获取毫秒数据并截断
        int time_nsec = time_end * 1000000;
        //int time_nsec = time_us * 1000;

        cur_time_sec_ = time_sec;
        cur_time_nsec_ = time_nsec;

        ros::Time time_stamp(cur_time_sec_, cur_time_nsec_);
        rosgraph_msgs::Clock clock_now;
        clock_now.clock = time_stamp;

        //发布虚拟时钟
        if (pub_virtual_clock_)
        {
            pub_clock_.publish(clock_now);
        }

        for (int i = 0; i < 11; i++)
        {
            const u_char *block_data = pkt_data + 42 + i * 100;
            readblock(block_data);
        }
    }

    if (res == -2) //到达文件尾部
    {
        return 0;
    }

    cur_packet_num_++;

    /*
    if(cur_packet_num_ == packet_num_per_scan_)
    {
        publish_scan();
        reset();
        std::cout<<"now there are points in pointcloud: "<<temp_cloud_->points.size()<<std::endl;
    }
    */
}

/**
  *brief:根据提供的block起始指针来完成对一个标准firing的点转换
  *author:Yang Chenglin
  *date:20210728
  **/
void pcap_convert::readblock(const u_char *start)
{
    u_int16_t azimuth_priv;
    memcpy(&azimuth_priv, start + 2, 2);
    float azimuth = ((float)azimuth_priv) / 100;
    if (azimuth == 0 || azimuth > 360) //等于0的角度的block不要，非法的azimuth也不要
    {
        return;
    }
    float sin_azimuth = sin(azimuth * M_PI / 180.0);
    float cos_azimuth = cos(azimuth * M_PI / 180.0);
    //std::cout<<"Azimuth is"<<azimuth<<std::endl;
    for (int i = 0; i < 31; i++)
    {
        u_int16_t distance_priv;
        u_int8_t reflectivity;
        memcpy(&distance_priv, start + 4 + i * 3, 2);
        memcpy(&reflectivity, start + 6 + i * 3, 1);
        if (distance_priv == 0)
        {
            continue;
        }
        float distance = ((float)distance_priv) * 102.308 / 51154;
        POINT point;
        point.x = -((float)distance) * cos_lookup_table_(i) * sin_azimuth;
        point.y = -((float)distance) * cos_lookup_table_(i) * cos_azimuth;
        point.z = ((float)distance) * sin_lookup_table_(i);
        point.intensity = (float)reflectivity;

        float vert_angle = atan2(point.z, sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int cal_ring = (vert_angle + bottom_angle_) / ang_res_y;

        point.ring = cal_ring;
        /*
        if(i == 1)
        {
            std::cout<<"x is: "<<point.x<<" "
                 <<"y is: "<<point.y<<" "
                 <<"z is: "<<point.z<<std::endl;
        }
        */
        //std::cout<<"ring id is: "<<i - cal_ring<<std::endl;
        //std::cout<<"cal_ring id is: "<<cal_ring<<std::endl;

        temp_cloud_->points.push_back(point);
        cur_point_num_++;
    }

    if (azimuth < 1 && (360 - last_azimuth_) < 1) //当前的角度已经在360度附近
    {
        if (temp_cloud_->points.size() < 1000) //特异性检查，有情况就丢包
        {
            reset();
        }
        else
        {
            publish_scan();
            reset();
        }
    }

    last_azimuth_ = azimuth;
}

/**
  *brief:类内点云转换为PL2数据并发布
  *author:Yang Chenglin
  *date:20210728
  **/
void pcap_convert::publish_scan()
{
    //std::cout<<"now there are points in pointcloud: "<<temp_cloud_->points.size()<<std::endl;
    temp_cloud_->height = 1;
    temp_cloud_->width = cur_point_num_;
    sensor_msgs::PointCloud2 pl2_msg;
    pcl::toROSMsg(*temp_cloud_, pl2_msg);
    pl2_msg.header.frame_id = "kyxz_velodyne";
    //记得打上时间戳
    ros::Time time_stamp(cur_time_sec_, cur_time_nsec_);
    ros::Time time_now = ros::Time::now();
    std::cout << "lidar time_stamp sec is: " << time_stamp.sec << "nsec is: " << time_stamp.nsec << std::endl;
    //std::cout<<"ros time_stamp sec is: "<<time_now.sec<<std::endl;
    pl2_msg.header.stamp = time_stamp;
    pub_scan_.publish(pl2_msg);
    loop_rate_.sleep();
}

/**
  *brief:发布完一帧点云后将相关变量清零
  *author:Yang Chenglin
  *date:20210728
  **/
void pcap_convert::reset()
{
    temp_cloud_.reset(new CLOUD());
    cur_packet_num_ = 0;
    cur_point_num_ = 0;
}

/**
  *brief:将unix时间戳转换为东八区当日零点开始总秒数
  *author:Yang Chenglin
  *date:20210728
  **/
int pcap_convert::unix_convert(long time_in)
{
    time_t time = time_in;
    struct tm tm = *localtime(&time);
    char strTime[10] = {0};
    int buf_len = sizeof(strTime);
    strftime(strTime, buf_len - 1, "%H%M%S", &tm);
    strTime[buf_len - 1] = '\0';
    //std::cout<<strTime<<std::endl;
    int hour = 10 * (int)(strTime[0] - '0') + (int)(strTime[1] - '0');
    int minute = 10 * (int)(strTime[2] - '0') + (int)(strTime[3] - '0');
    int second = 10 * (int)(strTime[4] - '0') + (int)(strTime[5] - '0');
    //std::cout<<"hour: "<<hour<<"minute: "<<minute<<"second: "<<second<<std::endl;
    int all_sec = hour * 3600 + minute * 60 + second;
    //std::cout<<"all_sec: "<<all_sec<<std::endl;
    return all_sec;
}
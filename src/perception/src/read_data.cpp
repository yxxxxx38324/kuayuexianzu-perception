#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <iomanip>
using namespace std;
 
// OpenCV includes
#include <opencv2/video.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
 
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

// std::map<char,int>
std::map<string,string> get_image_dic(){
    ifstream inFile("/home/young/Raw-001/Raw-001-Camera-Timestamp.csv", ios::in);
    if (!inFile)
    {
        cout << "open Camera-Timestamp.csv failed" << endl;
        exit(1);
    }
    std::map<string,string> image_dic;
    int i = 0;
    string line;
    while (getline(inFile, line))//getline(inFile, line)表示按行读取CSV文件中的数据
    {
        string timestamp, filename;
        istringstream sin(line); //将整行字符串line读入到字符串流sin中

        getline(sin, timestamp, ','); //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符 
        cout<< timestamp <<" ";//将刚刚读取的字符串转换成int

        getline(sin, filename); //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符 
        cout << filename << " "<<endl;//将刚刚读取的字符串转换成int
        i++;

        image_dic[timestamp] = filename;
    }
    inFile.close();
    cout << "共读取了：" << i << "行" << endl;
    cout << "读取数据完成" << endl;
    return image_dic;

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "cam_data_pub");
    ros::NodeHandle nh;

    /****************获取csv文件为dic，并根据时间戳获取对应的图片文件名****************/
    std::map<string,string> image_dic;
    image_dic = get_image_dic();
    string timestamp;
    cout << "Enter the timestamp：" << endl;
    cin>>timestamp;
    string image_name = image_dic[timestamp];
    //csv中文件名前面多了2个空格
    image_name.erase(0,2);
    cout<< "get image: "<< image_name <<endl;

    /****************通过文件名进行读取并发布ros话题****************/
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);
    Mat image= imread("/home/young/Raw-001/Raw-001-Camera/" + image_name, CV_LOAD_IMAGE_COLOR);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
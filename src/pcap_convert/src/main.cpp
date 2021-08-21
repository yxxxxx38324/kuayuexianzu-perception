#include "pcap_convert.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcap_convert");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    pcap_convert ins(nh, nh_priv);
    int key = 1;
    while(ins.readpacket(key) != 0)
    {
        
    }
    while(ros::ok()){
        ros::spinOnce();
    }
    
    return 0;
}

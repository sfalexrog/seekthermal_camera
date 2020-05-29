#include "seekthermal_camera/seekthermal_ros.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "seekcam_node");
    ros::NodeHandle nh, nh_priv("~");
    seekthermal_camera::SeekThermalRos seek("seekcam_node", nh, nh_priv);
    while (ros::ok())
    {
        if (!seek.publish())
        {
            seek.init();
        }
        ros::spinOnce();
    }
    return 0;
}

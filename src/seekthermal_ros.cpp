#include <seekthermal_camera/seekthermal_ros.hpp>
#include <seek_thermal/seek.h>
#include <seek_thermal/SeekLogging.h>
#include <cv_bridge/cv_bridge.h>

namespace seekthermal_camera
{

namespace
{

/**
 * Logger trampoline
 */
void seekRosCallback(LibSeek::Severity severity, const char* message, void* userptr)
{
    SeekThermalRos* instance = reinterpret_cast<SeekThermalRos*>(userptr);
    ros::console::levels::Level ros_severity;
    switch(severity)
    {
        case LibSeek::Severity::Debug:
            ros_severity = ros::console::levels::Level::Debug;
            break;
        case LibSeek::Severity::Error:
            ros_severity = ros::console::levels::Level::Error;
            break;
        default:
            ros_severity = ros::console::levels::Level::Fatal;
    }
    instance->log(ros_severity, message);
}

}

SeekThermalRos::SeekThermalRos(const std::string nodeName, ros::NodeHandle& nh, ros::NodeHandle& nh_priv) : nodeName(nodeName), nh(nh), nh_priv(nh_priv)
{
    LibSeek::setLogFn(seekRosCallback, this);
    std::string model = nh_priv.param<std::string>("type", "seekpro");
    std::string cameraName = nh_priv.param<std::string>("camera_name", "seekpro");
    std::string cameraInfoUrl = nh_priv.param<std::string>("camera_info_url", "");
    frame_id = nh_priv.param<std::string>("frame_id", "seekpro_optical");
    if (model == "seekpro")
    {
        seek_cam.reset(new LibSeek::SeekThermalPro());
    }
    else if (model == "seek")
    {
        seek_cam.reset(new LibSeek::SeekThermal());
    }
    else
    {
        ROS_ERROR_STREAM_NAMED(nodeName, "Unknown model type: " << model);
        return;
    }

    if (!seek_cam->open())
    {
        ROS_ERROR_STREAM_NAMED(nodeName, "Failed to initialize camera");
        return;
    }

    
    info_manager.reset(new camera_info_manager::CameraInfoManager(nh, cameraName, cameraInfoUrl));

    image_transport::ImageTransport it(nh_priv);
    pub = it.advertiseCamera("image_raw", 1);
    normalized_pub = it.advertise("image_norm", 1);
}

SeekThermalRos::~SeekThermalRos() {}

void SeekThermalRos::log(ros::console::levels::Level severity, const char* message)
{
    ROS_LOG_STREAM(severity, std::string(ROSCONSOLE_NAME_PREFIX) + "." + nodeName, message);
}


bool SeekThermalRos::publish()
{
    if (!seek_cam) return false;
    if (!info_manager) return false;
    cv::Mat thermalImg;
    if(!seek_cam->read(thermalImg))
    {
        ROS_ERROR_STREAM_NAMED(nodeName, "Failed to read thermal image");
        ros::Duration(0.1).sleep();
        return false;
    }
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = frame_id;

    cv_bridge::CvImagePtr pubImg = boost::make_shared<cv_bridge::CvImage>(header, "mono16", thermalImg);
    sensor_msgs::CameraInfoPtr cameraInfo = boost::make_shared<sensor_msgs::CameraInfo>(info_manager->getCameraInfo());
    cameraInfo->header.stamp = header.stamp;
    cameraInfo->header.frame_id = header.frame_id;
    pub.publish(pubImg->toImageMsg(), cameraInfo);
    if (normalized_pub.getNumSubscribers() > 0)
    {
        cv_bridge::CvImagePtr normPubImg = boost::make_shared<cv_bridge::CvImage>(header, "mono16");
        cv::normalize(thermalImg, normPubImg->image, 0, 65536, cv::NORM_MINMAX);
        normalized_pub.publish(normPubImg->toImageMsg());
    }
}

}

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
        case LibSeek::Severity::Info:
            ros_severity = ros::console::levels::Level::Info;
            break;
        case LibSeek::Severity::Warning:
            ros_severity = ros::console::levels::Level::Warn;
        case LibSeek::Severity::Error:
            ros_severity = ros::console::levels::Level::Error;
            break;
        case LibSeek::Severity::Fatal:
            /* fall through */
        default:
            ros_severity = ros::console::levels::Level::Fatal;
    }
    instance->log(ros_severity, message);
}

}

SeekThermalRos::SeekThermalRos(const std::string nodeName, ros::NodeHandle& nh, ros::NodeHandle& nh_priv) : nodeName(nodeName), nh(nh), nh_priv(nh_priv)
{
    LibSeek::setLogFn(seekRosCallback, this);
    model = nh_priv.param<std::string>("type", "seekpro");
    std::string cameraName = nh_priv.param<std::string>("camera_name", "seekpro");
    std::string cameraInfoUrl = nh_priv.param<std::string>("camera_info_url", "");
    frame_id = nh_priv.param<std::string>("frame_id", "seekpro_optical");
    std::string ffc_filename = nh_priv.param<std::string>("ffc_image", "");
    if (ffc_filename != "")
    {
        ROS_INFO_STREAM_NAMED(nodeName, "Loading flat field calibration from " << ffc_filename);
        ffc_image = cv::imread(ffc_filename, cv::ImreadModes::IMREAD_UNCHANGED);
        if (ffc_image.empty())
        {
            ROS_ERROR_NAMED(nodeName, "Failed to load flat field calibration!");
        }
    }

    cal_beta = nh_priv.param<float>("cal_beta", 200.0f);
    linear_k = nh_priv.param<float>("linear_k", -1.5276f);
    linear_offset = nh_priv.param<float>("linear_offset", -470.8979f);

    info_manager.reset(new camera_info_manager::CameraInfoManager(nh, cameraName, cameraInfoUrl));

    image_transport::ImageTransport it(nh_priv);
    pub = it.advertiseCamera("image_raw", 1);
    thermal_image_pub = it.advertise("thermal_image", 1);
}

SeekThermalRos::~SeekThermalRos() {}

void SeekThermalRos::log(ros::console::levels::Level severity, const char* message)
{
    ROS_LOG_STREAM(severity, std::string(ROSCONSOLE_NAME_PREFIX) + "." + nodeName, message);
}

void SeekThermalRos::reset()
{
    if (model == "seekpro")
    {
        seek_cam.reset(new LibSeek::SeekThermalPro(ffc_image));
    }
    else if (model == "seek")
    {
        seek_cam.reset(new LibSeek::SeekThermal(ffc_image));
    }
    else
    {
        ROS_ERROR_STREAM_NAMED(nodeName, "Unknown model type: " << model);
        ros::shutdown();
    }
}

bool SeekThermalRos::init(int num_retries)
{
    while (num_retries-- != 0)
    {
        reset();
        if (seek_cam->open())
        {
            return true;
        }
        ROS_WARN_NAMED(nodeName, "Could not initialize camera, retrying");
        ros::Duration(0.5).sleep();
    }
    return false;
}

void SeekThermalRos::getThermalImage(const cv::Mat& src, cv::Mat& dst)
{
    dst.create(src.size(), CV_32FC1);
    
    auto device_k = [](float beta, int sensor_value) -> float {
        constexpr auto ref_temp = 297.0f;
        constexpr auto ref_sensor = 6616.0f;
        float part3 = std::log(sensor_value/ref_sensor);
        float parte = part3 / beta + 1.0 / ref_temp;
        return 1.0 / parte;
    }(cal_beta, seek_cam->device_temp_sensor());

    auto temp_k = [&](uint16_t raw_value) -> float {
        const float raw_scaled = raw_value * 330 / 16384.0f;
        return raw_scaled - device_k * linear_k + linear_offset - 273.0f;
    };

    for(auto row = 0; row < src.rows; ++row)
    {
        for(auto col = 0; col < src.cols; ++col)
        {
            dst.at<float>(row, col) = temp_k(src.at<uint16_t>(row, col));
        }
    }
}

bool SeekThermalRos::publish()
{
    if (!seek_cam || !seek_cam->isOpened()) return false;
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

    cv_bridge::CvImagePtr pubImg = boost::make_shared<cv_bridge::CvImage>(header, "16UC1", thermalImg);
    sensor_msgs::CameraInfoPtr cameraInfo = boost::make_shared<sensor_msgs::CameraInfo>(info_manager->getCameraInfo());
    cameraInfo->header.stamp = header.stamp;
    cameraInfo->header.frame_id = header.frame_id;
    pub.publish(pubImg->toImageMsg(), cameraInfo);
    if (thermal_image_pub.getNumSubscribers() > 0)
    {
        cv_bridge::CvImagePtr thermalImgProcessed = boost::make_shared<cv_bridge::CvImage>(header, "32FC1");
        getThermalImage(thermalImg, thermalImgProcessed->image);
        thermal_image_pub.publish(thermalImgProcessed->toImageMsg());
    }
    return true;
}

}

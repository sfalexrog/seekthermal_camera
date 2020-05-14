#include "seekthermal_camera/seekthermal_ros.hpp"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <thread>

namespace seekthermal_camera
{

class Camera : public nodelet::Nodelet
{
private:
    std::unique_ptr<SeekThermalRos> instance;
    std::thread pubThread;
    std::atomic<bool> shouldStop;

    void pubThreadFunc()
    {
        SeekThermalRos camera_connection(getName(), getNodeHandle(), getPrivateNodeHandle());
        while (!shouldStop)
        {
            if(!camera_connection.publish())
            {
                // TODO: do something about that
            }
        }
    }

    void onInit() override
    {
        shouldStop = false;
        pubThread = std::thread(&Camera::pubThreadFunc, this);
    }

    ~Camera()
    {
        shouldStop = true;
        pubThread.join();
    }
};

}


PLUGINLIB_EXPORT_CLASS(seekthermal_camera::Camera, nodelet::Nodelet)

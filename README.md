# Seek Thermal/Seek Thermal Pro ROS driver

Based on [Seek Thermal userspace driver](https://github.com/CopterExpress/libseek-thermal).

This package provides a node and a nodelet for interfacing with the Seek Thermal and Seek Thermal Pro thermal cameras, as well as interface-compatible devices (like HTI-Xintai HT-201).

## Setup

In order to be able to run the driver as an unprivileged user, you need to set permissions for the camera device. This can be done automatically by copying the [`config/99-seekthermal.rules`](config/99-seekthermal.rules) file from the repository to `/etc/udev/rules.d` and running the following commands:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

The user accessing the camera must be a member of `video` group.

## seekthermal_node

The node interfaces with the camera using `libusb`. The node will publish raw sensor data (as provided by the device) and calculated temperature values in Kelvin. The node also utilizes [camera_info_manager](http://wiki.ros.org/camera_info_manager) for optical calibration data.

**Note**: Since there is currently no proven way to read calibration data from the camera itself, you'll need to rely on specifying temperature calibration data.

### Published topics

* `~image_raw` (*sensor_msgs/Image*) - raw sensor data from the camera (16-bit unsigned integer);
* `~thermal_image` (*sensor_msgs/Image*) - temperature values in Kelvin (32-bit float);
* `~camera_info` (*sensor_msgs/CameraInfo*) - optical calibration data.

### Advertised services

* `~set_camera_info` (*sensor_msgs/SetCameraInfo*) - write new optical calibration data.

### Parameters

* `type` (*string*, default: `seekpro`) - camera model. Supported values are `seek` and `seekpro`;
* `camera_name` (*string*, default: `seekpro`) - camera name for [camera_info_manager](http://wiki.ros.org/camera_info_manager);
* `camera_info_url` (*string*, default: `""`) - url for camera info yaml file;
* `frame_id` (*string*, default: `seekpro_optical`) - frame name for published images;
* `ffc_image` (*string*, default: `""`) - path to flat field calibration file (16-bit PNG);
* `cal_beta` (*float*, default: `200.0`) - beta calibration coefficient for internal thermistor;
* `linear_k` (*float*, default: `-1.5276`) - linear model slope for device temperature compensation;
* `linear_offset` (*float*, default: `-470.8979`) - linear model offset for device temperature compensation;

### Flat field calibration

The node can use [flat-field calibration](https://github.com/CopterExpress/libseek-thermal#apply-additional-flat-field-calibration) to reduce noise and offset radial gradients.

## Nodelet

The package also provides the `seekthermal_camera/camera` nodelet that has all functionality of the node.

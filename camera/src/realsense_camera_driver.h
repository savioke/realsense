#pragma once
#ifndef REALSENSE_CAMERA_DRIVER
#define REALSENSE_CAMERA_DRIVER

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>

#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <iostream>
#include <boost/thread.hpp>
#include <thread>
#include <cstdlib>
#include <cctype>
#include <algorithm>
#include <sstream>
#include <memory>
#include <map>

#include <librealsense/librealsense/rs.hpp>
#include <dynamic_reconfigure/server.h>
#include <realsense_camera/camera_paramsConfig.h>

namespace realsense_camera
{
sensor_msgs::ImagePtr rawToFloatingPointConversion(sensor_msgs::ImagePtr raw_image);

class RealsenseCamera
{
public:
  ~RealsenseCamera();
  RealsenseCamera(ros::NodeHandle &pnh, ros::NodeHandle &nh);

  void setup();
  bool connectToCamera();
  void shutdown_camera();
  void poll();

private:
  sensor_msgs::CameraInfoPtr getCameraInfoPtr(rs_stream rs_strm);
  cv::Mat prepareStreamData (rs_stream rs_strm);
  void fillStreamEncoding ();
  void publishStreams ();
  void publishPointCloud (cv::Mat & image_color);
  void publishTransforms();
  void configCallback(realsense_camera::camera_paramsConfig &config, uint32_t level);
  void initDynamicReconfigure();

  ros::NodeHandle pnh_;
  ros::NodeHandle nh_;
  void check_error();

  // Default Constants.
  const int MAX_Z = 8;  // in meters
  const int DEPTH_HEIGHT = 360;
  const int DEPTH_WIDTH = 480;
  const int COLOR_HEIGHT = 480;
  const int COLOR_WIDTH = 640;
  const int DEPTH_FPS = 60;
  const int COLOR_FPS = 60;
  const rs_format DEPTH_FORMAT = RS_FORMAT_Z16;
  const rs_format COLOR_FORMAT = RS_FORMAT_RGB8;
  const rs_format IR1_FORMAT = RS_FORMAT_Y8;
  const rs_format IR2_FORMAT = RS_FORMAT_Y8;
  const char *SETTINGS_SERVICE = "camera/get_settings";
  const char *R200 = "R200";
  const static int STREAM_COUNT = 4;

 // Member Variables.
  boost::shared_ptr<boost::thread> device_thread_;
  boost::shared_ptr<boost::thread> transform_thread_;

  rs_error *rs_error_ = 0;
  rs_context *rs_context_;
  rs_device *rs_device_;

  std::string serial_number_;

  int color_height_;
  int color_width_;
  int depth_height_;
  int depth_width_;
  int depth_fps_;
  int color_fps_;
  bool is_device_started_;
  bool first_connection_;
  bool publishing_tf_;
  volatile bool reconfiguring_;
  boost::mutex mutex_;
  ros::Rate cycle_;
  std::vector<std::string> camera_configuration_;
  std::string camera_ = "R200";

  rs_option edge_options_[4] = {
    RS_OPTION_R200_AUTO_EXPOSURE_LEFT_EDGE,
    RS_OPTION_R200_AUTO_EXPOSURE_TOP_EDGE,
    RS_OPTION_R200_AUTO_EXPOSURE_RIGHT_EDGE,
    RS_OPTION_R200_AUTO_EXPOSURE_BOTTOM_EDGE
  };
  double edge_values_[4];

  sensor_msgs::CameraInfoPtr camera_info_ptr_[STREAM_COUNT];
  sensor_msgs::CameraInfo * camera_info_[STREAM_COUNT];
  image_transport::CameraPublisher camera_publisher_[STREAM_COUNT];
  image_transport::CameraPublisher pub_depth_;
  ros::Time time_stamp_;
  ros::Publisher pointcloud_publisher_;
  ros::ServiceServer get_options_service_;

  std::string base_frame_, depth_frame_, rgb_frame_, depth_optical_frame_, rgb_optical_frame_,
    ir_optical_frame_, ir2_optical_frame_;
  std::string frame_id_[STREAM_COUNT];
  std::string stream_encoding_[STREAM_COUNT];
  std::string mode_;
  std::map<std::string, std::string> config_;
  int stream_step_[STREAM_COUNT];

  struct option_str
  {
    rs_option opt;
    double min, max, step, value;
  };
  std::vector<option_str> options;
  dynamic_reconfigure::Server<realsense_camera::camera_paramsConfig> dynamic_reconf_server_;
  camera_paramsConfig last_config_;

};
}  // end namespace
#endif

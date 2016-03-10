#include "realsense_camera_driver.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "realsense_camera");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");
  ros::NodeHandle camera_nh("camera");
  realsense_camera::RealsenseCamera dvr(priv_nh, camera_nh);

  dvr.setup();
  while (node.ok())
    {
      dvr.poll();
      ros::spinOnce();
    }
  dvr.shutdown_camera();

  return 0;
}

# RealSense camera setup

## In Ubuntu 14.04

### Our instructions (as installed in beta50)

Make sure we have an updated system:
```
sudo apt-get update
sudo apt-get dist-upgrade
```
Install the 16.04 kernel, as it does not require patching:
```
sudo apt-get install linux-image-generic-lts-xenial
sudo reboot
```
Install realsense camera driver (this should also install `ros-indigo-librealsense` as a dependency):
```
sudo apt-get install ros-indigo-realsense-camera
```
Figure out the IDs for each camera. `ros-indigo-librealsense` has a binary which enumerates the cameras:
```
$  /etc/robot/ros/lib/librealsense/enumerate_devices | grep Serial
```
For example, in beta50:
```
savibot@beta50:~$ /etc/robot/ros/lib/librealsense/enumerate_devices | grep Serial
 Serial number: 2161307289
 Serial number: 2161307269
```
Now we are ready to modify the robot upstart script `/etc/init/savibot.conf`. We need to modify these env variables:
```
export CAMERA_TYPE=realsense
export CAMERA_DOWN_ID="2161307269"
export CAMERA_FORWARD_ID="2161307289"
```
Make sure that the IDs correspond to the forward/down cameras. A simple way of doing this is to check the video in the ROC.

### Intel suggested instructions

* sudo apt-get update
* sudo apt-get dist-upgrade
* sudo apt-get install libusb-1.0-0-dev
* git clone https://github.com/IntelRealSense/librealsense.git
* cd librealsense
* ./scripts/install_glfw3.sh
* sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
* ./scripts/install_dependencies-4.4.sh
* sudo reboot
* cd librealsense
* ./scripts/patch-uvcvideo-4.4.sh v4.4-wily
* make
* sudo make install

To test the cameras:
* ./bin/cpp-multicam

# Intel&reg; RealSense&trade; ROS
These are packages for using Intel RealSense cameras with ROS.  

www.intel.com/realsense  

www.ros.org  


Refer to the README.md file within each package for more details.

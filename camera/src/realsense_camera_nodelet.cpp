
/******************************************************************************
  Copyright (c) 2016, Intel Corporation
  All rights reserved.
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
  1. Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
  3. Neither the name of the copyright holder nor the names of its contributors
  may be used to endorse or promote products derived from this software without
  specific prior written permission.
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#include "realsense_camera_driver.h"
#include "realsense_camera_nodelet.h"

using namespace cv;
using namespace std;

// Nodelet dependencies.
#include <pluginlib/class_list_macros.h>
#include <tf/transform_broadcaster.h>

PLUGINLIB_EXPORT_CLASS (realsense_camera::RealsenseCameraNodelet, nodelet::Nodelet)

namespace realsense_camera
{
	RealsenseCameraNodelet::RealsenseCameraNodelet(){}

	RealsenseCameraNodelet::~RealsenseCameraNodelet()
	{
		if(running_)
		{
			NODELET_INFO("Shutting down driver thread");
			running_ = false;
			device_thread_->join();
			NODELET_INFO("Driver thread stopped");
		}
		dvr_->shutdown_camera();
	}

	void RealsenseCameraNodelet::RealsenseCameraNodelet::onInit()
	{
		ros::NodeHandle priv_nh(getPrivateNodeHandle());
		ros::NodeHandle node(getNodeHandle());
		ros::NodeHandle camera_nh(node, "camera");
		dvr_.reset(new realsense_camera::RealsenseCamera(priv_nh, camera_nh));
		dvr_->setup();

		running_ = true;
		device_thread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&RealsenseCameraNodelet::devicePoll, this)));
	}

	void RealsenseCameraNodelet::devicePoll()
	{
		while(running_)
		{
		  dvr_->poll();
		}
	}

} // end namespace

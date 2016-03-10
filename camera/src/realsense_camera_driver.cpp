#include "realsense_camera_driver.h"

using namespace cv;
using namespace std;

#include <tf/transform_broadcaster.h>

namespace realsense_camera
{
    RealsenseCamera::RealsenseCamera(ros::NodeHandle &pnh, ros::NodeHandle &nh): pnh_(pnh), nh_(nh),
    dynamic_reconf_server_(pnh), reconfiguring_(false), cycle_(1.0), first_connection_(true), publishing_tf_(true) {}

    RealsenseCamera::~RealsenseCamera()
    {
        if(device_thread_ != nullptr)
            device_thread_->join ();

        if (last_config_.publish_tf == true and transform_thread_ != nullptr)
        {
            transform_thread_->join();
        }

        // Stop device.
        if (is_device_started_ == true)
        {
            rs_stop_device (rs_device_, 0);
            rs_delete_context (rs_context_, &rs_error_);
            check_error ();
        }

        ROS_INFO_STREAM ("RealSense Camera - Stopping camera driver.");
        ros::shutdown ();
    }

    void RealsenseCamera::check_error ()
    {
        /*        if (rs_error_)
                  {
                  ROS_ERROR_STREAM ("RealSense Camera - Error calling " << rs_get_failed_function (rs_error_) << " ( "
                  << rs_get_failed_args (rs_error_) << " ): \n" << rs_get_error_message (rs_error_) << " \n");
                  rs_free_error (rs_error_);

                  ros::shutdown ();

                  exit (EXIT_FAILURE);
                  }*/
    }

    void RealsenseCamera::shutdown_camera()
    {
        ROS_INFO("Shutting down the camera ...");
        publishing_tf_ = false;
        if (last_config_.publish_tf == true and transform_thread_ != nullptr)
        {
            ROS_INFO("Waiting for transform thread ...");
            transform_thread_->join();
            ROS_INFO("Done");
        }
        publishing_tf_ = true;
        // Stop device.
        if (is_device_started_ == true)
        {
            ROS_INFO("Stopping device ...");
            rs_stop_device (rs_device_, 0);
            rs_delete_context (rs_context_, &rs_error_);
            is_device_started_ = false;
        }
    }

    void RealsenseCamera::poll()
    {
        if(!reconfiguring_)
        {
            if(!is_device_started_)
            {
                bool connected = connectToCamera();
                sleep(1.0);
                if(connected)
                    configCallback(last_config_, 0);
            }
            else {
                boost::mutex::scoped_lock lock(mutex_);
                publishStreams();  
            }
        } 
        if (!is_device_started_)
        {
            // device closed or poll not running, this avoids busy wait
            cycle_.sleep();
        }
    }


    void RealsenseCamera::setup()
    {
        is_device_started_ = false;

        pnh_.getParam("device_id", serial_number_);
        dynamic_reconf_server_.setCallback( boost::bind(&RealsenseCamera::configCallback, this, _1, _2));
        dynamic_reconf_server_.getConfigDefault(last_config_);

        std::string test;
        pnh_.param<bool>("enable_depth", last_config_.enable_depth, true);
        pnh_.param<bool>("enable_color", last_config_.enable_color, true);
        pnh_.param<bool>("enable_pointcloud", last_config_.enable_pointcloud, false);
        pnh_.param<bool>("publish_tf", last_config_.publish_tf, true);

        pnh_.param<string>("base_frame_id", base_frame_, "link");
        pnh_.param<string>("depth_frame_id", depth_frame_, "depth_frame");
        pnh_.param<string>("rgb_frame_id", rgb_frame_, "rgb_frame");
        pnh_.param<string>("depth_optical_frame_id", depth_optical_frame_, "depth_optical_frame");
        pnh_.param<string>("rgb_optical_frame_id", rgb_optical_frame_, "rgb_optical_frame");
        pnh_.param<string>("infrared_optical_frame_id", ir_optical_frame_, "ir_optical_frame");
        pnh_.param<string>("infrared2_optical_frame_id", ir2_optical_frame_, "ir2_optical_frame");

        frame_id_[RS_STREAM_DEPTH] = depth_optical_frame_;
        frame_id_[RS_STREAM_COLOR] = rgb_optical_frame_;
        frame_id_[RS_STREAM_INFRARED] = ir_optical_frame_;
        frame_id_[RS_STREAM_INFRARED2] = ir2_optical_frame_;

        // Set up the default dynamic reconfigure values to mimic those passed in
        initDynamicReconfigure();

        ros::NodeHandle nh;

        // Set up the topics.
        image_transport::ImageTransport it (nh);

        camera_publisher_[RS_STREAM_DEPTH] = it.advertiseCamera ("depth/image_raw", 1);
        camera_publisher_[RS_STREAM_INFRARED] = it.advertiseCamera ("ir1/image_raw", 1);
        camera_publisher_[RS_STREAM_INFRARED2] = it.advertiseCamera ("ir2/image_raw", 1);
        camera_publisher_[RS_STREAM_COLOR] = it.advertiseCamera ("rgb/image_raw", 1);
        pub_depth_ = it.advertiseCamera("depth/image",1);
    }

    void RealsenseCamera::initDynamicReconfigure()
    {
      // DON'T THINK THIS IS WOGKIN
      // Set dynamic reconfigure values for frames
      realsense_camera::camera_paramsConfig config;
      config.enable_depth = last_config_.enable_depth;
      config.enable_color = last_config_.enable_color;
      config.enable_pointcloud = last_config_.enable_pointcloud;
      config.publish_tf = last_config_.publish_tf;

      /*config.base_frame = last_config_.base_frame;
      config.depth_frame = last_config_.depth_frame;
      config.rgb_frame = last_config_.rgb_frame;
      config.depth_optical_frame = last_config_.depth_optical_frame;
      config.rgb_optical_frame = last_config_.rgb_optical_frame;
      config.ir_optical_frame = last_config_.ir_optical_frame;
      config.ir2_optical_frame = last_config_.ir2_optical_frame;*/
      
      dynamic_reconf_server_.setConfigDefault(config);
    }

    void RealsenseCamera::configCallback(realsense_camera::camera_paramsConfig &config, uint32_t level)
    {
        boost::mutex::scoped_lock lock(mutex_);
        reconfiguring_ = true;
        ROS_INFO("In config callback"); 
        // Device initialized yet?
        if(!is_device_started_)
        {
            last_config_ = config;
            reconfiguring_ = false;
            return;
        }

        if( (config.enable_depth && config.depth_mode != last_config_.depth_mode) || first_connection_)
        {
            shutdown_camera();
            first_connection_ = false;
            last_config_ = config;
            reconfiguring_ = false;
            return;
        }
        else if(!config.enable_depth)
            rs_disable_stream(rs_device_, RS_STREAM_DEPTH, &rs_error_);

        if(config.enable_color && config.rgb_mode != last_config_.rgb_mode)
        {
            shutdown_camera();
            last_config_ = config;
            reconfiguring_ = false;
            return;
        }
        else if(!config.enable_color)
            rs_disable_stream(rs_device_, RS_STREAM_COLOR, &rs_error_);

        //rs_start_device (rs_device_, 0);
        rs_set_device_option(rs_device_, RS_OPTION_COLOR_BACKLIGHT_COMPENSATION, config.COLOR_BACKLIGHT_COMPENSATION, 0);
        rs_set_device_option(rs_device_, RS_OPTION_COLOR_BRIGHTNESS, config.COLOR_BRIGHTNESS, 0);
        rs_set_device_option(rs_device_, RS_OPTION_COLOR_CONTRAST, config.COLOR_CONTRAST, 0);
        rs_set_device_option(rs_device_, RS_OPTION_COLOR_GAIN, config.COLOR_GAIN, 0);
        rs_set_device_option(rs_device_, RS_OPTION_COLOR_GAMMA, config.COLOR_GAMMA, 0);
        rs_set_device_option(rs_device_, RS_OPTION_COLOR_HUE, config.COLOR_HUE, 0);
        rs_set_device_option(rs_device_, RS_OPTION_COLOR_SATURATION, config.COLOR_SATURATION, 0);
        rs_set_device_option(rs_device_, RS_OPTION_COLOR_SHARPNESS, config.COLOR_SHARPNESS, 0);
        rs_set_device_option(rs_device_, RS_OPTION_COLOR_ENABLE_AUTO_WHITE_BALANCE, config.COLOR_ENABLE_AUTO_WHITE_BALANCE, 0);

        if(config.COLOR_ENABLE_AUTO_WHITE_BALANCE == 0) {
            rs_set_device_option(rs_device_, RS_OPTION_COLOR_WHITE_BALANCE, config.COLOR_WHITE_BALANCE, 0);
        }

        //R200 camera specific options
        rs_set_device_option(rs_device_, RS_OPTION_R200_LR_AUTO_EXPOSURE_ENABLED, config.R200_LR_AUTO_EXPOSURE_ENABLED, 0);

        if(config.R200_LR_AUTO_EXPOSURE_ENABLED == 0) {
            rs_set_device_option(rs_device_, RS_OPTION_R200_LR_EXPOSURE, config.R200_LR_EXPOSURE, 0);
        }

        rs_set_device_option(rs_device_, RS_OPTION_R200_LR_GAIN, config.R200_LR_GAIN, 0);
        rs_set_device_option(rs_device_, RS_OPTION_R200_EMITTER_ENABLED, config.R200_EMITTER_ENABLED, 0);
        rs_set_device_option(rs_device_, RS_OPTION_R200_DISPARITY_MULTIPLIER, config.R200_DISPARITY_MULTIPLIER, 0);

        if(config.R200_LR_AUTO_EXPOSURE_ENABLED == 1)
        {
            if(config.R200_AUTO_EXPOSURE_TOP_EDGE >= depth_height_) {
                config.R200_AUTO_EXPOSURE_TOP_EDGE = depth_height_ - 1;
            }
            if(config.R200_AUTO_EXPOSURE_BOTTOM_EDGE >= depth_height_) {
                config.R200_AUTO_EXPOSURE_BOTTOM_EDGE = depth_height_ - 1;
            }
            if(config.R200_AUTO_EXPOSURE_LEFT_EDGE >= depth_width_) {
                config.R200_AUTO_EXPOSURE_LEFT_EDGE = depth_width_ - 1;
            }
            if(config.R200_AUTO_EXPOSURE_RIGHT_EDGE >= depth_width_) {
                config.R200_AUTO_EXPOSURE_RIGHT_EDGE = depth_width_ - 1;
            }
            edge_values_[0] = config.R200_AUTO_EXPOSURE_LEFT_EDGE;
            edge_values_[1] = config.R200_AUTO_EXPOSURE_TOP_EDGE;
            edge_values_[2] = config.R200_AUTO_EXPOSURE_RIGHT_EDGE;
            edge_values_[3] = config.R200_AUTO_EXPOSURE_BOTTOM_EDGE;

            rs_set_device_options(rs_device_, edge_options_, 4, edge_values_, 0);
        }
        reconfiguring_ = false;
        last_config_ = config;
        ROS_INFO("Reconfiguring done");
    }

    bool RealsenseCamera::connectToCamera()
    {
        if (last_config_.enable_depth == false && last_config_.enable_color == false)
        {
            ROS_INFO_STREAM ("RealSense Camera - None of the streams are enabled. Exiting.");
            return false;
        }

        sleep(rand()/RAND_MAX*3.0);  // Not sure if this is necessary to avoid a race condition?
        rs_context_ = rs_create_context (RS_API_VERSION, &rs_error_);
        check_error ();

        int num_of_cameras = rs_get_device_count (rs_context_, NULL);
        ROS_INFO_STREAM(num_of_cameras << " connected");
        if (num_of_cameras < 1)
        {
            ROS_ERROR_STREAM ("RealSense Camera - No cameras are connected.");
            return false;
        }

        if(serial_number_.empty())
        {
            rs_device_ = rs_get_device (rs_context_, 0, &rs_error_);
            check_error ();
        }
        else // open a particular camera
        {
            ROS_INFO_STREAM("Trying to open " << serial_number_ << " for " << frame_id_[RS_STREAM_DEPTH]);
            bool found_device = false;
            // Open the specified device
            for(int i=0; i < num_of_cameras; i++)
            {
                ROS_INFO_STREAM("Getting device number " << i);
                sleep(rand()/RAND_MAX*3.0);  // Not sure if this is necessary to avoid a race condition?
                rs_device_ = rs_get_device (rs_context_, i, &rs_error_);
                check_error ();
                if(rs_device_ == nullptr)
                    continue;
                std::string serial_no = rs_get_device_serial(rs_device_, &rs_error_);
                check_error ();
                ROS_INFO_STREAM("Fetched device with serial " << serial_no);
                if( serial_no == serial_number_)
                {
                    ROS_INFO_STREAM("Found device " << serial_number_);
                    found_device = true;
                    break;
                }
            }
            if(!found_device)
            {
                ROS_ERROR("Could not find device with serial number %s", serial_number_.c_str());
                return false;

            }
        }

        ROS_INFO_STREAM ("RealSense Camera - Number of cameras connected: " << num_of_cameras);
        ROS_INFO_STREAM ("RealSense Camera - Firmware version: " <<
                rs_get_device_firmware_version (rs_device_, &rs_error_));
        check_error ();
        ROS_INFO_STREAM ("RealSense Camera - Name: " << rs_get_device_name (rs_device_, &rs_error_));
        check_error ();
        ROS_INFO_STREAM ("RealSense Camera - Serial no: " << rs_get_device_serial (rs_device_, &rs_error_));
        check_error ();

        // Enable streams, set to user params AFTER we can connect
        if(first_connection_)
        {
            if (last_config_.enable_depth == true)
            {
                rs_enable_stream_preset(rs_device_, RS_STREAM_DEPTH, RS_PRESET_BEST_QUALITY, &rs_error_);
                rs_enable_stream_preset(rs_device_, RS_STREAM_INFRARED, RS_PRESET_BEST_QUALITY, &rs_error_);
                rs_enable_stream_preset(rs_device_, RS_STREAM_INFRARED2, RS_PRESET_BEST_QUALITY, 0);
            }

            if (last_config_.enable_color == true)
            {
                ROS_INFO_STREAM ("RealSense Camera - Enabling Color Stream: manual mode");
                rs_enable_stream_preset(rs_device_, RS_STREAM_COLOR, RS_PRESET_BEST_QUALITY, &rs_error_);
            }
        }
        else {
            int depth_width, depth_height, depth_framerate;
            int rgb_width, rgb_height, rgb_framerate;
            rs_format depth_format, rgb_format, ir_format;
            rs_get_stream_mode(rs_device_, RS_STREAM_DEPTH, last_config_.depth_mode, &depth_width, &depth_height, &depth_format, &depth_framerate, &rs_error_);
            rs_get_stream_mode(rs_device_, RS_STREAM_INFRARED, last_config_.depth_mode, &depth_width, &depth_height, &ir_format, &depth_framerate, &rs_error_);
            rs_get_stream_mode(rs_device_, RS_STREAM_COLOR, last_config_.rgb_mode, &rgb_width, &rgb_height, &rgb_format, &rgb_framerate, &rs_error_);

            if(last_config_.enable_depth)
            {
                ROS_INFO("Reconfiguring depth stream");
                rs_enable_stream (rs_device_, RS_STREAM_DEPTH, depth_width, depth_height, depth_format, depth_framerate, &rs_error_);
                rs_enable_stream(rs_device_, RS_STREAM_INFRARED, depth_width, depth_height, ir_format, depth_framerate, &rs_error_);
                rs_enable_stream(rs_device_, RS_STREAM_INFRARED2, depth_width, depth_height, ir_format, depth_framerate, &rs_error_);
                ROS_INFO_STREAM("Enabling depth at " << depth_width << "x" << depth_height << " at " << depth_framerate << " fps");   
            }
            if(last_config_.enable_color)
            {
                rs_enable_stream (rs_device_, RS_STREAM_COLOR, rgb_width, rgb_height, rgb_format, rgb_framerate, &rs_error_);
                ROS_INFO_STREAM("Enabling rgb at " << rgb_width << "x" << rgb_height << " at " << rgb_framerate << " fps");
            }
        }

        // Start device.
        rs_start_device (rs_device_, &rs_error_);
        check_error ();

        sleep(1.0);

        int is_device_streaming = rs_is_device_streaming(rs_device_, &rs_error_);
        if(is_device_streaming)
            is_device_started_ = true;
        else
        {
            rs_delete_context (rs_context_, &rs_error_);
            sleep(1.0);
            return false;
        }

        fillStreamEncoding ();

        if (last_config_.publish_tf == true)
        {
            transform_thread_ = boost::shared_ptr < boost::thread > (new boost::thread (boost::bind
                        (&RealsenseCamera::publishTransforms, this)));
        }
        return true;
    }

    sensor_msgs::CameraInfoPtr RealsenseCamera::getCameraInfoPtr(rs_stream rs_strm)
    {
        uint32_t stream_index = (uint32_t) rs_strm;
        rs_intrinsics intrinsic;
        rs_get_stream_intrinsics (rs_device_, rs_strm, &intrinsic, &rs_error_);
        check_error ();

        sensor_msgs::CameraInfoPtr camera_info(new sensor_msgs::CameraInfo ());

        int width, height, framerate, mode;
        rs_format format;
        if(rs_strm == RS_STREAM_COLOR)
            mode = last_config_.rgb_mode;
        else 
            mode = last_config_.depth_mode;
        rs_get_stream_mode(rs_device_, rs_strm, mode, &width, &height, &format, &framerate, &rs_error_);

        camera_info->header.frame_id = frame_id_[stream_index];
        camera_info->width = width;
        camera_info->height = height ;

        camera_info->K.at (0) = intrinsic.fx;
        camera_info->K.at (2) = intrinsic.ppx;
        camera_info->K.at (4) = intrinsic.fy;
        camera_info->K.at (5) = intrinsic.ppy;
        camera_info->K.at (8) = 1;

        camera_info->P[0] = camera_info->K.at (0);
        camera_info->P[1] = 0;
        camera_info->P[2] = camera_info->K.at (2);
        camera_info->P[3] = 0;

        camera_info->P[4] = 0;
        camera_info->P[5] = camera_info->K.at (4);
        camera_info->P[6] = camera_info->K.at (5);
        camera_info->P[7] = 0;

        camera_info->P[8] = 0;
        camera_info->P[9] = 0;
        camera_info->P[10] = 1;
        camera_info->P[11] = 0;

        camera_info->distortion_model = "plumb_bob";

        for (int i = 0; i < 5; i++)
            camera_info->D.push_back (0);

        return camera_info;
    }

    cv::Mat RealsenseCamera::prepareStreamData (rs_stream rs_strm)
    {
        int depth_width, depth_height, depth_framerate;
        int rgb_width, rgb_height, rgb_framerate;
        rs_format depth_format, rgb_format;
        rs_get_stream_mode(rs_device_, RS_STREAM_DEPTH, last_config_.depth_mode, &depth_width, &depth_height, &depth_format, &depth_framerate, &rs_error_);
        rs_get_stream_mode(rs_device_, RS_STREAM_COLOR, last_config_.rgb_mode, &rgb_width, &rgb_height, &rgb_format, &rgb_framerate, &rs_error_);
        cv::Mat image;
        const uint16_t *image_depth16;
        switch (rs_strm)
        {
            case RS_STREAM_COLOR:
                image.create(rgb_height, rgb_width, CV_8UC3);
                image.data = (unsigned char *) (rs_get_frame_data (rs_device_, RS_STREAM_COLOR, 0));
                stream_encoding_[(uint32_t) RS_STREAM_COLOR] = "rgb8";
                stream_step_[(uint32_t) RS_STREAM_COLOR] = rgb_width * sizeof (unsigned char) * 3;
                break;
            case RS_STREAM_DEPTH:
                image.create(depth_height, depth_width, CV_16UC1);
                image_depth16 = reinterpret_cast <const uint16_t * >(rs_get_frame_data (rs_device_, RS_STREAM_DEPTH, 0));
                image.data = (unsigned char*)image_depth16;
                stream_encoding_[(uint32_t) RS_STREAM_DEPTH] = sensor_msgs::image_encodings::TYPE_16UC1;
                stream_step_[(uint32_t) RS_STREAM_DEPTH] = depth_width * sizeof (uint16_t);
                break;
            case RS_STREAM_INFRARED:
                image.create(depth_height, depth_width, CV_8UC1);
                image.data = (unsigned char *) (rs_get_frame_data (rs_device_, RS_STREAM_INFRARED, 0));
                stream_encoding_[(uint32_t) RS_STREAM_INFRARED] = sensor_msgs::image_encodings::TYPE_8UC1;
                stream_step_[(uint32_t) RS_STREAM_INFRARED] = depth_width * sizeof (unsigned char);
                break;
            case RS_STREAM_INFRARED2:
                image.create(depth_height, depth_width, CV_8UC1);
                image.data = (unsigned char *) (rs_get_frame_data (rs_device_, RS_STREAM_INFRARED2, 0));
                stream_encoding_[(uint32_t) RS_STREAM_INFRARED2] = sensor_msgs::image_encodings::TYPE_8UC1;
                stream_step_[(uint32_t) RS_STREAM_INFRARED2] = depth_width * sizeof (unsigned char);
                break;
            default:
                // no other streams supported
                break;
        }
        return image;
    }


    void RealsenseCamera::fillStreamEncoding ()
    {
        stream_encoding_[(uint32_t) RS_STREAM_COLOR] = "rgb8";
        stream_step_[(uint32_t) RS_STREAM_COLOR] = color_width_ * sizeof (unsigned char) * 3;
        stream_encoding_[(uint32_t) RS_STREAM_DEPTH] = sensor_msgs::image_encodings::TYPE_16UC1;
        stream_step_[(uint32_t) RS_STREAM_DEPTH] = depth_width_ * sizeof (uint16_t);
        stream_encoding_[(uint32_t) RS_STREAM_INFRARED] = sensor_msgs::image_encodings::TYPE_8UC1;
        stream_step_[(uint32_t) RS_STREAM_INFRARED] = depth_width_ * sizeof (unsigned char);
        stream_encoding_[(uint32_t) RS_STREAM_INFRARED2] = sensor_msgs::image_encodings::TYPE_8UC1;
        stream_step_[(uint32_t) RS_STREAM_INFRARED2] = depth_width_ * sizeof (unsigned char);
    }

    void RealsenseCamera::publishStreams ()
    {
        if(!is_device_started_)
            return;
        rs_wait_for_frames (rs_device_, &rs_error_);
        check_error ();
        time_stamp_ = ros::Time::now ();

        for (int stream_index = 0; stream_index < STREAM_COUNT; ++stream_index)
        {
            // Publish image stream only if there is at least one subscriber.
            if (camera_publisher_[stream_index].getNumSubscribers () > 0)
            {
                cv::Mat image = prepareStreamData ((rs_stream) stream_index);
                sensor_msgs::ImagePtr msg = cv_bridge::CvImage (std_msgs::Header (),
                        stream_encoding_[stream_index],
                        image).toImageMsg ();

                msg->header.frame_id = frame_id_[stream_index];
                msg->header.stamp = time_stamp_;        // Publish timestamp to synchronize frames.
                msg->width = image.cols;
                msg->height = image.rows;
                msg->is_bigendian = false;
                msg->step = image.step;


                sensor_msgs::CameraInfoPtr cam_info = getCameraInfoPtr((rs_stream) stream_index);
                cam_info->header.stamp = msg->header.stamp;
                cam_info->header.frame_id= msg->header.frame_id;
                camera_publisher_[stream_index].publish (msg, cam_info);

                if((rs_stream) stream_index == RS_STREAM_DEPTH)
                {
                    // publish the /image as well
                    sensor_msgs::ImageConstPtr floating_point_image = rawToFloatingPointConversion(msg);
                    pub_depth_.publish(floating_point_image, cam_info);
                }
            }
        }
    }

    void RealsenseCamera::publishTransforms()
    {
        // publish transforms for the cameras
        ROS_INFO_STREAM("RealSense Camera - Publishing camera transforms");
        tf::Transform tr;
        tf::Quaternion q;
        tf::TransformBroadcaster tf_broadcaster;
        rs_extrinsics z_extrinsic;

        // extrinsics are offsets between the cameras
        rs_get_device_extrinsics (rs_device_, RS_STREAM_DEPTH, RS_STREAM_COLOR, &z_extrinsic, &rs_error_); check_error ();

        ros::Duration sleeper(0.1); // 100ms

        while (ros::ok() && publishing_tf_)
        {
            // time stamp is future dated to be valid for given duration
            ros::Time time_stamp = ros::Time::now() + sleeper;

            // transform base frame to depth frame
            tr.setOrigin(tf::Vector3(z_extrinsic.translation[0], z_extrinsic.translation[1], z_extrinsic.translation[2]));
            tr.setRotation(tf::Quaternion(0, 0, 0, 1));
            tf_broadcaster.sendTransform(tf::StampedTransform(tr, time_stamp, base_frame_, depth_frame_));

            // transform depth frame to depth optical frame
            tr.setOrigin(tf::Vector3(0,0,0));
            q.setEuler( M_PI/2, 0.0, -M_PI/2 );
            tr.setRotation( q );
            tf_broadcaster.sendTransform(tf::StampedTransform(tr, time_stamp, depth_frame_, frame_id_[RS_STREAM_DEPTH]));

            // transform base frame to color frame (these are the same)
            tr.setOrigin(tf::Vector3(0,0,0));
            tr.setRotation(tf::Quaternion(0, 0, 0, 1));
            tf_broadcaster.sendTransform(tf::StampedTransform(tr, time_stamp, base_frame_, rgb_frame_));

            // transform color frame to color optical frame
            tr.setOrigin(tf::Vector3(0,0,0));
            q.setEuler( M_PI/2, 0.0, -M_PI/2 );
            tr.setRotation( q );
            tf_broadcaster.sendTransform(tf::StampedTransform(tr, time_stamp, rgb_frame_, frame_id_[RS_STREAM_COLOR]));

            sleeper.sleep(); // need sleep or transform won't publish correctly
        }
    }

    sensor_msgs::ImagePtr rawToFloatingPointConversion(sensor_msgs::ImagePtr raw_image)
    {
        static const float bad_point = std::numeric_limits<float>::quiet_NaN ();

        sensor_msgs::ImagePtr new_image = boost::make_shared<sensor_msgs::Image>();

        new_image->header = raw_image->header;
        new_image->width = raw_image->width;
        new_image->height = raw_image->height;
        new_image->is_bigendian = 0;
        new_image->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        new_image->step = sizeof(float) * raw_image->width;

        std::size_t data_size = new_image->width * new_image->height;
        new_image->data.resize(data_size * sizeof(float));

        const unsigned short* in_ptr = reinterpret_cast<const unsigned short*>(&raw_image->data[0]);
        float* out_ptr = reinterpret_cast<float*>(&new_image->data[0]);

        for (std::size_t i = 0; i < data_size; ++i, ++in_ptr, ++out_ptr) {
            if (*in_ptr == 0 || *in_ptr == 0x7FF) {
                *out_ptr = bad_point;
            } else {
                *out_ptr = static_cast<float>(*in_ptr) / 1000.0f;
            }
        }

        return new_image;
    }

}  // end namespace


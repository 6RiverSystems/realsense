/******************************************************************************
 Copyright (c) 2017, Intel Corporation
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

#include <string>
#include <vector>
#include <cstdlib>
#include <bitset>

#include <realsense_camera/r200_nodelet.h>

PLUGINLIB_EXPORT_CLASS(realsense_camera::R200Nodelet, nodelet::Nodelet)

namespace realsense_camera
{
  /*
   * Initialize the nodelet.
   */
  void R200Nodelet::onInit()
  {
    format_[RS_STREAM_COLOR] = RS_FORMAT_RGB8;
    encoding_[RS_STREAM_COLOR] = sensor_msgs::image_encodings::RGB8;
    cv_type_[RS_STREAM_COLOR] = CV_8UC3;
    unit_step_size_[RS_STREAM_COLOR] = sizeof(unsigned char) * 3;

    format_[RS_STREAM_DEPTH] = RS_FORMAT_Z16;
    encoding_[RS_STREAM_DEPTH] = sensor_msgs::image_encodings::TYPE_16UC1;
    cv_type_[RS_STREAM_DEPTH] = CV_16UC1;
    unit_step_size_[RS_STREAM_DEPTH] = sizeof(uint16_t);

    format_[RS_STREAM_INFRARED] = RS_FORMAT_Y8;
    encoding_[RS_STREAM_INFRARED] = sensor_msgs::image_encodings::TYPE_8UC1;
    cv_type_[RS_STREAM_INFRARED] = CV_8UC1;
    unit_step_size_[RS_STREAM_INFRARED] = sizeof(unsigned char);

    format_[RS_STREAM_INFRARED2] = RS_FORMAT_Y8;
    encoding_[RS_STREAM_INFRARED2] = sensor_msgs::image_encodings::TYPE_8UC1;
    cv_type_[RS_STREAM_INFRARED2] = CV_8UC1;
    unit_step_size_[RS_STREAM_INFRARED2] = sizeof(unsigned char);

    max_z_ = R200_MAX_Z;

    SyncNodelet::onInit();
  }

  /*
   * Get the nodelet parameters.
   */
  void R200Nodelet::getParameters()
  {
    BaseNodelet::getParameters();
    pnh_.param("ir2_frame_id", frame_id_[RS_STREAM_INFRARED2], DEFAULT_IR2_FRAME_ID);
    pnh_.param("ir2_optical_frame_id", optical_frame_id_[RS_STREAM_INFRARED2], DEFAULT_IR2_OPTICAL_FRAME_ID);
    pnh_.param("enable_ir2", enable_[RS_STREAM_INFRARED2], ENABLE_IR2);

    // set IR2 stream to match depth
    width_[RS_STREAM_INFRARED2] = width_[RS_STREAM_DEPTH];
    height_[RS_STREAM_INFRARED2] = height_[RS_STREAM_DEPTH];
    fps_[RS_STREAM_INFRARED2] = fps_[RS_STREAM_DEPTH];
  }

  /*
   * Advertise topics.
   */
  void R200Nodelet::advertiseTopics()
  {
    BaseNodelet::advertiseTopics();
    ros::NodeHandle ir2_nh(nh_, IR2_NAMESPACE);
    image_transport::ImageTransport ir2_image_transport(ir2_nh);
    camera_publisher_[RS_STREAM_INFRARED2] = ir2_image_transport.advertiseCamera(IR2_TOPIC, 1);
  }

  /*
   * Set Dynamic Reconfigure Server and return the dynamic params.
   */
  std::vector<std::string> R200Nodelet::setDynamicReconfServer()
  {
    dynamic_reconf_server_.reset(new dynamic_reconfigure::Server<realsense_camera::r200_paramsConfig>(pnh_));

    // Get dynamic options from the dynamic reconfigure server.
    realsense_camera::r200_paramsConfig params_config;
    dynamic_reconf_server_->getConfigDefault(params_config);
    std::vector<realsense_camera::r200_paramsConfig::AbstractParamDescriptionConstPtr> param_desc =
        params_config.__getParamDescriptions__();
    std::vector<std::string> dynamic_params;
    for (realsense_camera::r200_paramsConfig::AbstractParamDescriptionConstPtr param_desc_ptr : param_desc)
    {
      dynamic_params.push_back((* param_desc_ptr).name);
    }

    return dynamic_params;
  }

  /*
   * Start Dynamic Reconfigure Callback.
   */
  void R200Nodelet::startDynamicReconfCallback()
  {
    dynamic_reconf_server_->setCallback(boost::bind(&R200Nodelet::configCallback, this, _1, _2));
  }

  /*
   * Get the dynamic param values.
   */
  void R200Nodelet::configCallback(realsense_camera::r200_paramsConfig &config, uint32_t level)
  {
    // level is the ORing of all levels which have a changed value
    std::bitset<32> bit_level{level};

    if (bit_level.test(6))  // 2^6 = 64 : Depth Control Preset
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Setting dynamic camera options" <<
          " (r200_dc_preset=" << config.r200_dc_preset << ")");
    }
    else
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Setting dynamic camera options");
    }

    // set the depth enable
    BaseNodelet::setDepthEnable(config.enable_depth);

    // Set common options
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_BACKLIGHT_COMPENSATION, config.color_backlight_compensation, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_BRIGHTNESS, config.color_brightness, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_CONTRAST, config.color_contrast, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_GAIN, config.color_gain, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_GAMMA, config.color_gamma, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_HUE, config.color_hue, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_SATURATION, config.color_saturation, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_SHARPNESS, config.color_sharpness, 0);
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_ENABLE_AUTO_EXPOSURE,
    config.color_enable_auto_exposure, 0);
    if (config.color_enable_auto_exposure == 0)
    {
      rs_set_device_option(rs_device_, RS_OPTION_COLOR_EXPOSURE, config.color_exposure, 0);
    }
    rs_set_device_option(rs_device_, RS_OPTION_COLOR_ENABLE_AUTO_WHITE_BALANCE,
        config.color_enable_auto_white_balance, 0);
    if (config.color_enable_auto_white_balance == 0)
    {
      rs_set_device_option(rs_device_, RS_OPTION_COLOR_WHITE_BALANCE, config.color_white_balance, 0);
    }

    // Set R200 specific options
    rs_set_device_option(rs_device_, RS_OPTION_R200_LR_AUTO_EXPOSURE_ENABLED, config.r200_lr_auto_exposure_enabled, 0);
    if (config.r200_lr_auto_exposure_enabled == 0)
    {
      rs_set_device_option(rs_device_, RS_OPTION_R200_LR_GAIN, config.r200_lr_gain, 0);
      rs_set_device_option(rs_device_, RS_OPTION_R200_LR_EXPOSURE, config.r200_lr_exposure, 0);
    }

    if (config.r200_lr_auto_exposure_enabled == 1)
    {
      if (config.r200_auto_exposure_top_edge >= height_[RS_STREAM_DEPTH])
      {
        config.r200_auto_exposure_top_edge = height_[RS_STREAM_DEPTH] - 1;
      }
      if (config.r200_auto_exposure_bottom_edge >= height_[RS_STREAM_DEPTH])
      {
        config.r200_auto_exposure_bottom_edge = height_[RS_STREAM_DEPTH] - 1;
      }
      if (config.r200_auto_exposure_left_edge >= width_[RS_STREAM_DEPTH])
      {
        config.r200_auto_exposure_left_edge = width_[RS_STREAM_DEPTH] - 1;
      }
      if (config.r200_auto_exposure_right_edge >= width_[RS_STREAM_DEPTH])
      {
        config.r200_auto_exposure_right_edge = width_[RS_STREAM_DEPTH] - 1;
      }
      double edge_values_[4];
      edge_values_[0] = config.r200_auto_exposure_left_edge;
      edge_values_[1] = config.r200_auto_exposure_top_edge;
      edge_values_[2] = config.r200_auto_exposure_right_edge;
      edge_values_[3] = config.r200_auto_exposure_bottom_edge;
      rs_set_device_options(rs_device_, edge_options_, 4, edge_values_, 0);
    }

    rs_set_device_option(rs_device_, RS_OPTION_R200_EMITTER_ENABLED, config.r200_emitter_enabled, 0);

    // Depth Control Group Settings
    // NOTE: do NOT use the config.groups values as they are zero the first time called
    if (bit_level.test(5))  // 2^5 = 32 : Individual Depth Control settings
    {
      ROS_DEBUG_STREAM(nodelet_name_ << " - Setting Individual Depth Control");

      rs_set_device_option(rs_device_, RS_OPTION_R200_DEPTH_CONTROL_ESTIMATE_MEDIAN_DECREMENT,
          config.r200_dc_estimate_median_decrement, 0);
      rs_set_device_option(rs_device_, RS_OPTION_R200_DEPTH_CONTROL_ESTIMATE_MEDIAN_INCREMENT,
          config.r200_dc_estimate_median_increment, 0);
      rs_set_device_option(rs_device_, RS_OPTION_R200_DEPTH_CONTROL_MEDIAN_THRESHOLD,
          config.r200_dc_median_threshold, 0);
      rs_set_device_option(rs_device_, RS_OPTION_R200_DEPTH_CONTROL_SCORE_MINIMUM_THRESHOLD,
          config.r200_dc_score_minimum_threshold, 0);
      rs_set_device_option(rs_device_, RS_OPTION_R200_DEPTH_CONTROL_SCORE_MAXIMUM_THRESHOLD,
          config.r200_dc_score_maximum_threshold, 0);
      rs_set_device_option(rs_device_, RS_OPTION_R200_DEPTH_CONTROL_TEXTURE_COUNT_THRESHOLD,
          config.r200_dc_texture_count_threshold, 0);
      rs_set_device_option(rs_device_, RS_OPTION_R200_DEPTH_CONTROL_TEXTURE_DIFFERENCE_THRESHOLD,
          config.r200_dc_texture_difference_threshold, 0);
      rs_set_device_option(rs_device_, RS_OPTION_R200_DEPTH_CONTROL_SECOND_PEAK_THRESHOLD,
          config.r200_dc_second_peak_threshold, 0);
      rs_set_device_option(rs_device_, RS_OPTION_R200_DEPTH_CONTROL_NEIGHBOR_THRESHOLD,
          config.r200_dc_neighbor_threshold, 0);
      rs_set_device_option(rs_device_, RS_OPTION_R200_DEPTH_CONTROL_LR_THRESHOLD,
          config.r200_dc_lr_threshold, 0);

      // Preset also changed in the same update callback
      // This is either First callback special case, or both set via
      // dynamic configure command line.
      if (bit_level.test(6))  // 2^6 = 64 : Depth Control Preset
      {
          ROS_INFO_STREAM(nodelet_name_ << " - Initializing Depth Control Preset to " << config.r200_dc_preset);
          ROS_DEBUG_STREAM(nodelet_name_ << " - NOTICE: Individual Depth Control values " <<
              "set by params will be ignored; set r200_dc_preset=-1 to override.");
          rs_apply_depth_control_preset(rs_device_, config.r200_dc_preset);
      }
    }
    else
    { // Individual Depth Control not set
      if (bit_level.test(6))  // 2^6 = 64 : Depth Control Preset
      {
        ROS_DEBUG_STREAM(nodelet_name_ << " - Set Depth Control Preset to " << config.r200_dc_preset);
        rs_apply_depth_control_preset(rs_device_, config.r200_dc_preset);
      }
    }
  }

  /*
   * Get the camera extrinsics
   */
  void R200Nodelet::getCameraExtrinsics()
  {
    BaseNodelet::getCameraExtrinsics();

    // Get offset between base frame and infrared2 frame
    rs_get_device_extrinsics(rs_device_, RS_STREAM_INFRARED2, RS_STREAM_COLOR, &color2ir2_extrinsic_, &rs_error_);
    if (rs_error_)
    {
      ROS_ERROR_STREAM(nodelet_name_ << " - Verify camera is calibrated!");
    }
    checkError();
  }

  /*
   * Publish Static transforms.
   */
  void R200Nodelet::publishStaticTransforms()
  {
    BaseNodelet::publishStaticTransforms();

    if (enable_[RS_STREAM_INFRARED2])
    {
      tf::Quaternion q_i2io;
      geometry_msgs::TransformStamped c2i_msg;
      geometry_msgs::TransformStamped i2io_msg;

      // Transform color frame to infrared2 frame
      c2i_msg.header.stamp = transform_ts_;
      c2i_msg.header.frame_id = base_frame_id_;
      c2i_msg.child_frame_id = frame_id_[RS_STREAM_INFRARED2];
      c2i_msg.transform.translation.x =  color2ir2_extrinsic_.translation[2];
      c2i_msg.transform.translation.y = -color2ir2_extrinsic_.translation[0];
      c2i_msg.transform.translation.z = -color2ir2_extrinsic_.translation[1];
      c2i_msg.transform.rotation.x = 0;
      c2i_msg.transform.rotation.y = 0;
      c2i_msg.transform.rotation.z = 0;
      c2i_msg.transform.rotation.w = 1;
      static_tf_broadcaster_.sendTransform(c2i_msg);

      // Transform infrared2 frame to infrared2 optical frame
      q_i2io.setRPY(-M_PI/2, 0.0, -M_PI/2);
      i2io_msg.header.stamp = transform_ts_;
      i2io_msg.header.frame_id = frame_id_[RS_STREAM_INFRARED2];
      i2io_msg.child_frame_id = optical_frame_id_[RS_STREAM_INFRARED2];
      i2io_msg.transform.translation.x = 0;
      i2io_msg.transform.translation.y = 0;
      i2io_msg.transform.translation.z = 0;
      i2io_msg.transform.rotation.x = q_i2io.getX();
      i2io_msg.transform.rotation.y = q_i2io.getY();
      i2io_msg.transform.rotation.z = q_i2io.getZ();
      i2io_msg.transform.rotation.w = q_i2io.getW();
      static_tf_broadcaster_.sendTransform(i2io_msg);
    }
  }

  /*
   * Publish Dynamic transforms.
   */
  void R200Nodelet::publishDynamicTransforms()
  {
    tf::Transform tr;
    tf::Quaternion q;

    BaseNodelet::publishDynamicTransforms();

    if (enable_[RS_STREAM_INFRARED2])
    {
      // Transform color frame to infrared2 frame
      tr.setOrigin(tf::Vector3(
             color2ir2_extrinsic_.translation[2],
            -color2ir2_extrinsic_.translation[0],
            -color2ir2_extrinsic_.translation[1]));
      tr.setRotation(tf::Quaternion(0, 0, 0, 1));
      dynamic_tf_broadcaster_.sendTransform(tf::StampedTransform(tr, transform_ts_,
            base_frame_id_, frame_id_[RS_STREAM_INFRARED2]));

      // Transform infrared2 frame to infrared2 optical frame
      tr.setOrigin(tf::Vector3(0, 0, 0));
      q.setRPY(-M_PI/2, 0.0, -M_PI/2);
      tr.setRotation(q);
      dynamic_tf_broadcaster_.sendTransform(tf::StampedTransform(tr, transform_ts_,
            frame_id_[RS_STREAM_INFRARED2], optical_frame_id_[RS_STREAM_INFRARED2]));
    }
  }
}  // namespace realsense_camera

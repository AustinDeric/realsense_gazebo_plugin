#ifndef _GAZEBO_ROS_REALSENSE_PLUGIN_
#define _GAZEBO_ROS_REALSENSE_PLUGIN_

#include "realsense_gazebo_plugin/RealSensePlugin.h"

#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

#include <memory>
#include <string>

namespace gazebo_plugins {

class GazeboRosRealsense : public RealSensePlugin {

public:
  GazeboRosRealsense();

  /**
   * @brief Destroy the Gazebo Ros Realsense object
   * 
   */
  ~GazeboRosRealsense();

  /**
   * @brief Documentation Inherited.
   * 
   * @param _model 
   * @param _sdf 
   */
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /**
   * @brief Callback that publishes a received Depth Camera Frame as an ImageStamped message.
   * 
   */
  virtual void OnNewDepthFrame();

  /**
   * @brief Helper function to fill the pointcloud information
   * 
   * @param point_cloud_msg 
   * @param rows_arg 
   * @param cols_arg 
   * @param step_arg 
   * @param data_arg 
   * @return true 
   * @return false 
   */
  bool FillPointCloudHelper(sensor_msgs::PointCloud2 &point_cloud_msg, uint32_t rows_arg,
                            uint32_t cols_arg, uint32_t step_arg, void *data_arg);

  /**
   * @brief Callback that publishes a received Camera Frame as an ImageStamped message
   * 
   * @param cam 
   * @param pub 
   */
  virtual void OnNewFrame(const rendering::CameraPtr cam,
                          const transport::PublisherPtr pub);
  
  gazebo_ros::Node::SharedPtr ros_node_;

protected:
  boost::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  image_transport::CameraPublisher color_pub_, ir1_pub_, ir2_pub_, depth_pub_;
  sensor_msgs::Image image_msg_, depth_msg_;
  sensor_msgs::PointCloud2 pointcloud_msg_;

private:
  image_transport::ImageTransport *itnode_;

};
} // namespace gazebo_plugins
#endif // _GAZEBO_ROS_REALSENSE_PLUGIN_

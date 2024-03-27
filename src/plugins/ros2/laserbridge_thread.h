#ifndef _PLUGINS_ROS2_LASER_BRIDGE_H_
#define _PLUGINS_ROS2_LASER_BRIDGE_H_

#include <plugins/ros2/aspect/ros2.h>
#include <core/threading/thread.h>
#include <aspect/configurable.h>
#include <aspect/pointcloud.h>
#include <core/utils/refptr.h>

#include <mutex>

#include "sensor_msgs/msg/laser_scan.hpp"



class ROS2LaserBridgeThread : public fawkes::Thread,
                              public fawkes::ROS2Aspect,
                              public fawkes::ConfigurableAspect,
                              public fawkes::PointCloudAspect
{
public:
  ROS2LaserBridgeThread();
  virtual ~ROS2LaserBridgeThread();

  virtual void init();
  virtual void finalize();

private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

  fawkes::RefPtr<pcl::PointCloud<pcl::PointXYZ>> cloud_;

  std::mutex mutex_;

};

#endif // _PLUGINS_ROS2_LASER_BRIDGE_H_

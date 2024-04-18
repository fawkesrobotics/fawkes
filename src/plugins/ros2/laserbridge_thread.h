#ifndef _PLUGINS_ROS2_LASER_BRIDGE_H_
#define _PLUGINS_ROS2_LASER_BRIDGE_H_

#include <plugins/ros2/aspect/ros2.h>
#include <core/threading/thread.h>
#include <aspect/configurable.h>
#include <aspect/pointcloud.h>
#include <aspect/tf.h>
#include <aspect/clock.h>
#include <aspect/blocked_timing.h>
#include <aspect/blackboard.h>
#include <core/utils/refptr.h>

#include <interfaces/Position3DInterface.h>

#include <mutex>

#include "sensor_msgs/msg/laser_scan.hpp"



class ROS2LaserBridgeThread : public fawkes::Thread,
                              public fawkes::ROS2Aspect,
                              public fawkes::ClockAspect,
                              public fawkes::ConfigurableAspect,
                              public fawkes::TransformAspect,
                              public fawkes::BlockedTimingAspect,
                              public fawkes::BlackBoardAspect,
                              public fawkes::PointCloudAspect
{
public:
  ROS2LaserBridgeThread();
  virtual ~ROS2LaserBridgeThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

private:
  std::mutex main_loop;
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

  fawkes::RefPtr<pcl::PointCloud<pcl::PointXYZ>> cloud_;
  pcl::PointCloud<pcl::PointXYZ> ros2_cloud_;

  std::mutex mutex_;

  int beams_used_;
  std::string target_frame_;
	fawkes::Position3DInterface *if_front_dist_;
};

#endif // _PLUGINS_ROS2_LASER_BRIDGE_H_

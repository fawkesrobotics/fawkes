#include "laserbridge_thread.h"
#include <cmath>

// NEED TO INJECT INTO PCL_MANAGER THE front-filtered-1080 TOPIC IT IS JUST:
// A POINCLOUD WITH ALL MESSUREMNTS FROM THE FRONT LASER BUT FILTERED
// NOT IN THERE IS EVERYLINE LOWER THAN 0.04M AND NOT THE LINES BETWEEN 335 AND 765
//   filter-front-1080:
//     active: true
//     # URG input interface
//     # in/urg: Laser360Interface::Laser urg
//     in/sick-tim55x: Laser1080Interface::Laser front base
//     # URG filtered output interface
//     out/filtered: Laser1080Interface::Laser front-filtered-1080
//     filters:
//       1-min:
//         # Threshold for minimum value to get rid of erroneous beams on most
//         # black surfaces
//         type: min_circle
//         # Radius of minimum length; m
//         radius: 0.04
//       2-sector:
//         # Ignore a defined range of beams (e.g. those hitting the robot itself)
//         type: circle_sector
//         from: 765
//         to: 335

using namespace fawkes;
ROS2LaserBridgeThread::ROS2LaserBridgeThread() : Thread("LaserBridgeThread", Thread::OPMODE_WAITFORWAKEUP) {

}

ROS2LaserBridgeThread::~ROS2LaserBridgeThread() {
}


void ROS2LaserBridgeThread::init() {
  std::string ros_namespace = config->get_string_or_default("/ros2/namespace", "");

  laser_sub_ = node_handle->create_subscription<sensor_msgs::msg::LaserScan>(ros_namespace + "/front/sick_scan/scan", 10,
  std::bind(&ROS2LaserBridgeThread::laser_callback, this, std::placeholders::_1));
  cloud_ = new pcl::PointCloud<pcl::PointXYZ>();
  cloud_->width = 0;
  cloud_->height = 1;
  pcl_manager->add_pointcloud("front-filtered-1080", cloud_);
}

#define INCREMENT 2 * M_PI / 1080

void ROS2LaserBridgeThread::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // fawkes::RefPtr<pcl::PointCloud<pcl::PointXYZ>> cloud;
    // cloud = new pcl::PointCloud<pcl::PointXYZ>();
    // Assuming the laser scanner is horizontally mounted and scan is in the x-y plane
    if (scan->ranges.size() == 0) return;
    if (scan->ranges.size() != cloud_->size()){
        cloud_->points.resize(scan->ranges.size());
        cloud_->height = 1;
        cloud_->width = scan->ranges.size();
    }
    for (unsigned int i = 0; i < scan->ranges.size(); ++i) {
      pcl::PointXYZ &point = cloud_->points[i];
      if (scan->ranges[i] < scan->range_min || scan->ranges[i] > scan->range_max){
        point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
        continue;
      }

      float range = scan->ranges[i];
      float angle = scan->angle_min + i * scan->angle_increment;
      if(angle > 335 * INCREMENT && angle < 765 * INCREMENT) { //LEGACY
        point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
        continue;
      }

      point.x = range * cos(angle);
      point.y = range * sin(angle);
      point.z = 0;  // Since this is a 2D laser scan, z will be 0
    }
    // pcl_manager->set_cloud("front-filtered-1080", cloud);

}


void ROS2LaserBridgeThread::finalize() {

}

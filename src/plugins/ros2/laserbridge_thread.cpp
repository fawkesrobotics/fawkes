#include "laserbridge_thread.h"
#include <cmath>
#include <mutex>

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
#include <tf/types.h>

using namespace fawkes;
ROS2LaserBridgeThread::ROS2LaserBridgeThread() : Thread("LaserBridgeThread", Thread::OPMODE_WAITFORWAKEUP),
  TransformAspect(TransformAspect::ONLY_PUBLISHER, "ros2-laserbridge"),
 BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE), ros2_cloud_()

{}

ROS2LaserBridgeThread::~ROS2LaserBridgeThread() {
}


void ROS2LaserBridgeThread::init() {
  std::string ros_namespace = config->get_string_or_default("/ros2/namespace", "");

  laser_sub_ = node_handle->create_subscription<sensor_msgs::msg::LaserScan>("/" + ros_namespace + "/front/sick_scan/scan", 10,
  std::bind(&ROS2LaserBridgeThread::laser_callback, this, std::placeholders::_1));
  cloud_ = new pcl::PointCloud<pcl::PointXYZ>();
  cloud_->width = 0;
  cloud_->height = 1;
  cloud_->header.frame_id = "front_laser";
  pcl_manager->add_pointcloud("front-filtered-1080", cloud_);

	// read config values
	beams_used_   = config->get_int("plugins/laser-front-dist/number_beams_used");
	target_frame_ = config->get_string("plugins/laser-front-dist/target_frame");

	if_front_dist_ = blackboard->open_for_writing<Position3DInterface>(
	  config->get_string("plugins/laser-front-dist/output_result_interface").c_str());

}

#define INCREMENT 2 * M_PI / 1080

void ROS2LaserBridgeThread::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  std::lock_guard lock(main_loop);
    // Assuming the laser scanner is horizontally mounted and scan is in the x-y plane
    if (scan->ranges.size() == 0) return;
    if (scan->ranges.size() != ros2_cloud_.size()){
        ros2_cloud_.points.resize(scan->ranges.size());
        ros2_cloud_.height = 1;
        ros2_cloud_.width = scan->ranges.size();
    }
    for (unsigned int i = 0; i < scan->ranges.size(); ++i) {
      pcl::PointXYZ &point = ros2_cloud_.points[i];
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

    int center_beam_index = scan->ranges.size() / 2;

    float sum = 0.0;

    for (int i = center_beam_index - beams_used_ / 2; i < center_beam_index + beams_used_ / 2; i++) {
      if (!std::isnormal(scan->ranges[i])) {
        // this is invalid
        if_front_dist_->set_visibility_history(-1);
        if_front_dist_->write();
        return;
      }
      sum += scan->ranges[i];
    }

    float average = sum / (float)beams_used_;
    std::string frame_ = "front_laser";

    tf::Transform        transform(tf::create_quaternion_from_yaw(M_PI), tf::Vector3(average, 0, 0));
    Time                 time(clock);
    tf::StampedTransform stamped_transform(transform, time, frame_.c_str(), target_frame_.c_str());
    tf_publisher->send_transform(stamped_transform);

    // write result
    if_front_dist_->set_visibility_history(1);
    if_front_dist_->set_translation(0, average);
    if_front_dist_->set_frame(frame_.c_str());
    if_front_dist_->write();

}


void ROS2LaserBridgeThread::loop() {
  std::lock_guard lock(main_loop);
    if (cloud_->size() != ros2_cloud_.size()){
        cloud_->points.resize(ros2_cloud_.size());
        cloud_->height = 1;
        cloud_->width = ros2_cloud_.size();
    }
    for (unsigned int i = 0; i < cloud_->width; ++i) {
      cloud_->points[i] = ros2_cloud_[i];
    }


}
void ROS2LaserBridgeThread::finalize() {

}

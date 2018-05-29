/***************************************************************************
 *  time_offset_calibration.cpp - Laser time offset calibration
 *
 *  Created: Tue 18 Jul 2017 17:40:16 CEST 17:40
 *  Copyright  2017-2018  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "time_offset_calibration.h"

#include <tf/transformer.h>
#include <config/netconf.h>

#include <interfaces/MotorInterface.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/geometry.h>
#include <pcl/registration/icp.h>

#include <random>

using namespace fawkes;
using namespace std;

/** @class TimeOffsetCalibration "time_offset_calibration.h"
 *  Calibrate the time offset of a laser. This is done as follows:
 *    1. Move the robot to a place with some recognizable object in the laser,
 *       e.g., a corner
 *    2. Start rotating the robot
 *    3. Record a reference pointcloud
 *    4. Stop rotating
 *    5. Record a second pointcloud
 *    6. Compare the two pointclouds and update the time offset based on the
 *       angle between the two pointclouds.
 *
 *  @author Till Hofmann
 */

/** Constructor.
 *  @param laser The laser to get the data from
 *  @param motor The MotorInterface used to control the rotation of the robot
 *  @param tf_transformer The transformer to use to compute transforms
 *  @param config The network config to read from and write the time offset to
 *  @param config_path The config path to read from and write the time offset to
 */
TimeOffsetCalibration::TimeOffsetCalibration(LaserInterface *laser,
    MotorInterface *motor, tf::Transformer *tf_transformer, NetworkConfiguration
    *config, string config_path)
  : LaserCalibration(laser, tf_transformer, config, config_path),
    motor_(motor),
    step_(numeric_limits<float>::max())
  {}

/** Calibrate the time offset.
 *  Continuously execute the calibration procedure until the offset is small
 *  enough. To improve convergence rate, in each iteration, jump to the minimum
 *  with a certain probability based on the current cost and the minimal cost.
 *  The time offset is written to the config in each iteration. At the end, the
 *  time offset is always set to the offset with minimal cost.
 */
void TimeOffsetCalibration::calibrate() {
  float current_offset = config_->get_float(config_path_.c_str());
  map<float, float> costs;
  float min_cost = numeric_limits<float>::max();
  float min_offset = 0.;
  mt19937 random_gen;
  uniform_real_distribution<float> random_float_dist(0, 1);
  do {
    printf("Rotating bot with omega %f\n", omega_);
    for (uint i = 0; i < rotation_time_ * frequency_; i++) {
      MotorInterface::TransRotMessage *rot_message =
          new MotorInterface::TransRotMessage(0.f, 0.f, omega_);
      motor_->msgq_enqueue(rot_message);
      usleep(1e6/frequency_);
      motor_->read();
      if (motor_->omega() < 0.8 * omega_) {
        i = 0;
      }
    }
    printf("Taking snapshot (moving)\n");
    PointCloudPtr moving_cloud = get_lasercloud(laser_);
    MotorInterface::TransRotMessage *stop_message =
        new MotorInterface::TransRotMessage(0.f, 0.f, 0.f);
    printf("Stopping bot.\n");
    motor_->msgq_enqueue(stop_message);
    while (motor_->omega() > 0.02) {
      motor_->read();
    }
    usleep(50000);
    printf("Taking snapshot (resting)\n");
    PointCloudPtr rest_cloud;
    try {
      rest_cloud = get_lasercloud(laser_);
    } catch (Exception &e) {
      printf("Cloud not get pointcloud: %s\n", e.what_no_backtrace());
      continue;
    }
    float yaw;
    float current_cost;
    try {
      current_cost = get_matching_cost(rest_cloud, moving_cloud, &yaw);
    } catch (InsufficientDataException &e) {
      printf("Insufficient data: %s.\nPlease move the robot.\n",
          e.what_no_backtrace());
      continue;
    }
    float next_offset;
    float jump_probability =
        static_cast<float>((current_cost - min_cost)) / current_cost;
    float rand_01 = random_float_dist(random_gen);
    if (current_cost > min_cost && rand_01 > 1 - jump_probability) {
      printf("Setting back to minimum: %f -> %f (probability %f)\n",
          current_offset, min_offset, jump_probability);
      next_offset = min_offset;
      step_ = next_offset - current_offset;
    }  else {
      min_cost = current_cost;
      min_offset = current_offset;
      step_ = -0.05 * yaw / omega_;
      next_offset = current_offset + step_;
    }
    printf("Updating time offset from %f to %f (step %f), current cost %f\n",
        current_offset, next_offset, step_, current_cost);
    config_->set_float(config_path_.c_str(), next_offset);
    current_offset = next_offset;
    usleep(sleep_time_);
  } while (abs(step_) > 0.0005);
  printf("Setting to offset with minimal cost %f\n", min_offset);
  config_->set_float(config_path_.c_str(), min_offset);
}

/** Prepare the laser data for calibration.
 *  Convert the laser data into a pointcloud and filter it so it only contains
 *  data that is useful for calibration. In particular, restrict the data in x
 *  and y directions to the interval [-3,3], remove any points close to the
 *  robot in y direction, and limit the data in z direction to points above the
 *  ground and < 1m.
 *  @param laser The laser interface to read the unfiltered data from
 *  @return A filtered pointcloud
 */
PointCloudPtr
TimeOffsetCalibration::get_lasercloud(LaserInterface *laser) {
  laser->read();
  PointCloudPtr laser_cloud = laser_to_pointcloud(*laser);
  transform_pointcloud("odom", laser_cloud);
  pcl::PassThrough<Point> pass_x;
  pass_x.setInputCloud(laser_cloud);
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(-3., 3.);
  PointCloudPtr x_filtered(new PointCloud());
  pass_x.filter(*x_filtered);
  pcl::PassThrough<Point> pass_y;;
  pass_y.setInputCloud(x_filtered);
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimitsNegative(true);
  pass_y.setFilterLimits(-0.3, 0.3);
  PointCloudPtr xy_filtered_inner(new PointCloud());
  pass_y.filter(*xy_filtered_inner);
  pcl::PassThrough<Point> pass_y_outer;
  pass_y_outer.setInputCloud(xy_filtered_inner);
  pass_y_outer.setFilterFieldName("y");
  pass_y_outer.setFilterLimits(-3,3);
  PointCloudPtr xy_filtered(new PointCloud());
  pass_y_outer.filter(*xy_filtered);
  pcl::PassThrough<Point> pass_z;
  pass_z.setInputCloud(xy_filtered);
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(0.1, 1);
  PointCloudPtr xyz_filtered(new PointCloud());
  pass_z.filter(*xyz_filtered);
  return xyz_filtered;
}

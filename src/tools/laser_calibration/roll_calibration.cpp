/***************************************************************************
 *  roll_calibration.cpp - Calibrate roll transform of the back laser
 *
 *  Created: Tue 18 Jul 2017 16:28:09 CEST 16:28
 *  Copyright  2017  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

#include "roll_calibration.h"

#include <tf/transformer.h>
#include <tf/transform_listener.h>
#include <config/netconf.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/geometry.h>
#include <pcl/registration/icp.h>

using namespace fawkes;
using namespace std;


RollCalibration::RollCalibration(LaserInterface *laser, tf::Transformer
    *tf_transformer, NetworkConfiguration *config, string config_path)
  : LaserCalibration(laser, tf_transformer, config, config_path) {}

void
RollCalibration::calibrate() {
  printf("Starting to calibrate roll angle.\n");
  float lrd = 2 * threshold;
  uint iterations = 0;
  do {
    try {
      lrd = get_lr_mean_diff();
    } catch (InsufficientDataException &e) {
      printf("Insufficient data: %s\n", e.what_no_backtrace());
      usleep(sleep_time_);
      continue;
    }
    printf("Left-right difference is %f.\n", lrd);
    float old_roll = config_->get_float(config_path_.c_str());
    float new_roll = get_new_roll(lrd, old_roll);
    printf("Updating roll from %f to %f.\n", old_roll, new_roll);
    config_->set_float(config_path_.c_str(), new_roll);
    usleep(sleep_time_);
  } while (abs(lrd) > threshold && ++iterations < max_iterations_);
  printf("Roll calibration finished.\n");
}

float
RollCalibration::get_lr_mean_diff() {
  laser_->read();
  PointCloudPtr cloud = laser_to_pointcloud(*laser_);
  PointCloudPtr calib_cloud = filter_calibration_cloud(cloud);
  transform_pointcloud("base_link", cloud);
  PointCloudPtr rear_cloud = filter_cloud_in_rear(cloud);
  PointCloudPtr left_cloud = filter_left_cloud(rear_cloud);
  PointCloudPtr right_cloud = filter_right_cloud(rear_cloud);
  if (left_cloud->size() < min_points) {
    stringstream error;
    error << "Not enough laser points on the left, got "
          << left_cloud->size() << ", need " << min_points;
    throw InsufficientDataException(error.str().c_str());
  }
  if (right_cloud->size() < min_points) {
    stringstream error;
    error << "Not enough laser points on the right, got "
          << right_cloud->size() << ", need " << min_points;
    throw InsufficientDataException(error.str().c_str());
  }
  printf("Using %zu points on the left, %zu points on the right\n",
      left_cloud->size(), right_cloud->size());
  return get_mean_z(left_cloud) - get_mean_z(right_cloud);
}

float
RollCalibration::get_new_roll(float mean_error, float old_roll) {
  return old_roll + 0.5 * mean_error;
}

PointCloudPtr
RollCalibration::filter_calibration_cloud(PointCloudPtr input) {
  PointCloudPtr filtered(new PointCloud());
  std::vector<int> indices;
  input->is_dense = false;
  pcl::removeNaNFromPointCloud(*input, *filtered, indices);
  return filtered;
}

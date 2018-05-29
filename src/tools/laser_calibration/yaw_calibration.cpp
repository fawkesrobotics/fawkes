/***************************************************************************
 *  yaw_calibration.cpp - Calibrate yaw transform of the back laser
 *
 *  Created: Tue 18 Jul 2017 17:05:30 CEST 17:05
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

#include "yaw_calibration.h"

#include <tf/transformer.h>
#include <config/netconf.h>

using namespace fawkes;
using namespace std;

/** @class YawCalibration "yaw_calibration.h"
 *  Calibrate the yaw angle of the back laser using the front laser.
 *  This is done by comparing data of both lasers left and right of the robot.
 *  The yaw angle of the back laser is adapted so the matching cost between both
 *  lasers is minimzed.
 *  @author Till Hofmann
 */

/** Constructor.
 *  @param laser The interface of the back laser to get the data from
 *  @param front_laser The interface of the front laser to get the data from
 *  @param tf_transformer The transformer to use to compute transforms
 *  @param config The network config to read from and write the time offset to
 *  @param config_path The config path to read from and write the time offset to
 */
YawCalibration::YawCalibration(LaserInterface *laser, LaserInterface *front_laser,
      tf::Transformer *tf_transformer, NetworkConfiguration *config,
      string config_path)
  : LaserCalibration(laser, tf_transformer, config, config_path),
    front_laser_(front_laser), step_(init_step_),
    random_float_dist_(0,1),
    min_cost_(numeric_limits<float>::max()), min_cost_yaw_(0.) {}

/** The actual calibration.
 *  Continuously compare the data from both lasers and update the yaw config
 *  until the cost reaches the threshold.
 */
void
YawCalibration::calibrate() {
  printf("Starting to calibrate yaw angle.\n");
  float current_cost;
  while (true) {
    try {
      current_cost = get_current_cost(NULL);
      break;
    } catch (InsufficientDataException &e) {
      printf("Insufficient data, please move the robot\n");
    }
  }
  uint iterations = 0;
  float last_yaw = config_->get_float(config_path_.c_str());
  min_cost_ = current_cost;
  min_cost_yaw_ = last_yaw;
  while (abs(step_) > 0.0005 && iterations++ < max_iterations_) {
    float next_yaw;
    try {
      current_cost = get_current_cost(&step_);
      next_yaw = last_yaw + step_;
      if (current_cost < min_cost_) {
        min_cost_ = current_cost;
        min_cost_yaw_ = last_yaw;
      }
    } catch (InsufficientDataException &e) {
      printf("Insufficient data, skipping loop.\n");
      continue;
    }
    printf("Updating yaw from %f to %f (step %f), last cost %f\n",
        last_yaw, next_yaw, step_, current_cost);
    config_->set_float(config_path_.c_str(), next_yaw);
    last_yaw = next_yaw;
    usleep(sleep_time_);
  }
  if (current_cost > min_cost_) {
    printf("Setting yaw to %f with minimal cost %f\n",
        min_cost_yaw_, min_cost_);
    config_->set_float(config_path_.c_str(), min_cost_yaw_);
  }
  printf("Yaw calibration finished.\n");
}

/** Get the cost of the current configuration.
 *  @param new_yaw A pointer to the yaw configuration to write updates to
 *  @return The current matching cost
 */
float
YawCalibration::get_current_cost(float *new_yaw) {
  front_laser_->read();
  laser_->read();
  PointCloudPtr front_cloud = laser_to_pointcloud(*front_laser_);
  PointCloudPtr back_cloud = laser_to_pointcloud(*laser_);
  transform_pointcloud("base_link", front_cloud);
  transform_pointcloud("base_link", back_cloud);
  front_cloud = filter_center_cloud(front_cloud);
  back_cloud = filter_center_cloud(back_cloud);
  return get_matching_cost(front_cloud, back_cloud, new_yaw);
}

/** Compute the new yaw.
 *  The yaw is updated by taking steps into one direction until the cost
 *  increases. In that case, the step is size is decreased and negated.
 *  Also randomly reset the step size to avoid local minima.
 *  @param current_cost The current matching cost between both lasers
 *  @param last_yaw The last yaw configuration
 *  @return The new yaw configuration
 */
float
YawCalibration::get_new_yaw(float current_cost, float last_yaw) {
  static float last_cost = current_cost;
  costs_[last_yaw] = current_cost;
  float next_yaw = last_yaw + step_;
  for (auto &cost_pair : costs_) {
    if (cost_pair.second < current_cost && cost_pair.first != last_yaw) {
      float jump_probability =
          static_cast<float>((current_cost - cost_pair.second)) / current_cost;
      float rand_01 = random_float_dist_(random_generator_);
      if (rand_01 > 1 - jump_probability) {
        last_cost = current_cost;
        if (random_float_dist_(random_generator_) >= 0.5) {
          step_ = init_step_;
        } else {
          step_ = -init_step_;
        }
        printf("Jumping to %f, cost %f -> %f (probability was %f)\n",
            cost_pair.first, current_cost, cost_pair.second, jump_probability);
        return cost_pair.first;
      }
    }
  }
  if (current_cost > last_cost) {
    step_ = -step_/2;
  }
  last_cost = current_cost;
  return next_yaw;
}

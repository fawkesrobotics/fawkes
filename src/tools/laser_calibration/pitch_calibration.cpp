/***************************************************************************
 *  pitch_calibration.cpp - Calibrate pitch transform of the back laser
 *
 *  Created: Tue 18 Jul 2017 16:51:36 CEST 16:51
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

#include "pitch_calibration.h"

#include <config/netconf.h>

using namespace fawkes;
using namespace std;

/** @class PitchCalibration "pitch_calibration.h"
 *  Calibrate the pitch angle of the laser.
 *  Compute the angle by computing the mean z value of all points in the rear of
 *  the robot and update the angle accordingly.
 *  @author Till Hofmann
 */

/** Constructor.
 *  @param laser The laser interface to fetch data from
 *  @param tf_transformer The transformer to use to compute transforms
 *  @param config The network config to read from and write updates to
 *  @param config_path The config path to read from and write updates to
 */
PitchCalibration::PitchCalibration(LaserInterface *laser,
    tf::Transformer *tf_transformer,
    NetworkConfiguration *config, string config_path)
: LaserCalibration(laser, tf_transformer, config, config_path) {}

/** The actual calibration.
 * Apply the method continuously until the mean z reaches the threshold.
 * Write the upated pitch angle to the config in each iteration.
 */
void
PitchCalibration::calibrate() {
  printf("Starting pitch angle calibration.\n");
  float mean_z;
  do {
    laser_->read();
    PointCloudPtr cloud = laser_to_pointcloud(*laser_);
    transform_pointcloud("base_link", cloud);
    PointCloudPtr rear_cloud = filter_cloud_in_rear(cloud);
    printf("Rear cloud has %zu points.\n", rear_cloud->points.size());
    try {
      mean_z = get_mean_z(rear_cloud);
    } catch (InsufficientDataException &e) {
      printf("Insufficient data: %s\n", e.what_no_backtrace());
      usleep(sleep_time_);
      continue;
    }
    printf("Mean z is %f.\n", mean_z);
    float old_pitch = config_->get_float(config_path_.c_str());
    float new_pitch = get_new_pitch(mean_z, old_pitch);
    printf("Updating pitch from %f to %f.\n", old_pitch, new_pitch);
    config_->set_float(config_path_.c_str(), new_pitch);
    usleep(sleep_time_);
  } while (abs(mean_z) > threshold);
  printf("Pitch calibration finished.\n");
}

/** Compute the new pitch based on the old pitch and the mean z.
 *  @param z The mean z value of all points in the rear of the robot.
 *  @param old_pitch The pitch that was configured when recording the mean z.
 *  @return The new pitch angle.
 */
float
PitchCalibration::get_new_pitch(float z, float old_pitch) {
  // Note: We could also compute a more accurate new value using the measured
  // distance and height, but this works well enough.
  return old_pitch - z;
}


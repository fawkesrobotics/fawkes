/***************************************************************************
 *  laser_calibration.h - Base class for laser transform calibration
 *
 *  Created: Mon 10 Jul 2017 17:37:01 CEST 17:37
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

#ifndef LASER_CALIBRATION_H
#define LASER_CALIBRATION_H

#include <core/exception.h>
#include <interfaces/Laser360Interface.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <cmath>

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;

typedef fawkes::Laser360Interface LaserInterface;

namespace fawkes {
  class NetworkConfiguration;
  class MotorInterface;
  namespace tf {
    class Transformer;
  }
}

inline float
deg2rad(float deg)
{
  return (deg * M_PI / 180.f);
}

/** Exception that is thrown if there are not enough laser points to
 *  do a matching.
 */
class InsufficientDataException : public fawkes::Exception
{
public:
  /** Constructor.
   *  @param error: the error message
   */
  InsufficientDataException(const char *error) : Exception(error) {}
};

class LaserCalibration
{
public:
  LaserCalibration(LaserInterface *laser,
      fawkes::tf::Transformer *tf_transformer,
      fawkes::NetworkConfiguration *config,
      std::string config_path);
  virtual ~LaserCalibration();
  virtual void calibrate() = 0;

protected:
  PointCloudPtr laser_to_pointcloud(const LaserInterface &laser);
  void transform_pointcloud(const std::string &target_frame,
      PointCloudPtr cloud);
  PointCloudPtr filter_cloud_in_rear(PointCloudPtr input);
  float get_mean_z(PointCloudPtr cloud);
  PointCloudPtr filter_left_cloud(PointCloudPtr input);
  PointCloudPtr filter_right_cloud(PointCloudPtr input);
  PointCloudPtr filter_out_ground(PointCloudPtr input);
  float get_matching_cost(
      PointCloudPtr cloud1, PointCloudPtr cloud2, float *rot_yaw);
  PointCloudPtr filter_center_cloud(PointCloudPtr input);

protected:
  /** The laser that provides the input data */
  LaserInterface *laser_;
  /** The transformer used to compute transforms */
  fawkes::tf::Transformer *tf_transformer_;
  /** The network config to use for reading and updating config values */
  fawkes::NetworkConfiguration *config_;
  /** The config path to use for reading and updating config values */
  const std::string config_path_;
  /** Time in micro seconds to sleep between iterations */
  const static long sleep_time_ = 50000;
  /** The number of iterations to run before aborting the calibration */
  const static uint max_iterations_ = 100;
  /** The number of points required in a pointcloud to use it as input data */
  const static size_t min_points = 10;
};

#endif /* !LASER_CALIBRATION_H */

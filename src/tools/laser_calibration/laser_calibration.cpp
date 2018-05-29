/***************************************************************************
 *  laser_calibration.cpp - Tool to calibrate laser transforms
 *
 *  Created: Mon 10 Jul 2017 17:37:21 CEST 17:37
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

#include "laser_calibration.h"

#include <config/netconf.h>
#include <tf/transformer.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/geometry.h>
#include <pcl/registration/icp.h>

#include <cmath>

using namespace fawkes;
using namespace std;

/** @class LaserCalibration "laser_calibration.h"
 *  Abstract base class for laser calibration. The class provides functions that
 *  are common for all calibration methods.
 *  @author Till Hofmann
 */

/** @fn LaserCalibration::calibrate
 *  The actual calibration procedure.
 *  Virtual function that is called once to calibrate the laser.
 */

/** Constructor.
 *  @param laser The laser interface to fetch data from
 *  @param tf_transformer The transformer to use to compute transforms
 *  @param config The network config to read from and write updates to
 *  @param config_path The config path to read from and write updates to
 */
LaserCalibration::LaserCalibration(LaserInterface *laser, tf::Transformer
    *tf_transformer, NetworkConfiguration *config, string config_path)
: laser_(laser), tf_transformer_(tf_transformer), config_(config),
  config_path_(config_path) {}

/** Destructor. */
LaserCalibration::~LaserCalibration() {}

/** Convert the laser data into a pointcloud.
 *  The frame of the pointcloud is set to the frame of the laser, no transform
 *  is applied.
 *  @param laser The laser interface to read the data from
 *  @return A pointer to a pointcloud that contains the laser data
 */
PointCloudPtr
LaserCalibration::laser_to_pointcloud(const LaserInterface &laser) {
    PointCloudPtr cloud = PointCloudPtr(new PointCloud());
    cloud->points.resize(laser.maxlenof_distances());
    cloud->header.frame_id = laser.frame();
    cloud->height = 1;
    cloud->width = laser.maxlenof_distances();
    float const *distances = laser.distances();
    for (uint i = 0; i < laser.maxlenof_distances(); i++) {
      cloud->points[i].x = distances[i]
          * cosf(deg2rad(i) * (360. / laser.maxlenof_distances()));
      cloud->points[i].y = distances[i]
          * sinf(deg2rad(i) * (360. / laser.maxlenof_distances()));
    }
    return cloud;
  }

/** Transform the points in a pointcloud.
 *  The pointcloud is transformed in-place, i.e., the referenced input
 *  pointcloud is updated to be in the target frame.
 *  @param target_frame The frame all points should be transformed into
 *  @param cloud A reference to the pointcloud to transform
 */
void
LaserCalibration::transform_pointcloud(
    const string &target_frame, PointCloudPtr cloud) {
  for (auto &point : cloud->points) {
   tf::Stamped<tf::Point> point_in_laser_frame(
       tf::Point(point.x, point.y, point.z),
       fawkes::Time(0., cloud->header.stamp), cloud->header.frame_id);
   tf::Stamped<tf::Point> point_in_base_frame;
   tf_transformer_->transform_point(
       target_frame, point_in_laser_frame, point_in_base_frame);
   point.x = static_cast<float>(point_in_base_frame[0]);
   point.y = static_cast<float>(point_in_base_frame[1]);
   point.z = static_cast<float>(point_in_base_frame[2]);
  }
}

/** Remove points in the rear of the robot.
 *  @param input The pointcloud to remove the points from.
 *  @return The same pointcloud but without any points in the rear.
 */
PointCloudPtr
LaserCalibration::filter_cloud_in_rear(PointCloudPtr input) {
  pcl::PassThrough<Point> pass;
  pass.setInputCloud(input);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(-2., -0.8);
  PointCloudPtr output(new PointCloud());
  pass.filter(*output);
  return output;
}

/** Compute the mean z value of all points in the given pointcloud.
 *  This can be used to compute the height of a line, e.g., a line that should
 *  be on the ground. The value can be used to tweak the roll or pitch of the
 *  lasers.
 *  @param cloud The cloud that is used to compute the mean z
 *  @return The mean z of all points in the input cloud
 */
float
LaserCalibration::get_mean_z(PointCloudPtr cloud) {
  if (cloud->points.size() < min_points) {
    stringstream error;
    error << "Not enough laser points in rear cloud, got "
          << cloud->size() << ", need " << min_points;
    throw InsufficientDataException(error.str().c_str());
  }
  vector<float> zs;
  zs.resize(cloud->points.size());
  for (auto &point : *cloud) {
    zs.push_back(point.z);
  }
  return accumulate(zs.begin(), zs.end(), 0.) / zs.size();
}

/** Remove all points that are left of the robot.
 *  @param input The cloud to remove the points from
 *  @return The same cloud as the input but without any points left of the robot
 */
PointCloudPtr
LaserCalibration::filter_left_cloud(PointCloudPtr input) {
  pcl::PassThrough<Point> pass;
  pass.setInputCloud(input);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(0., 2.);
  PointCloudPtr output(new PointCloud());
  pass.filter(*output);
  return output;
}

/** Remove all points that are right of the robot.
 *  @param input The cloud to remove the points from
 *  @return The same cloud as the input but without any points right of the robot
 */
PointCloudPtr
LaserCalibration::filter_right_cloud(PointCloudPtr input) {
  pcl::PassThrough<Point> pass;
  pass.setInputCloud(input);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-2., 0.);
  PointCloudPtr output(new PointCloud());
  pass.filter(*output);
  return output;
}

/** Remove all points that belong to the ground.
 *  Points that have a height < 0.1 are assumed to be part of the ground.
 *  @param input The pointcloud to remove the points form
 *  @return The same cloud as the input but without any points on the ground
 */
PointCloudPtr
LaserCalibration::filter_out_ground(PointCloudPtr input) {
  pcl::PassThrough<Point> pass;
  pass.setInputCloud(input);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.1, 1);
  PointCloudPtr output(new PointCloud());
  pass.filter(*output);
  return output;
}

/** Compare two pointclouds with ICP.
 *  Compute the best angle to rotate cloud2 into cloud1 with ICP and get the
 *  cost.
 *  @param cloud1 The first input cloud, used as target cloud in ICP
 *  @param cloud2 The second input cloud, this is used as ICP input
 *  @param rot_yaw A pointer to a float to write the resulting rotation to
 *  @return The ICP fitness score as matching cost
 */
float
LaserCalibration::get_matching_cost(
    PointCloudPtr cloud1, PointCloudPtr cloud2, float *rot_yaw) {
  if (cloud1->points.size() < min_points || cloud2->points.size() < min_points) {
    stringstream error;
    error << "Not enough points, got " << cloud1->points.size() << " and "
        << cloud2->points.size() << " points, need " << min_points;
    throw InsufficientDataException(error.str().c_str());
  }
  pcl::IterativeClosestPoint<Point, Point> icp;
  icp.setInputCloud(cloud2);
  icp.setInputTarget(cloud1);
  PointCloud final;
  icp.align(final);
  if (!icp.hasConverged()) {
    throw InsufficientDataException("ICP did not converge.");
  }
  if (rot_yaw) {
    pcl::Registration<Point, Point, float>::Matrix4 transformation =
        icp.getFinalTransformation();
    *rot_yaw = atan2(transformation(1,0), transformation(0,0));
  }
  return icp.getFitnessScore();
}

/** Remove the center of a pointcloud
 *  This removes all points around the origin of the pointcloud. Use this to
 *  remove the robot from the data.
 *  @param input The pointcloud to filter
 *  @return The same cloud as the input but without any points around the center
 */
PointCloudPtr
LaserCalibration::filter_center_cloud(PointCloudPtr input) {
  pcl::PassThrough<Point> pass_x;
  pass_x.setInputCloud(input);
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(-2, 2);
  PointCloudPtr x_filtered(new PointCloud());
  pass_x.filter(*x_filtered);
  pcl::PassThrough<Point> pass_y;;
  pass_y.setInputCloud(x_filtered);
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimitsNegative(true);
  pass_y.setFilterLimits(-0.5,0.5);
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

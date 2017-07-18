/***************************************************************************
 *  laser_calibration.cpp - Tool to calibrate laser transforms
 *
 *  Created: Mon 10 Jul 2017 17:37:21 CEST 17:37
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

LaserCalibration::LaserCalibration(LaserInterface *laser, tf::Transformer
    *tf_transformer, NetworkConfiguration *config, string config_path)
: laser_(laser), tf_transformer_(tf_transformer), config_(config),
  config_path_(config_path) {}
LaserCalibration::~LaserCalibration() {}

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

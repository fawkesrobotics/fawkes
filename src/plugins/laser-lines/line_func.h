
/***************************************************************************
 *  line_func.h - function to detect lines in laser data
 *
 *  Created: Tue Mar 17 11:13:24 2015 (re-factoring)
 *  Copyright  2011-2015  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_LASER_LINES_LINE_FUNC_H_
#define __PLUGINS_LASER_LINES_LINE_FUNC_H_

#include "line_info.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>

/** Calculate length of line from associated points.
 * The unit depends on the units of the input data.
 * @param cloud_line point cloud with points from which the line model was
 * determined.
 * @param coeff line model coefficients
 * @param end_point_1 upon return contains one of the end points of the line segment
 * @param end_point_2 upon return contains the second end point of the line segment
 * @return length of line
 */
template <class PointType>
float
calc_line_length(typename pcl::PointCloud<PointType>::Ptr cloud_line,
		 pcl::ModelCoefficients::Ptr coeff,
		 Eigen::Vector3f &end_point_1, Eigen::Vector3f &end_point_2)
{
  if (cloud_line->points.size() < 2)  return 0.;

  // Project the model inliers
  typename pcl::PointCloud<PointType>::Ptr cloud_line_proj(new pcl::PointCloud<PointType>());
  pcl::ProjectInliers<PointType> proj;
  proj.setModelType(pcl::SACMODEL_LINE);
  proj.setInputCloud(cloud_line);
  proj.setModelCoefficients(coeff);
  proj.filter(*cloud_line_proj);

  Eigen::Vector3f point_on_line, line_dir;
  point_on_line[0]  = cloud_line_proj->points[0].x;
  point_on_line[1]  = cloud_line_proj->points[0].y;
  point_on_line[2]  = cloud_line_proj->points[0].z;
  line_dir[0]       = coeff->values[3];
  line_dir[1]       = coeff->values[4];
  line_dir[2]       = coeff->values[5];
  line_dir.normalize();

  ssize_t idx_1 = 0, idx_2 = 0;
  float max_dist_1 = 0.f, max_dist_2 = 0.f;

  for (size_t i = 1; i < cloud_line_proj->points.size(); ++i) {
    const PointType &pt = cloud_line_proj->points[i];
    Eigen::Vector3f ptv(pt.x, pt.y, pt.z);
    Eigen::Vector3f diff(ptv - point_on_line);
    float dist = diff.norm();
    float dir  = line_dir.dot(diff);
    if (dir >= 0) {
      if (dist > max_dist_1) {
	max_dist_1 = dist;
	idx_1 = i;
      }
    }
    if (dir <= 0) {
      if (dist > max_dist_2) {
	max_dist_2 = dist;
	idx_2 = i;
      }
    }
  }

  if (idx_1 >= 0 && idx_2 >= 0) {
    const PointType &pt_1 = cloud_line_proj->points[idx_1];
    const PointType &pt_2 = cloud_line_proj->points[idx_2];

    Eigen::Vector3f ptv_1(pt_1.x, pt_1.y, pt_1.z);
    Eigen::Vector3f ptv_2(pt_2.x, pt_2.y, pt_2.z);

    end_point_1 = ptv_1;
    end_point_2 = ptv_2;

    return (ptv_1 - ptv_2).norm();
  } else {
    return 0.f;
  }
}


/** Calculate a number of lines from a given point cloud.
 * @param input input point clouds from which to extract lines
 * @param segm_min_inliers minimum total number of required inliers to consider a line
 * @param segm_max_iterations maximum number of line RANSAC iterations
 * @param segm_distance_threshold maximum distance of point to line to account it to a line
 * @param segm_sample_max_dist max inter-sample distance for line RANSAC
 * @param min_length minimum length of line to consider it
 * @param max_length maximum length of a line to consider it
 * @param min_dist minimum distance from frame origin to closest point on line to consider it
 * @param max_dist maximum distance from frame origin to closest point on line to consider it
 * @param remaining_cloud if passed with a valid cloud will be assigned the remaining
 * points, that is points which have not been accounted to a line, upon return
 * @return vector of info about detected lines
 */
template <class PointType>
std::vector<LineInfo>
calc_lines(typename pcl::PointCloud<PointType>::ConstPtr input,
	   unsigned int segm_min_inliers, unsigned int segm_max_iterations,
	   float segm_distance_threshold, float segm_sample_max_dist,
	   float cluster_tolerance, float cluster_quota,
	   float min_length, float max_length, float min_dist, float max_dist,
	   typename pcl::PointCloud<PointType>::Ptr remaining_cloud = typename pcl::PointCloud<PointType>::Ptr())
{
  typename pcl::PointCloud<PointType>::Ptr in_cloud(new pcl::PointCloud<PointType>());

  {
    // Erase non-finite points
    pcl::PassThrough<PointType> passthrough;
    passthrough.setInputCloud(input);
    passthrough.filter(*in_cloud);
  }

  pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

  std::vector<LineInfo> linfos;

  while (in_cloud->points.size () > segm_min_inliers) {
    // Segment the largest linear component from the remaining cloud
    //logger->log_info(name(), "[L %u] %zu points left",
    //		     loop_count_, in_cloud->points.size());
      
    typename pcl::search::KdTree<PointType>::Ptr
      search(new pcl::search::KdTree<PointType>);
    search->setInputCloud(in_cloud);

    pcl::SACSegmentation<PointType> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(segm_max_iterations);
    seg.setDistanceThreshold(segm_distance_threshold);
    seg.setSamplesMaxDist(segm_sample_max_dist, search); 
    seg.setInputCloud(in_cloud);
    seg.segment(*inliers, *coeff);
    if (inliers->indices.size () == 0) {
      // no line found
      break;
    }

    // check for a minimum number of expected inliers
    if ((double)inliers->indices.size() < segm_min_inliers) {
      //logger->log_warn(name(), "[L %u] no more lines (%zu inliers, required %u)",
      //	       loop_count_, inliers->indices.size(), segm_min_inliers);
      break;
    }

    //logger->log_info(name(), "[L %u] Found line with %zu inliers",
    //		     loop_count_, inliers->indices.size());

    // Cluster within the line to make sure it is a contiguous line
    // the line search can output a line which combines lines at separate
    // ends of the field of view...

    typename pcl::search::KdTree<PointType>::Ptr
      kdtree_line_cluster(new pcl::search::KdTree<PointType>());
    typename pcl::search::KdTree<PointType>::IndicesConstPtr
      search_indices(new std::vector<int>(inliers->indices));
    kdtree_line_cluster->setInputCloud(in_cloud, search_indices);

    std::vector<pcl::PointIndices> line_cluster_indices;
    pcl::EuclideanClusterExtraction<PointType> line_ec;
    line_ec.setClusterTolerance(cluster_tolerance);
    size_t min_size = (size_t)floorf(cluster_quota * inliers->indices.size());
    line_ec.setMinClusterSize(min_size);
    line_ec.setMaxClusterSize(inliers->indices.size());
    line_ec.setSearchMethod(kdtree_line_cluster);
    line_ec.setInputCloud(in_cloud);
    line_ec.setIndices(inliers);
    line_ec.extract(line_cluster_indices);

    pcl::PointIndices::Ptr line_cluster_index;
    if (! line_cluster_indices.empty()) {
      line_cluster_index = pcl::PointIndices::Ptr(new pcl::PointIndices(line_cluster_indices[0]));
    }

    // re-calculate coefficients based on line cluster only
    if (line_cluster_index) {
      pcl::SACSegmentation<PointType> segc;
      segc.setOptimizeCoefficients(true);
      segc.setModelType(pcl::SACMODEL_LINE);
      segc.setMethodType(pcl::SAC_RANSAC);
      segc.setMaxIterations(segm_max_iterations);
      segc.setDistanceThreshold(segm_distance_threshold);
      segc.setInputCloud(in_cloud);
      segc.setIndices(line_cluster_index);
      pcl::PointIndices::Ptr tmp_index(new pcl::PointIndices());
      segc.segment(*tmp_index, *coeff);
      *line_cluster_index = *tmp_index;
    }

    // Remove the linear or clustered inliers, extract the rest
    typename pcl::PointCloud<PointType>::Ptr cloud_f(new pcl::PointCloud<PointType>());
    typename pcl::PointCloud<PointType>::Ptr cloud_line(new pcl::PointCloud<PointType>());
    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud(in_cloud);
    extract.setIndices((line_cluster_index && ! line_cluster_index->indices.empty())
		       ? line_cluster_index : inliers);
    extract.setNegative(false);
    extract.filter(*cloud_line);

    extract.setNegative(true);
    extract.filter(*cloud_f);
    *in_cloud = *cloud_f;

    if (!line_cluster_index || line_cluster_index->indices.empty())  continue;

    // Check if this line has the requested minimum length
    Eigen::Vector3f end_point_1, end_point_2;
    float length = calc_line_length<PointType>(cloud_line, coeff, end_point_1, end_point_2);

    if (length == 0 ||
	(min_length >= 0 && length < min_length) ||
	(max_length >= 0 && length > max_length))
    {
      continue;
    }

    LineInfo info;
    info.cloud.reset(new pcl::PointCloud<PointType>());

    info.point_on_line[0]  = coeff->values[0];
    info.point_on_line[1]  = coeff->values[1];
    info.point_on_line[2]  = coeff->values[2];
    info.line_direction[0] = coeff->values[3];
    info.line_direction[1] = coeff->values[4];
    info.line_direction[2] = coeff->values[5];

    info.length = length;

    Eigen::Vector3f ld_unit = info.line_direction / info.line_direction.norm();
    Eigen::Vector3f pol_invert = Eigen::Vector3f(0,0,0)-info.point_on_line;
    Eigen::Vector3f P = info.point_on_line + pol_invert.dot(ld_unit) * ld_unit;
    Eigen::Vector3f x_axis(1,0,0);
    info.bearing = acosf(x_axis.dot(P) / P.norm());
    // we also want to encode the direction of the angle
    if (P[1] < 0)  info.bearing = fabs(info.bearing)*-1.;

    info.base_point = P;
    float dist = info.base_point.norm();

    if ((min_dist >= 0. && dist < min_dist) ||
	(max_dist >= 0. && dist > max_dist))
    {
      //logger->log_warn(name(), "[L %u] line too close or too far (%f, min %f, max %f)",
      //	       loop_count_, dist, min_dist, max_dist);
      continue;
    }

    // Calculate parameter for e2 with respect to e1 as origin and the
    // direction vector, i.e. a k such that e1 + k * dir == e2.
    // If the resulting parameter is >= 0, then the direction vector
    // points from e1 to e2, otherwise it points from e2 to e1.
    float line_dir_k = ld_unit.dot(end_point_2 - end_point_1);

    if (line_dir_k >= 0) {
      info.end_point_1 = end_point_1;
      info.end_point_2 = end_point_2;
    } else {
      info.end_point_1 = end_point_2;
      info.end_point_2 = end_point_1;
    }

    coeff->values[0] = P[0];
    coeff->values[1] = P[1];
    coeff->values[2] = P[2];

    // Project the model inliers
    pcl::ProjectInliers<PointType> proj;
    proj.setModelType(pcl::SACMODEL_LINE);
    proj.setInputCloud(cloud_line);
    proj.setModelCoefficients(coeff);
    proj.filter(*info.cloud);

    linfos.push_back(info);
  }

  if (remaining_cloud) {
    *remaining_cloud = *in_cloud;
  }

  return linfos;
}


#endif

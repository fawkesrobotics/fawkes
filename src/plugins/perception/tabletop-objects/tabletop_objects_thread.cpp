
/***************************************************************************
 *  tabletop_objects_thread.cpp - Thread to detect tabletop objects
 *
 *  Created: Sat Nov 05 00:22:41 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
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

#include "tabletop_objects_thread.h"
#include "pcl_utils.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/conditional_removal.h>

#define TABLE_MAX_X  3.0
#define TABLE_MAX_Y  3.0
#define TABLE_MIN_X -3.0
#define TABLE_MIN_Y -3.0


/** @class TabletopObjectsThread "tabletop_objects_thread.h"
 * Main thread of tabletop objects plugin.
 * @author Tim Niemueller
 */

using namespace fawkes;

/** Constructor. */
TabletopObjectsThread::TabletopObjectsThread()
  : Thread("TabletopObjectsThread", Thread::OPMODE_CONTINUOUS)
{
}


/** Destructor. */
TabletopObjectsThread::~TabletopObjectsThread()
{
}


void
TabletopObjectsThread::init()
{
  finput_ = pcl_manager->get_pointcloud<PointType>("openni-pointcloud");
  input_.reset(*finput_);

  fclusters_ = new pcl::PointCloud<ColorPointType>();
  fclusters_->header.frame_id = finput_->header.frame_id;
  fclusters_->is_dense = false;
  pcl_manager->add_pointcloud<ColorPointType>("tabletop-object-clusters", fclusters_);
  clusters_.reset(*fclusters_);

  grid_.setFilterFieldName("x");
  grid_.setFilterLimits(0.0, 3.0);
  grid_.setLeafSize(0.01, 0.01, 0.01);

  seg_.setOptimizeCoefficients(true);
  seg_.setModelType(pcl::SACMODEL_PLANE);
  seg_.setMethodType(pcl::SAC_RANSAC);
  seg_.setMaxIterations(1000);
  seg_.setDistanceThreshold(0.01);
}


void
TabletopObjectsThread::finalize()
{
  pcl_manager->remove_pointcloud("tabletop-object-clusters");
}


void
TabletopObjectsThread::loop()
{
  CloudPtr temp_cloud (new Cloud);
  CloudPtr temp_cloud2 (new Cloud);
  pcl::ExtractIndices<PointType> extract_;
  std::vector<pcl::Vertices> vertices_;
  CloudPtr cloud_hull_;
  CloudPtr cloud_proj_;
  CloudPtr cloud_filt_;
  CloudPtr cloud_above_;
  CloudPtr cloud_objs_;
  pcl::KdTreeFLANN<PointType> kdtree_;

  grid_.setInputCloud (input_);
  grid_.filter (*temp_cloud);

  // set all colors to white for better distinguishing the pixels
  /*
  typename pcl::PointCloud<PointType>::iterator p;
  for (p = temp_cloud->begin(); p != temp_cloud->end(); ++p) {
    p->r = 255;
    p->g = 255;
    p->b = 255;
  }
  */

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

  seg_.setInputCloud (temp_cloud);
  seg_.segment (*inliers, *coefficients);

  extract_.setNegative (false);
  extract_.setInputCloud (temp_cloud);
  extract_.setIndices (inliers);
  extract_.filter (*temp_cloud2);

  // Project the model inliers
  pcl::ProjectInliers<PointType> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(temp_cloud2);
  proj.setModelCoefficients(coefficients);
  cloud_proj_.reset(new Cloud());
  proj.filter (*cloud_proj_);
  //printf("PointCloud after projection has: %zu data points.\n",
  //       cloud_proj_->points.size());


  // Estimate 3D convex hull -> TABLE BOUNDARIES
  pcl::ConvexHull<PointType> hr;
  //hr.setAlpha (0.1);  // only for ConcaveHull
  hr.setInputCloud(cloud_proj_);
  cloud_hull_.reset(new Cloud());
  hr.reconstruct (*cloud_hull_, vertices_);

  //printf("Found %zu vertices, first has size %zu\n",
  //       vertices_.size(), vertices_[0].vertices.size());

  /*
  for (size_t i = 0; i < cloud_proj_->points.size(); ++i) {
    cloud_proj_->points[i].r =   0;
    cloud_proj_->points[i].g = 255;
    cloud_proj_->points[i].b =   0;
  }
  */

  // Extract all non-plane points
  cloud_filt_.reset(new Cloud());
  extract_.setNegative(true);
  extract_.filter(*cloud_filt_);

  // remove all pixels below table
  typename PlaneDistanceComparison<PointType>::ConstPtr
    above_comp(new PlaneDistanceComparison<PointType>(coefficients,
                                                      pcl::ComparisonOps::LT));
  typename pcl::ConditionAnd<PointType>::Ptr
    above_cond(new pcl::ConditionAnd<PointType>());
  above_cond->addComparison(above_comp);
  pcl::ConditionalRemoval<PointType> above_condrem(above_cond);
  above_condrem.setInputCloud(cloud_filt_);
  //above_condrem.setKeepOrganized(true);
  cloud_above_.reset(new Cloud());
  above_condrem.filter(*cloud_above_);

  printf("Before: %zu  After: %zu\n", cloud_filt_->points.size(),
         cloud_above_->points.size());

  // Extract only points on the table plane
  if (! vertices_.empty()) {
    pcl::PointIndices::Ptr polygon(new pcl::PointIndices());
    polygon->indices = vertices_[0].vertices;

    pcl::PointCloud<PointType> polygon_cloud;
    pcl::ExtractIndices<PointType> polygon_extract;

    polygon_extract.setInputCloud(cloud_hull_);
    polygon_extract.setIndices(polygon);
    polygon_extract.filter(polygon_cloud);

    typename pcl::ConditionAnd<PointType>::Ptr
      polygon_cond(new pcl::ConditionAnd<PointType>());

    typename PolygonComparison<PointType>::ConstPtr
      inpoly_comp(new PolygonComparison<PointType>(polygon_cloud));
    polygon_cond->addComparison(inpoly_comp);

    // build the filter
    pcl::ConditionalRemoval<PointType> condrem(polygon_cond);
    condrem.setInputCloud(cloud_above_);
    //condrem.setKeepOrganized(true);
    cloud_objs_.reset(new Cloud());
    condrem.filter(*cloud_objs_);
  } else {
    cloud_objs_.reset(new Cloud(*cloud_above_));
  }

  // CLUSTERS
  // extract clusters of OBJECTS

  ColorCloudPtr tmp_clusters(new ColorCloud());
  tmp_clusters->header.frame_id = clusters_->header.frame_id;
  std::vector<int> &indices = inliers->indices;
  tmp_clusters->height = 1;
  tmp_clusters->width = indices.size ();
  tmp_clusters->points.resize(indices.size());
  for (size_t i = 0; i < indices.size(); ++i) {
    PointType &p1 = temp_cloud2->points[i];
    ColorPointType &p2 = tmp_clusters->points[i];
    p2.x = p1.x;
    p2.y = p1.y;
    p2.z = p1.z;
    p2.r = p2.b = 0;
    p2.g = 255;
  }

  if (cloud_objs_->points.size() > 0) {
    // Creating the KdTree object for the search method of the extraction
    pcl::KdTree<PointType>::Ptr
      kdtree_cl(new pcl::KdTreeFLANN<PointType>());
    kdtree_cl->setInputCloud(cloud_objs_);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance(0.02); // 2cm
    ec.setMinClusterSize(50);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(kdtree_cl);
    ec.setInputCloud(cloud_objs_);
    ec.extract(cluster_indices);

    printf("Found %zu clusters\n", cluster_indices.size());

    uint8_t colors[5][3] = { {255, 0, 0}, {0, 0, 255}, {255, 255, 0}, {255, 0, 255},
                             {0, 255, 255} };

    ColorCloudPtr colored_clusters(new ColorCloud());
    colored_clusters->header.frame_id = clusters_->header.frame_id;
    std::vector<pcl::PointIndices>::const_iterator it;
    unsigned int color = 0;
    unsigned int i = 0;
    unsigned int num_points;
    for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
      num_points += it->indices.size();

    colored_clusters->points.resize(num_points);
    unsigned int cci = 0;

    for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
      uint8_t r, g, b;
      if (color < 5) {
        r = colors[color][0];
        g = colors[color][1];
        b = colors[color][2];
        ++color;
      } else {
        double dr=0, dg=0, db=0;
        //pcl::visualization::getRandomColors(dr, dg, db);
        r = (uint8_t)roundf(dr * 255);
        g = (uint8_t)roundf(dg * 255);
        b = (uint8_t)roundf(db * 255);
      }
      printf("Cluster %u  size: %zu  color %u, %u, %u\n",
             ++i, it->indices.size(), r, g, b);
      std::vector<int>::const_iterator pit;
      for (pit = it->indices.begin (); pit != it->indices.end(); pit++) {
        ColorPointType &p1 = colored_clusters->points[cci++];
        PointType &p2 = cloud_objs_->points[*pit];
        p1.x = p2.x;
        p1.y = p2.y;
        p1.z = p2.z;
        p1.r = r;
        p1.g = g;
        p1.b = b;
      }
    }

    *tmp_clusters += *colored_clusters;
  } else {
    logger->log_info(name(), "Filter left no points for clustering");
  }

  *clusters_ = *tmp_clusters;
  pcl_copy_time(finput_, fclusters_);

  // To show differences between cloud_filt and cloud_above
  // (draw both, increase point size of cloud_above
  //for (int i = 0; i < cloud_filt_->points.size(); ++i) {
  //  cloud_filt_->points[i].r = 255;
  //  cloud_filt_->points[i].g =   0;
  //  cloud_filt_->points[i].b =   0;
  //}

  // give rviz time to catch up...
  //usleep(500000);
}


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
#include <pcl/common/centroid.h>

#include <interfaces/Position3DInterface.h>

#define MAX_CENTROIDS 12

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

  try {
    __pos_ifs.clear();
    __pos_ifs.resize(MAX_CENTROIDS, NULL);
    for (unsigned int i = 0; i < MAX_CENTROIDS; ++i) {
      char *tmp;
      if (asprintf(&tmp, "Tabletop Object %u", i) != -1) {
        // Copy to get memory freed on exception
        std::string id = tmp;
        free(tmp);
        Position3DInterface *iface =
          blackboard->open_for_writing<Position3DInterface>(id.c_str());
        __pos_ifs[i] = iface;
        double rotation[4] = {0., 0., 0., 1.};
        iface->set_rotation(rotation);
        iface->write();
      }
    }
  } catch (Exception &e) {
    for (unsigned int i = 0; i < MAX_CENTROIDS; ++i) {
      if (__pos_ifs[i]) {
        blackboard->close(__pos_ifs[i]);
      }
    }
    __pos_ifs.clear();
    throw;
  }

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
  input_.reset();
  clusters_.reset();

  pcl_manager->remove_pointcloud("tabletop-object-clusters");
  
  for (unsigned int i = 0; i < MAX_CENTROIDS; ++i) {
    blackboard->close(__pos_ifs[i]);
  }
  __pos_ifs.clear();
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


  // Estimate 3D convex hull -> TABLE BOUNDARIES
  pcl::ConvexHull<PointType> hr;
  //hr.setAlpha (0.1);  // only for ConcaveHull
  hr.setInputCloud(cloud_proj_);
  cloud_hull_.reset(new Cloud());
  hr.reconstruct (*cloud_hull_, vertices_);

  // Extract all non-plane points
  cloud_filt_.reset(new Cloud());
  extract_.setNegative(true);
  extract_.filter(*cloud_filt_);

  // Use only points above tables
  // Why coefficients->values[3] > 0 ? ComparisonOps::GT : ComparisonOps::LT?
  // The model coefficients are in Hessian Normal Form, hence coeff[0..2] are
  // the normal vector. We need to distinguish the cases where the normal vector
  // points towards the origin (camera) or away from it. This can be checked
  // by calculating the distance towards the origin, which conveniently in
  // dist = N * x + p is just p which is coeff[3]. Therefore, if coeff[3] is
  // positive, the normal vector points towards the camera and we want all
  // points with positive distance from the table plane, otherwise it points
  // away from the origin and we want points with "negative distance".
  // We make use of the fact that we only have a boring RGB-D camera and
  // not an X-Ray...
  pcl::ComparisonOps::CompareOp op =
    coefficients->values[3] > 0 ? pcl::ComparisonOps::GT : pcl::ComparisonOps::LT;
  typename PlaneDistanceComparison<PointType>::ConstPtr
    above_comp(new PlaneDistanceComparison<PointType>(coefficients, op));
  typename pcl::ConditionAnd<PointType>::Ptr
    above_cond(new pcl::ConditionAnd<PointType>());
  above_cond->addComparison(above_comp);
  pcl::ConditionalRemoval<PointType> above_condrem(above_cond);
  above_condrem.setInputCloud(cloud_filt_);
  //above_condrem.setKeepOrganized(true);
  cloud_above_.reset(new Cloud());
  above_condrem.filter(*cloud_above_);

  //printf("Before: %zu  After: %zu\n", cloud_filt_->points.size(),
  //       cloud_above_->points.size());
  if (cloud_filt_->points.size() < 50) {
    logger->log_warn(name(), "Less points than cluster min size");
  }

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

  std::vector<Eigen::Vector4f> centroids;
  centroids.resize(MAX_CENTROIDS);
  unsigned int centroid_i = 0;

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

    //logger->log_debug(name(), "Found %zu clusters", cluster_indices.size());

    uint8_t colors[MAX_CENTROIDS][3] = {{255, 0, 0}, {0, 0, 255}, {255, 255, 0}, {255, 0, 255},
                                        {0, 255, 255}, {255, 90, 0}, {176, 0, 30}, {0, 106, 53},
                                        {137, 82, 39}, {27, 117, 196}, {99, 0, 30}, {56, 23, 90}};

    ColorCloudPtr colored_clusters(new ColorCloud());
    colored_clusters->header.frame_id = clusters_->header.frame_id;
    std::vector<pcl::PointIndices>::const_iterator it;
    unsigned int cci = 0;
    //unsigned int i = 0;
    unsigned int num_points = 0;
    for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
      num_points += it->indices.size();

    if (num_points > 0) {
      colored_clusters->points.resize(num_points);
      for (it = cluster_indices.begin();
           it != cluster_indices.end() && centroid_i < MAX_CENTROIDS;
           ++it, ++centroid_i)
      {
        pcl::compute3DCentroid(*cloud_objs_, it->indices, centroids[centroid_i]);

        uint8_t r, g, b;
        r = colors[centroid_i][0];
        g = colors[centroid_i][1];
        b = colors[centroid_i][2];

        //printf("Cluster %u  size: %zu  color %u, %u, %u\n",
        //       centroid_i, it->indices.size(), r, g, b);
        std::vector<int>::const_iterator pit;
        for (pit = it->indices.begin (); pit != it->indices.end(); ++pit) {
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
      logger->log_info(name(), "No clustered points found");
    }
  } else {
    logger->log_info(name(), "Filter left no points for clustering");
  }

  for (unsigned int i = 0; i < MAX_CENTROIDS; ++i) {
    int visibility_history = __pos_ifs[i]->visibility_history();
    if (i < centroid_i) { // valid
      Eigen::Vector4f &c = centroids[i];
      if (visibility_history >= 0) {
        __pos_ifs[i]->set_visibility_history(visibility_history + 1);
      } else {
        __pos_ifs[i]->set_visibility_history(1);
      }
      double translation[3] = { c[0], c[1], c[2] };
      __pos_ifs[i]->set_translation(translation);
      __pos_ifs[i]->write();
    } else {
      // invalid
      if (visibility_history <= 0) {
        __pos_ifs[i]->set_visibility_history(visibility_history - 1);
      } else {
        __pos_ifs[i]->set_visibility_history(-1);
        double translation[3] = { 0, 0, 0 };
        __pos_ifs[i]->set_translation(translation);
      }
      __pos_ifs[i]->write();
    }
  }

  *clusters_ = *tmp_clusters;
  pcl_copy_time(finput_, fclusters_);
}


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
#include "cluster_colors.h"
#ifdef HAVE_VISUAL_DEBUGGING
#  include "visualization_thread_base.h"
#endif

#include <pcl_utils/utils.h>
#include <pcl_utils/comparisons.h>
#include <utils/time/wait.h>

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
#include <pcl/registration/icp.h>

#include <interfaces/Position3DInterface.h>
#include <interfaces/SwitchInterface.h>

#include <iostream>
using namespace std;

#define CFG_PREFIX "/perception/tabletop-objects/"

/** @class TabletopObjectsThread "tabletop_objects_thread.h"
 * Main thread of tabletop objects plugin.
 * @author Tim Niemueller
 */

using namespace fawkes;

/** Constructor. */
TabletopObjectsThread::TabletopObjectsThread()
  : Thread("TabletopObjectsThread", Thread::OPMODE_CONTINUOUS),
    TransformAspect(TransformAspect::ONLY_LISTENER)
{
#ifdef HAVE_VISUAL_DEBUGGING
  visthread_ = NULL;
#endif
}


/** Destructor. */
TabletopObjectsThread::~TabletopObjectsThread()
{
}


void
TabletopObjectsThread::init()
{
  cfg_depth_filter_min_x_ = config->get_float(CFG_PREFIX"depth_filter_min_x");
  cfg_depth_filter_max_x_ = config->get_float(CFG_PREFIX"depth_filter_max_x");
  cfg_voxel_leaf_size_    = config->get_float(CFG_PREFIX"voxel_leaf_size");
  cfg_segm_max_iterations_ =
    config->get_uint(CFG_PREFIX"table_segmentation_max_iterations");
  cfg_segm_distance_threshold_ =
    config->get_float(CFG_PREFIX"table_segmentation_distance_threshold");
  cfg_segm_inlier_quota_ =
    config->get_float(CFG_PREFIX"table_segmentation_inlier_quota");
  cfg_max_z_angle_deviation_ = config->get_float(CFG_PREFIX"max_z_angle_deviation");
  cfg_table_min_height_      = config->get_float(CFG_PREFIX"table_min_height");
  cfg_table_max_height_      = config->get_float(CFG_PREFIX"table_max_height");
  cfg_cluster_tolerance_     = config->get_float(CFG_PREFIX"cluster_tolerance");
  cfg_cluster_min_size_      = config->get_uint(CFG_PREFIX"cluster_min_size");
  cfg_cluster_max_size_      = config->get_uint(CFG_PREFIX"cluster_max_size");
  cfg_result_frame_          = config->get_string(CFG_PREFIX"result_frame");

  cfg_table_model_width  = 1.60;
  cfg_table_model_height = 0.80;
  cfg_table_model_step   = 0.05;

  finput_ = pcl_manager->get_pointcloud<PointType>("openni-pointcloud");
  input_ = pcl_utils::cloudptr_from_refptr(finput_);

  try {
    double rotation[4] = {0., 0., 0., 1.};
    table_pos_if_ = NULL;
    table_pos_if_ = blackboard->open_for_writing<Position3DInterface>("Tabletop Height");
    table_pos_if_->set_rotation(rotation);
    table_pos_if_->write();

    switch_if_ = NULL;
    switch_if_ = blackboard->open_for_writing<SwitchInterface>("tabletop-objects");
    switch_if_->set_enabled(true);
    switch_if_->write();

    pos_ifs_.clear();
    pos_ifs_.resize(MAX_CENTROIDS, NULL);
    for (unsigned int i = 0; i < MAX_CENTROIDS; ++i) {
      char *tmp;
      if (asprintf(&tmp, "Tabletop Object %u", i + 1) != -1) {
        // Copy to get memory freed on exception
        std::string id = tmp;
        free(tmp);
        Position3DInterface *iface =
          blackboard->open_for_writing<Position3DInterface>(id.c_str());
        pos_ifs_[i] = iface;
        iface->set_rotation(rotation);
        iface->write();
      }
    }
  } catch (Exception &e) {
    blackboard->close(table_pos_if_);
    blackboard->close(switch_if_);
    for (unsigned int i = 0; i < MAX_CENTROIDS; ++i) {
      if (pos_ifs_[i]) {
        blackboard->close(pos_ifs_[i]);
      }
    }
    pos_ifs_.clear();
    throw;
  }

  fclusters_ = new pcl::PointCloud<ColorPointType>();
  fclusters_->header.frame_id = finput_->header.frame_id;
  fclusters_->is_dense = false;
  pcl_manager->add_pointcloud<ColorPointType>("tabletop-object-clusters", fclusters_);
  clusters_ = pcl_utils::cloudptr_from_refptr(fclusters_);

  ftable_model_ = new Cloud();
  table_model_ = pcl_utils::cloudptr_from_refptr(ftable_model_);
  CloudPtr generated = generate_table_model(1.6, 0.8, 0.05);
  *table_model_ = *generated;
  ftable_model_->header.frame_id = finput_->header.frame_id;
  pcl_manager->add_pointcloud("tabletop-table-model", ftable_model_);
  pcl_utils::set_time(ftable_model_, fawkes::Time(clock));

  grid_.setFilterFieldName("x");
  grid_.setFilterLimits(cfg_depth_filter_min_x_, cfg_depth_filter_max_x_);
  grid_.setLeafSize(cfg_voxel_leaf_size_, cfg_voxel_leaf_size_, cfg_voxel_leaf_size_);

  seg_.setOptimizeCoefficients(true);
  seg_.setModelType(pcl::SACMODEL_PLANE);
  seg_.setMethodType(pcl::SAC_RANSAC);
  seg_.setMaxIterations(cfg_segm_max_iterations_);
  seg_.setDistanceThreshold(cfg_segm_distance_threshold_);
}


void
TabletopObjectsThread::finalize()
{
  input_.reset();
  clusters_.reset();

  pcl_manager->remove_pointcloud("tabletop-object-clusters");
  
  blackboard->close(table_pos_if_);
  blackboard->close(switch_if_);
  for (unsigned int i = 0; i < MAX_CENTROIDS; ++i) {
    blackboard->close(pos_ifs_[i]);
  }
  pos_ifs_.clear();

  finput_.reset();
  fclusters_.reset();
}


void
TabletopObjectsThread::loop()
{
  while (! switch_if_->msgq_empty()) {
    if (SwitchInterface::EnableSwitchMessage *msg =
        switch_if_->msgq_first_safe(msg))
    {
      switch_if_->set_enabled(true);
      switch_if_->write();
    } else if (SwitchInterface::DisableSwitchMessage *msg =
               switch_if_->msgq_first_safe(msg))
    {
      switch_if_->set_enabled(false);
      switch_if_->write();
    }

    switch_if_->msgq_pop();
  }

  if (! switch_if_->is_enabled()) {
    TimeWait::wait(250000);
    return;
  }

  CloudPtr temp_cloud(new Cloud);
  CloudPtr temp_cloud2(new Cloud);
  pcl::ExtractIndices<PointType> extract_;
  std::vector<pcl::Vertices> vertices_;
  CloudPtr cloud_hull_;
  CloudPtr cloud_proj_;
  CloudPtr cloud_filt_;
  CloudPtr cloud_above_;
  CloudPtr cloud_objs_;
  pcl::KdTreeFLANN<PointType> kdtree_;

  grid_.setInputCloud(input_);
  grid_.filter(*temp_cloud);

  if (temp_cloud->points.size() <= 10) {
    // this can happen if run at startup. Since tabletop threads runs continuous
    // and not synchronized with main loop, but point cloud acquisition thread is
    // synchronized, we might start before any data has been read
    //logger->log_warn(name(), "Empty voxelized point cloud, omitting loop");
    return;
  }

  pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  Eigen::Vector4f table_centroid;

  // This will search for the first plane which:
  // 1. has a considerable amount of points (>= 2% of input points)
  // 2. is parallel to the floor (transformed normal Z dominant axis)
  // 3. is on a typical table height (between 0.3 to 1.0m in robot frame)
  // Planes found along the way not satisfying any of the criteria are removed,
  // the first plane either satisfying all criteria, or violating the first
  // one end the loop
  bool happy_with_plane = false;
  while (! happy_with_plane) {
    happy_with_plane = true;

    seg_.setInputCloud(temp_cloud);
    seg_.segment(*inliers, *coeff);

    // 1. check for a minimum number of expected inliers
    if (inliers->indices.size() < (cfg_segm_inlier_quota_ * input_->points.size())) {
      //logger->log_warn(name(), "No table in scene, skipping loop");
      set_position(table_pos_if_, false, table_centroid);
      return;
    }

    // 2. Check angle between normal vector and Z axis of the
    // base_link robot frame since tables are usually parallel to the ground...
    try {
      tf::Stamped<tf::Vector3>
        table_normal(tf::Vector3(coeff->values[0], coeff->values[1], coeff->values[2]),
                     fawkes::Time(0,0), input_->header.frame_id);

      tf::Stamped<tf::Vector3> baserel_normal;
      tf_listener->transform_vector("/base_link", table_normal, baserel_normal);
      tf::Vector3 z_axis(0, 0, copysign(1.0, baserel_normal.z()));

      if (fabs(z_axis.angle(baserel_normal)) > cfg_max_z_angle_deviation_ ) {
        happy_with_plane = false;
        logger->log_warn(name(), "Table normal (%f,%f,%f) Z angle deviation |%f| > %f, excluding",
                         baserel_normal.x(), baserel_normal.y(), baserel_normal.z(),
                         z_axis.angle(baserel_normal), cfg_max_z_angle_deviation_);
      }
    } catch (tf::TransformException &e) {
      //logger->log_warn(name(), "Transforming normal failed, exception follows");
      //logger->log_warn(name(), e);
    }

    // 3. Calculate table centroid, then transform it to the base_link system
    // to make a table height sanity check, they tend to be at a specific height...
    try {
      pcl::compute3DCentroid(*temp_cloud, *inliers, table_centroid);
      tf::Stamped<tf::Point>
        centroid(tf::Point(table_centroid[0], table_centroid[1], table_centroid[2]),
                 fawkes::Time(0, 0), input_->header.frame_id);
      tf::Stamped<tf::Point> baserel_centroid;
      tf_listener->transform_point("/base_link", centroid, baserel_centroid);
      if ((baserel_centroid.z() < cfg_table_min_height_) ||
          (baserel_centroid.z() > cfg_table_max_height_))
      {
        happy_with_plane = false;
        //logger->log_warn(name(), "Table height %f not in range [%f, %f]", baserel_centroid.z(),
        //                 cfg_table_min_height_, cfg_table_max_height_);
      }
    } catch (tf::TransformException &e) {
      //logger->log_warn(name(), "Transforming centroid failed, exception follows");
      //logger->log_warn(name(), e);
    }

    if (! happy_with_plane) {
      // throw away 
      Cloud extracted;
      extract_.setNegative(true);
      extract_.setInputCloud(temp_cloud);
      extract_.setIndices(inliers);
      extract_.filter(extracted);
      *temp_cloud = extracted;
    }
  }

  // If we got here we found the table
  set_position(table_pos_if_, true, table_centroid);

  extract_.setNegative(false);
  extract_.setInputCloud(temp_cloud);
  extract_.setIndices(inliers);
  extract_.filter(*temp_cloud2);


  // Project the model inliers
  pcl::ProjectInliers<PointType> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(temp_cloud2);
  proj.setModelCoefficients(coeff);
  cloud_proj_.reset(new Cloud());
  proj.filter(*cloud_proj_);


  // Estimate 3D convex hull -> TABLE BOUNDARIES
  pcl::ConvexHull<PointType> hr;
  //hr.setAlpha(0.1);  // only for ConcaveHull
  hr.setInputCloud(cloud_proj_);
  cloud_hull_.reset(new Cloud());
  hr.reconstruct(*cloud_hull_, vertices_);


  // Fit table model into projected cloud using ICP
  CloudPtr table_model = generate_table_model(cfg_table_model_width,
                                              cfg_table_model_height,
                                              cfg_table_model_step);

  bool front_line_center_ok = true;
  tf::Stamped<tf::Point> front_line_center;
  try {
    tf::StampedTransform t;
    fawkes::Time input_time;
    pcl_utils::get_time(finput_, input_time);
    tf_listener->lookup_transform("/base_link", finput_->header.frame_id,
                                  input_time, t);

    logger->log_debug(name(), "Translation %s -> %s (%f,%f,%f)",
                      finput_->header.frame_id.c_str(), "/base_link",
                      t.getOrigin().x(), t.getOrigin().y(), t.getOrigin().z());

    tf::Quaternion q = t.getRotation();
    Eigen::Affine3f affine_cloud =
      Eigen::Translation3f(t.getOrigin().x(), t.getOrigin().y(), t.getOrigin().z())
      * Eigen::Quaternionf(q.w(), q.x(), q.y(), q.z());

    CloudPtr baserel_polygon_cloud(new Cloud());
    pcl::transformPointCloud(*cloud_hull_, *baserel_polygon_cloud, affine_cloud);
    
    Eigen::Vector3f p_left(std::numeric_limits<float>::max(), 0, 0);
    Eigen::Vector3f p_right(std::numeric_limits<float>::max(), 0, 0);
    for (size_t i = 0; i < baserel_polygon_cloud->points.size(); ++i) {
      PointType &p = baserel_polygon_cloud->points[i];

      if ((p.x < p_left[0]) || (p.x < p_right[0])) {
        if (p.y < 0)  {
          p_left[0] = p.x;
          p_left[1] = p.y;
          p_left[2] = p.z;
        } else {
          p_right[0] = p.x;
          p_right[1] = p.y;
          p_right[2] = p.z;
        }
      }
    }

    tf::Stamped<tf::Point> baserel_front_line_center;
    baserel_front_line_center.setX((p_left[0] + p_right[0] + cfg_table_model_height) * 0.5);
    baserel_front_line_center.setY((p_left[1] + p_right[1]) * 0.5);
    baserel_front_line_center.setZ((p_left[2] + p_right[2]) * 0.5);
    baserel_front_line_center.frame_id = t.frame_id;
    baserel_front_line_center.stamp = t.stamp;

    logger->log_info(name(), "Baserel front left=(%f,%f,%f)  right=(%f,%f,%f)  center=(%f,%f,%f)",
                     p_left[0], p_left[1], p_left[2],
                     p_right[0], p_right[1], p_right[2],
                     baserel_front_line_center.x(), baserel_front_line_center.y(),
                     baserel_front_line_center.z());

    tf_listener->transform_point(finput_->header.frame_id,
                                 baserel_front_line_center, front_line_center);
  } catch (Exception &e) {
    logger->log_warn(name(), "Failed to transform convex hull cloud: %s", e.what());
    front_line_center_ok = false;
  }

  if (front_line_center_ok) {
    // First create initial transformation guess based on closest polygon line and normal
    Eigen::Vector3f model_normal(1, 0, 0);
    Eigen::Vector3f normal(coeff->values[0], coeff->values[1], coeff->values[2]);
    Eigen::Vector3f rotaxis = model_normal.cross(normal);
    rotaxis.normalize();
    // -M_PI/2 because we calc the angle for the normal
    float angle = acosf(normal.dot(model_normal)) - M_PI/2.f;  
  
    Eigen::Affine3f affine =
      //Eigen::Translation3f(table_centroid[0], table_centroid[1], table_centroid[2])
      Eigen::Translation3f(front_line_center.x(), front_line_center.y(), front_line_center.z())
      * Eigen::AngleAxisf(angle, rotaxis);

    CloudPtr sparse_table(new Cloud());
    pcl::VoxelGrid<PointType> grid_table_icp;
    grid_table_icp.setLeafSize(0.05, 0.05, 0.05);
    grid_table_icp.setInputCloud(cloud_proj_);
    grid_table_icp.filter(*sparse_table);

    logger->log_info(name(), "Proj: %zu  Sparse: %zu  Table: %zu", cloud_proj_->points.size(), sparse_table->points.size(), table_model->points.size());

    logger->log_debug(name(), "Starting ICP");
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    //icp.setMaxCorrespondenceDistance(0.05);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations(50);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon(0.01);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon(0.01);
    icp.setInputCloud(table_model);
    icp.setInputTarget(sparse_table);

    Cloud final;
    logger->log_debug(name(), "ICP aligning");
    icp.align(final, affine.matrix());
    logger->log_debug(name(), "ICP finished");
    if (! icp.hasConverged()) {
      logger->log_warn(name(), "Table ICP did not converge");
    } else {
      //Eigen::Matrix4f m = icp.getFinalTransformation();
      logger->log_warn(name(), "Converged with score %f", icp.getFitnessScore());
    }

    // to show initial guess:
    pcl::transformPointCloud(*table_model, *table_model_, affine.matrix());
    // to show final alignment:
    //*table_model_ = final;
    table_model_->header.frame_id = finput_->header.frame_id;
  }


  // Extract all non-plane points
  cloud_filt_.reset(new Cloud());
  extract_.setNegative(true);
  extract_.filter(*cloud_filt_);

  // Use only points above tables
  // Why coeff->values[3] > 0 ? ComparisonOps::GT : ComparisonOps::LT?
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
    coeff->values[3] > 0 ? pcl::ComparisonOps::GT : pcl::ComparisonOps::LT;
  typename pcl_utils::PlaneDistanceComparison<PointType>::ConstPtr
    above_comp(new pcl_utils::PlaneDistanceComparison<PointType>(coeff, op));
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
  if (cloud_filt_->points.size() < cfg_cluster_min_size_) {
    //logger->log_warn(name(), "Less points than cluster min size");
    return;
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

    typename pcl_utils::PolygonComparison<PointType>::ConstPtr
      inpoly_comp(new pcl_utils::PolygonComparison<PointType>(polygon_cloud));
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
  tmp_clusters->width = indices.size();
  tmp_clusters->points.resize(indices.size());
  for (size_t i = 0; i < indices.size(); ++i) {
    PointType &p1 = temp_cloud2->points[i];
    ColorPointType &p2 = tmp_clusters->points[i];
    p2.x = p1.x;
    p2.y = p1.y;
    p2.z = p1.z;
    p2.r = table_color[0];
    p2.g = table_color[1];
    p2.b = table_color[2];
  }

  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > centroids;
  centroids.resize(MAX_CENTROIDS);
  unsigned int centroid_i = 0;

  if (cloud_objs_->points.size() > 0) {
    // Creating the KdTree object for the search method of the extraction
    pcl::KdTree<PointType>::Ptr
      kdtree_cl(new pcl::KdTreeFLANN<PointType>());
    kdtree_cl->setInputCloud(cloud_objs_);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance(cfg_cluster_tolerance_);
    ec.setMinClusterSize(cfg_cluster_min_size_);
    ec.setMaxClusterSize(cfg_cluster_max_size_);
    ec.setSearchMethod(kdtree_cl);
    ec.setInputCloud(cloud_objs_);
    ec.extract(cluster_indices);

    //logger->log_debug(name(), "Found %zu clusters", cluster_indices.size());

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

        std::vector<int>::const_iterator pit;
        for (pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
          ColorPointType &p1 = colored_clusters->points[cci++];
          PointType &p2 = cloud_objs_->points[*pit];
          p1.x = p2.x;
          p1.y = p2.y;
          p1.z = p2.z;
          p1.r = cluster_colors[centroid_i][0];
          p1.g = cluster_colors[centroid_i][1];;
          p1.b = cluster_colors[centroid_i][2];;
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
    set_position(pos_ifs_[i], i < centroid_i, centroids[i]);
  }
  centroids.resize(centroid_i);

  *clusters_ = *tmp_clusters;
  pcl_utils::copy_time(finput_, fclusters_);
  pcl_utils::copy_time(finput_, ftable_model_);

#ifdef HAVE_VISUAL_DEBUGGING
  if (visthread_) {
    Eigen::Vector4f normal;
    normal[0] = coeff->values[0];
    normal[1] = coeff->values[1];
    normal[2] = coeff->values[2];
    normal[3] = 0.;

    TabletopVisualizationThreadBase::V_Vector4f hull_vertices;
    std::vector<int> &v = vertices_[0].vertices;
    hull_vertices.resize(v.size());
    for (unsigned int i = 0; i < v.size(); ++i) {
      hull_vertices[i][0] = cloud_hull_->points[v[i]].x;
      hull_vertices[i][1] = cloud_hull_->points[v[i]].y;
      hull_vertices[i][2] = cloud_hull_->points[v[i]].z;
    }

    visthread_->visualize(input_->header.frame_id,
                          table_centroid, normal, hull_vertices, centroids);
  }
#endif
}


void
TabletopObjectsThread::set_position(fawkes::Position3DInterface *iface,
                                    bool is_visible, Eigen::Vector4f &centroid)
{
  tf::Stamped<tf::Point> baserel_centroid;
  try{
    tf::Stamped<tf::Point>
      scentroid(tf::Point(centroid[0], centroid[1], centroid[2]),
                fawkes::Time(0, 0), input_->header.frame_id);
    tf_listener->transform_point(cfg_result_frame_, scentroid, baserel_centroid);
    iface->set_frame(cfg_result_frame_.c_str());
  } catch (tf::TransformException &e) {
    is_visible = false;
  }

  int visibility_history = iface->visibility_history();
  if (is_visible) {
    if (visibility_history >= 0) {
      iface->set_visibility_history(visibility_history + 1);
    } else {
      iface->set_visibility_history(1);
    }
    double translation[3] = { baserel_centroid.x(), baserel_centroid.y(), baserel_centroid.z() };
    iface->set_translation(translation);
      
  } else {
    if (visibility_history <= 0) {
      iface->set_visibility_history(visibility_history - 1);
    } else {
      iface->set_visibility_history(-1);
      double translation[3] = { 0, 0, 0 };
      iface->set_translation(translation);
    }
  }
  iface->write();  
}


TabletopObjectsThread::CloudPtr
TabletopObjectsThread::generate_table_model(const float width, const float height,
                                            const float thickness, const float step,
                                            const float max_error)
{
  CloudPtr c(new Cloud());

  const float width_2     = fabs(width)     * 0.5;
  const float height_2    = fabs(height)    * 0.5;
  const float thickness_2 = fabs(thickness) * 0.5;

  // calculate table points
  const unsigned int w_base_num = std::max(2u, (unsigned int)floor(width / step));
  const unsigned int num_w = w_base_num +
    ((width < w_base_num * step) ? 0 : ((width - w_base_num * step) > max_error ? 2 : 1));
  const unsigned int h_base_num = std::max(2u, (unsigned int)floor(height / step));
  const unsigned int num_h = h_base_num +
    ((height < h_base_num * step) ? 0 : ((height - h_base_num * step) > max_error ? 2 : 1));
  const unsigned int t_base_num = std::max(2u, (unsigned int)floor(thickness / step));
  const unsigned int num_t = t_base_num +
    ((thickness < t_base_num * step) ? 0 : ((thickness - t_base_num * step) > max_error ? 2 : 1));

  logger->log_debug(name(), "Generating table model %fx%fx%f (%ux%ux%u=%u points)",
                    width, height, thickness,
                    num_w, num_h, num_t, num_t * num_w * num_h);

  c->height = 1;
  c->width = num_t * num_w * num_h;
  c->is_dense = true;
  c->points.resize(num_t * num_w * num_h);

  unsigned int idx = 0;
  for (unsigned int t = 0; t < num_t; ++t) {
    for (unsigned int w = 0; w < num_w; ++w) {
      for (unsigned int h = 0; h < num_h; ++h) {
        PointType &p = c->points[idx++];

        p.x = h * step - height_2;
        if ((h == num_h - 1) && fabs(p.x - height_2) > max_error) p.x = height_2;

        p.y = w * step - width_2;
        if ((w == num_w - 1) && fabs(p.y - width_2) > max_error)  p.y = width_2;

        p.z = t * step - thickness_2;
        if ((t == num_t - 1) && fabs(p.z - thickness_2) > max_error)  p.z = thickness_2;
      }
    }
  }

  return c;
}

TabletopObjectsThread::CloudPtr
TabletopObjectsThread::generate_table_model(const float width, const float height,
                                            const float step, const float max_error)
{
  CloudPtr c(new Cloud());

  const float width_2     = fabs(width)     * 0.5;
  const float height_2    = fabs(height)    * 0.5;

  // calculate table points
  const unsigned int w_base_num = std::max(2u, (unsigned int)floor(width / step));
  const unsigned int num_w = w_base_num +
    ((width < w_base_num * step) ? 0 : ((width - w_base_num * step) > max_error ? 2 : 1));
  const unsigned int h_base_num = std::max(2u, (unsigned int)floor(height / step));
  const unsigned int num_h = h_base_num +
    ((height < h_base_num * step) ? 0 : ((height - h_base_num * step) > max_error ? 2 : 1));

  logger->log_debug(name(), "Generating table model %fx%f (%ux%u=%u points)",
                    width, height, num_w, num_h, num_w * num_h);

  c->height = 1;
  c->width = num_w * num_h;
  c->is_dense = true;
  c->points.resize(num_w * num_h);

  unsigned int idx = 0;
  for (unsigned int w = 0; w < num_w; ++w) {
    for (unsigned int h = 0; h < num_h; ++h) {
      PointType &p = c->points[idx++];

      p.x = h * step - height_2;
      if ((h == num_h - 1) && fabs(p.x - height_2) > max_error) p.x = height_2;

      p.y = w * step - width_2;
      if ((w == num_w - 1) && fabs(p.y - width_2) > max_error)  p.y = width_2;

      p.z = 0.;
    }
  }

  return c;
}


#ifdef HAVE_VISUAL_DEBUGGING
void
TabletopObjectsThread::set_visualization_thread(TabletopVisualizationThreadBase *visthread)
{
  visthread_ = visthread;
}
#endif

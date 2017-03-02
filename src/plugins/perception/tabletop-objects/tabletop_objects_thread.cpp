
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
#include <utils/math/angle.h>
#ifdef USE_TIMETRACKER
#  include <utils/time/tracker.h>
#endif
#include <utils/time/tracker_macros.h>

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
#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>
#include <pcl/registration/distances.h>

#include <utils/hungarian_method/hungarian.h>


#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/filters/statistical_outlier_removal.h>

#include <interfaces/Position3DInterface.h>
#include <interfaces/SwitchInterface.h>

#include <iostream>
#include <algorithm>
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
  cfg_table_min_cluster_quota_ =
    config->get_float(CFG_PREFIX"table_min_cluster_quota");
  cfg_table_downsample_leaf_size_ =
    config->get_float(CFG_PREFIX"table_downsample_leaf_size");
  cfg_table_cluster_tolerance_ =
    config->get_float(CFG_PREFIX"table_cluster_tolerance");
  cfg_max_z_angle_deviation_ = config->get_float(CFG_PREFIX"max_z_angle_deviation");
  cfg_table_min_height_      = config->get_float(CFG_PREFIX"table_min_height");
  cfg_table_max_height_      = config->get_float(CFG_PREFIX"table_max_height");
  cfg_table_model_enable_    = config->get_bool(CFG_PREFIX"table_model_enable");
  cfg_table_model_length_    = config->get_float(CFG_PREFIX"table_model_length");
  cfg_table_model_width_     = config->get_float(CFG_PREFIX"table_model_width");
  cfg_table_model_step_      = config->get_float(CFG_PREFIX"table_model_step");
  cfg_horizontal_va_         = deg2rad(config->get_float(CFG_PREFIX"horizontal_viewing_angle"));
  cfg_vertical_va_           = deg2rad(config->get_float(CFG_PREFIX"vertical_viewing_angle"));
  cfg_cluster_tolerance_     = config->get_float(CFG_PREFIX"cluster_tolerance");
  cfg_cluster_min_size_      = config->get_uint(CFG_PREFIX"cluster_min_size");
  cfg_cluster_max_size_      = config->get_uint(CFG_PREFIX"cluster_max_size");
  cfg_base_frame_            = config->get_string("/frames/base");
  cfg_result_frame_          = config->get_string(CFG_PREFIX"result_frame");
  cfg_input_pointcloud_      = config->get_string(CFG_PREFIX"input_pointcloud");
  cfg_centroid_max_age_      = config->get_uint(CFG_PREFIX"centroid_max_age");
  cfg_centroid_max_distance_ = config->get_float(CFG_PREFIX"centroid_max_distance");
  cfg_centroid_min_distance_ = config->get_float(CFG_PREFIX"centroid_min_distance");
  cfg_centroid_max_height_   = config->get_float(CFG_PREFIX"centroid_max_height");
  cfg_cylinder_fitting_      = config->get_bool(CFG_PREFIX"enable_cylinder_fitting");
  cfg_track_objects_         = config->get_bool(CFG_PREFIX"enable_object_tracking");
  try {
    cfg_verbose_cylinder_fitting_ = config->get_bool(CFG_PREFIX"verbose_cylinder_fitting");
  } catch (const Exception &e) {
    cfg_verbose_cylinder_fitting_ = false;
  }

  if (pcl_manager->exists_pointcloud<PointType>(cfg_input_pointcloud_.c_str())) {
    finput_ = pcl_manager->get_pointcloud<PointType>(cfg_input_pointcloud_.c_str());
    input_ = pcl_utils::cloudptr_from_refptr(finput_);
  } else if (pcl_manager->exists_pointcloud<ColorPointType>(cfg_input_pointcloud_.c_str())) {
    logger->log_warn(name(), "XYZ/RGB input point cloud, conversion required");
    fcoloredinput_ =
      pcl_manager->get_pointcloud<ColorPointType>(cfg_input_pointcloud_.c_str());
    colored_input_ = pcl_utils::cloudptr_from_refptr(fcoloredinput_);
    converted_input_.reset(new Cloud());
    input_ = converted_input_;
    converted_input_->header.frame_id = colored_input_->header.frame_id;
    converted_input_->header.stamp    = colored_input_->header.stamp;
  } else {
    throw Exception("Point cloud '%s' does not exist or not XYZ or XYZ/RGB PCL",
		    cfg_input_pointcloud_.c_str());
  }

  try {
    double rotation[4] = {0., 0., 0., 1.};
    table_pos_if_ = NULL;
    switch_if_    = NULL;

    table_pos_if_ = blackboard->open_for_writing<Position3DInterface>("Tabletop");
    table_pos_if_->set_rotation(rotation);
    table_pos_if_->write();

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
    throw;
  }

  fclusters_ = new pcl::PointCloud<ColorPointType>();
  fclusters_->header.frame_id = input_->header.frame_id;
  fclusters_->is_dense = false;
  pcl_manager->add_pointcloud<ColorPointType>("tabletop-object-clusters", fclusters_);
  clusters_ = pcl_utils::cloudptr_from_refptr(fclusters_);

  char *tmp_name;
  fawkes::RefPtr<pcl::PointCloud<ColorPointType> > f_tmp_cloud;
  pcl::PointCloud<ColorPointType>::Ptr tmp_cloud;
  for (int i = 0; i < MAX_CENTROIDS; i++) {
    f_tmp_cloud = new pcl::PointCloud<ColorPointType>();
    f_tmp_cloud->header.frame_id = input_->header.frame_id;
    f_tmp_cloud->is_dense = false;
    std::string obj_id;
    if (asprintf(&tmp_name, "obj_cluster_%u", i) != -1) {
      obj_id = tmp_name;
      free(tmp_name);
    }
    pcl_manager->add_pointcloud<ColorPointType> (obj_id.c_str(), f_tmp_cloud);
    f_obj_clusters_.push_back(f_tmp_cloud);
    tmp_cloud = pcl_utils::cloudptr_from_refptr(f_tmp_cloud);
    obj_clusters_.push_back(tmp_cloud);
  }

  ////////////////////////////////////////////////////////////
  //TODO must do initialization better (look-up table for known objects)
//  obj_shape_confidence_.resize(MAX_CENTROIDS, 0.0);
  NUM_KNOWN_OBJS_ = 3;
  std::vector<double> init_likelihoods;
  init_likelihoods.resize(NUM_KNOWN_OBJS_ + 1, 0.0);
  // TODO obj_likelihoods_ initialization
//  obj_likelihoods_.resize(MAX_CENTROIDS, init_likelihoods);
//  best_obj_guess_.resize(MAX_CENTROIDS, -1);

  known_obj_dimensions_.resize(NUM_KNOWN_OBJS_);

  //Green cup
  known_obj_dimensions_[0][0] = 0.07;
  known_obj_dimensions_[0][1] = 0.07;
  known_obj_dimensions_[0][2] = 0.104;
  //Red cup
  known_obj_dimensions_[1][0] = 0.088;
  known_obj_dimensions_[1][1] = 0.088;
  known_obj_dimensions_[1][2] = 0.155;
  //White cylinder
  known_obj_dimensions_[2][0] = 0.106;
  known_obj_dimensions_[2][1] = 0.106;
  known_obj_dimensions_[2][2] = 0.277;

  ////////////////////////////////////////////////////////////

  table_inclination_ = 0.0;

  ftable_model_ = new Cloud();
  table_model_ = pcl_utils::cloudptr_from_refptr(ftable_model_);
  table_model_->header.frame_id = input_->header.frame_id;
  pcl_manager->add_pointcloud("tabletop-table-model", ftable_model_);
  pcl_utils::set_time(ftable_model_, fawkes::Time(clock));

  fsimplified_polygon_ = new Cloud();
  simplified_polygon_ = pcl_utils::cloudptr_from_refptr(fsimplified_polygon_);
  simplified_polygon_->header.frame_id = input_->header.frame_id;
  pcl_manager->add_pointcloud("tabletop-simplified-polygon", fsimplified_polygon_);
  pcl_utils::set_time(fsimplified_polygon_, fawkes::Time(clock));

  grid_.setFilterFieldName("x");
  grid_.setFilterLimits(cfg_depth_filter_min_x_, cfg_depth_filter_max_x_);
  grid_.setLeafSize(cfg_voxel_leaf_size_, cfg_voxel_leaf_size_, cfg_voxel_leaf_size_);

  seg_.setOptimizeCoefficients(true);
  seg_.setModelType(pcl::SACMODEL_PLANE);
  seg_.setMethodType(pcl::SAC_RANSAC);
  seg_.setMaxIterations(cfg_segm_max_iterations_);
  seg_.setDistanceThreshold(cfg_segm_distance_threshold_);

  loop_count_ = 0;

  last_pcl_time_ = new Time(clock);

  first_run_ = true;

  old_centroids_.clear();
  for (unsigned int i = 0; i < MAX_CENTROIDS; i++)
    free_ids_.push_back(i);

#ifdef USE_TIMETRACKER
  tt_ = new TimeTracker();
  tt_loopcount_ = 0;
  ttc_full_loop_          = tt_->add_class("Full Loop");
  ttc_msgproc_            = tt_->add_class("Message Processing");
  ttc_convert_            = tt_->add_class("Input Conversion");
  ttc_voxelize_           = tt_->add_class("Downsampling");
  ttc_plane_              = tt_->add_class("Plane Segmentation");
  ttc_extract_plane_      = tt_->add_class("Plane Extraction");
  ttc_plane_downsampling_ = tt_->add_class("Plane Downsampling");
  ttc_cluster_plane_      = tt_->add_class("Plane Clustering");
  ttc_convex_hull_        = tt_->add_class("Convex Hull");
  ttc_simplify_polygon_   = tt_->add_class("Polygon Simplification");
  ttc_find_edge_          = tt_->add_class("Polygon Edge");
  ttc_transform_          = tt_->add_class("Transform");
  ttc_transform_model_    = tt_->add_class("Model Transformation");
  ttc_extract_non_plane_  = tt_->add_class("Non-Plane Extraction");
  ttc_polygon_filter_     = tt_->add_class("Polygon Filter");
  ttc_table_to_output_    = tt_->add_class("Table to Output");
  ttc_cluster_objects_    = tt_->add_class("Object Clustering");
  ttc_visualization_      = tt_->add_class("Visualization");
  ttc_hungarian_          = tt_->add_class("Hungarian Method (centroids)");
  ttc_old_centroids_      = tt_->add_class("Old Centroid Removal");
  ttc_obj_extraction_      = tt_->add_class("Object Extraction");
#endif
}


void
TabletopObjectsThread::finalize()
{
  input_.reset();
  clusters_.reset();
  simplified_polygon_.reset();

  pcl_manager->remove_pointcloud("tabletop-object-clusters");
  pcl_manager->remove_pointcloud("tabletop-table-model");
  pcl_manager->remove_pointcloud("tabletop-simplified-polygon");

  blackboard->close(table_pos_if_);
  blackboard->close(switch_if_);
  for (PosIfsVector::iterator it = pos_ifs_.begin(); it != pos_ifs_.end(); it++) {
    blackboard->close(*it);
  }
  pos_ifs_.clear();

  finput_.reset();
  fclusters_.reset();
  ftable_model_.reset();
  fsimplified_polygon_.reset();
}

template <typename PointType>
inline bool
comparePoints2D(const PointType &p1, const PointType &p2)
{
  double angle1 = atan2(p1.y, p1.x) + M_PI;
  double angle2 = atan2(p2.y, p2.x) + M_PI;
  return (angle1 > angle2);
}


// Criteria for *not* choosing a segment:
// 1. the existing current best is clearly closer in base-relative X direction
// 2. the existing current best is longer
bool
TabletopObjectsThread::is_polygon_edge_better(PointType &cb_br_p1p, PointType &cb_br_p2p,
                                              PointType &br_p1p, PointType &br_p2p)
{
  // current best base-relative points
  Eigen::Vector3f cb_br_p1(cb_br_p1p.x, cb_br_p1p.y, cb_br_p1p.z);
  Eigen::Vector3f cb_br_p2(cb_br_p2p.x, cb_br_p2p.y, cb_br_p2p.z);
  Eigen::Vector3f cb_br_p1_p2_center = (cb_br_p1 + cb_br_p2) * 0.5;

  Eigen::Vector3f br_p1(br_p1p.x, br_p1p.y, br_p1p.z);
  Eigen::Vector3f br_p2(br_p2p.x, br_p2p.y, br_p2p.z);
  Eigen::Vector3f br_p1_p2_center = (br_p2 + br_p1) * 0.5;

  double dist_x = (cb_br_p1_p2_center[0] - br_p1_p2_center[0]);

  // Criteria for *not* choosing a segment:
  // 1. the existing current best is clearly closer in base-relative X direction
  // 2. the existing current best is longer
  if ( (dist_x < -0.25) ||
       ((abs(dist_x)  <= 0.25) && ((br_p2 - br_p1).norm() < (cb_br_p2 - cb_br_p1).norm())) )
    return false;
  else
    return true;
}


void
TabletopObjectsThread::loop()
{
  TIMETRACK_START(ttc_full_loop_);

  ++loop_count_;

  TIMETRACK_START(ttc_msgproc_);

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
    TIMETRACK_ABORT(ttc_full_loop_);
    return;
  }

  TIMETRACK_END(ttc_msgproc_);

  fawkes::Time pcl_time;
  if (colored_input_) {
    pcl_utils::get_time(colored_input_, pcl_time);
  } else {
    pcl_utils::get_time(input_, pcl_time);
  }
  if (*last_pcl_time_ == pcl_time) {
    TimeWait::wait(20000);
    TIMETRACK_ABORT(ttc_full_loop_);
    return;
  }
  *last_pcl_time_ = pcl_time;

  if (colored_input_) {
    TIMETRACK_START(ttc_convert_);
    convert_colored_input();
    TIMETRACK_END(ttc_convert_);
  }

  TIMETRACK_START(ttc_voxelize_);

  CloudPtr temp_cloud(new Cloud);
  CloudPtr temp_cloud2(new Cloud);
  pcl::ExtractIndices<PointType> extract_;
  CloudPtr cloud_hull_;
  CloudPtr model_cloud_hull_;
  CloudPtr cloud_proj_;
  CloudPtr cloud_filt_;
  CloudPtr cloud_above_;
  CloudPtr cloud_objs_;
  pcl::search::KdTree<PointType> kdtree_;

  grid_.setInputCloud(input_);
  grid_.filter(*temp_cloud);

  if (temp_cloud->points.size() <= 10) {
    // this can happen if run at startup. Since tabletop threads runs continuous
    // and not synchronized with main loop, but point cloud acquisition thread is
    // synchronized, we might start before any data has been read
    //logger->log_warn(name(), "Empty voxelized point cloud, omitting loop");
    TimeWait::wait(50000);
    return;
  }

  TIMETRACK_INTER(ttc_voxelize_, ttc_plane_)

  pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  Eigen::Vector4f baserel_table_centroid(0,0,0,0);

  // This will search for the first plane which:
  // 1. has a considerable amount of points (>= some percentage of input points)
  // 2. is parallel to the floor (transformed normal angle to Z axis in specified epsilon)
  // 3. is on a typical table height (at a specified height range in robot frame)
  // Planes found along the way not satisfying any of the criteria are removed,
  // the first plane either satisfying all criteria, or violating the first
  // one end the loop
  bool happy_with_plane = false;
  while (! happy_with_plane) {
    happy_with_plane = true;

    if (temp_cloud->points.size() <= 10) {
      logger->log_warn(name(), "[L %u] no more points for plane detection, skipping loop", loop_count_);
      set_position(table_pos_if_, false);
      TIMETRACK_ABORT(ttc_plane_);
      TIMETRACK_ABORT(ttc_full_loop_);
      TimeWait::wait(50000);
      return;
    }

    seg_.setInputCloud(temp_cloud);
    seg_.segment(*inliers, *coeff);

    // 1. check for a minimum number of expected inliers
    if ((double)inliers->indices.size() < (cfg_segm_inlier_quota_ * (double)temp_cloud->points.size())) {
      logger->log_warn(name(), "[L %u] no table in scene, skipping loop (%zu inliers, required %f, voxelized size %zu)",
                       loop_count_, inliers->indices.size(),
                       (cfg_segm_inlier_quota_ * temp_cloud->points.size()), temp_cloud->points.size());
      set_position(table_pos_if_, false);
      TIMETRACK_ABORT(ttc_plane_);
      TIMETRACK_ABORT(ttc_full_loop_);
      TimeWait::wait(50000);
      return;
    }

    // 2. Check angle between normal vector and Z axis of the
    // base_link robot frame since tables are usually parallel to the ground...
    try {
      tf::Stamped<tf::Vector3>
        table_normal(tf::Vector3(coeff->values[0], coeff->values[1], coeff->values[2]),
                     fawkes::Time(0,0), input_->header.frame_id);

      tf::Stamped<tf::Vector3> baserel_normal;
      tf_listener->transform_vector(cfg_base_frame_, table_normal, baserel_normal);
      tf::Vector3 z_axis(0, 0, copysign(1.0, baserel_normal.z()));
      table_inclination_ = z_axis.angle(baserel_normal);
      if (fabs(z_axis.angle(baserel_normal)) > cfg_max_z_angle_deviation_) {
        happy_with_plane = false;
        logger->log_warn(name(), "[L %u] table normal (%f,%f,%f) Z angle deviation |%f| > %f, excluding",
                         loop_count_, baserel_normal.x(), baserel_normal.y(), baserel_normal.z(),
                         z_axis.angle(baserel_normal), cfg_max_z_angle_deviation_);
      }
    } catch (Exception &e) {
      logger->log_warn(name(), "Transforming normal failed, exception follows");
      logger->log_warn(name(), e);
      happy_with_plane = false;
    }

    if (happy_with_plane) {
      // ok so far

      // 3. Calculate table centroid, then transform it to the base_link system
      // to make a table height sanity check, they tend to be at a specific height...
      try {
        pcl::compute3DCentroid(*temp_cloud, *inliers, table_centroid);
        tf::Stamped<tf::Point>
          centroid(tf::Point(table_centroid[0], table_centroid[1], table_centroid[2]),
                   fawkes::Time(0, 0), input_->header.frame_id);
        tf::Stamped<tf::Point> baserel_centroid;
        tf_listener->transform_point(cfg_base_frame_, centroid, baserel_centroid);
        baserel_table_centroid[0] = baserel_centroid.x();
        baserel_table_centroid[1] = baserel_centroid.y();
        baserel_table_centroid[2] = baserel_centroid.z();

        if ((baserel_centroid.z() < cfg_table_min_height_) ||
            (baserel_centroid.z() > cfg_table_max_height_))
        {
          happy_with_plane = false;
          logger->log_warn(name(), "[L %u] table height %f not in range [%f, %f]",
                           loop_count_, baserel_centroid.z(),
                           cfg_table_min_height_, cfg_table_max_height_);
        }
      } catch (tf::TransformException &e) {
        //logger->log_warn(name(), "Transforming centroid failed, exception follows");
        //logger->log_warn(name(), e);
      }
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
  // Do NOT set it here, we will still try to determine the rotation as well
  // set_position(table_pos_if_, true, table_centroid);

  TIMETRACK_INTER(ttc_plane_, ttc_extract_plane_)

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

  TIMETRACK_INTER(ttc_extract_plane_, ttc_plane_downsampling_);

  // ***
  // In the following cluster the projected table plane. This is done to get
  // the largest continuous part of the plane to remove outliers, for instance
  // if the intersection of the plane with a wall or object is taken into the
  // table points.
  // To achieve this cluster, if an acceptable cluster was found, extract this
  // cluster as the new table points. Otherwise continue with the existing
  // point cloud.

  // further downsample table
  CloudPtr cloud_table_voxelized(new Cloud());
  pcl::VoxelGrid<PointType> table_grid;
  table_grid.setLeafSize(cfg_table_downsample_leaf_size_,
			 cfg_table_downsample_leaf_size_,
			 cfg_table_downsample_leaf_size_);
  table_grid.setInputCloud(cloud_proj_);
  table_grid.filter(*cloud_table_voxelized);

  TIMETRACK_INTER(ttc_plane_downsampling_, ttc_cluster_plane_);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<PointType>::Ptr
    kdtree_table(new pcl::search::KdTree<PointType>());
  kdtree_table->setInputCloud(cloud_table_voxelized);

  std::vector<pcl::PointIndices> table_cluster_indices;
  pcl::EuclideanClusterExtraction<PointType> table_ec;
  table_ec.setClusterTolerance(cfg_table_cluster_tolerance_);
  table_ec.setMinClusterSize(cfg_table_min_cluster_quota_
			     * cloud_table_voxelized->points.size());
  table_ec.setMaxClusterSize(cloud_table_voxelized->points.size());
  table_ec.setSearchMethod(kdtree_table);
  table_ec.setInputCloud(cloud_table_voxelized);
  table_ec.extract(table_cluster_indices);

  if (! table_cluster_indices.empty()) {
    // take the first, i.e. the largest cluster
    CloudPtr cloud_table_extracted(new Cloud());
    pcl::PointIndices::ConstPtr table_cluster_indices_ptr(new pcl::PointIndices(table_cluster_indices[0]));
    pcl::ExtractIndices<PointType> table_cluster_extract;
    table_cluster_extract.setNegative(false);
    table_cluster_extract.setInputCloud(cloud_table_voxelized);
    table_cluster_extract.setIndices(table_cluster_indices_ptr);
    table_cluster_extract.filter(*cloud_table_extracted);
    *cloud_proj_ = *cloud_table_extracted;

    // recompute based on the new chosen table cluster
    pcl::compute3DCentroid(*cloud_proj_, table_centroid);

  } else {
    // Don't mess with the table, clustering didn't help to make it any better
    logger->log_info(name(), "[L %u] table plane clustering did not generate any clusters", loop_count_);
  }

  TIMETRACK_INTER(ttc_cluster_plane_, ttc_convex_hull_)


  // Estimate 3D convex hull -> TABLE BOUNDARIES
  pcl::ConvexHull<PointType> hr;
#ifdef PCL_VERSION_COMPARE
#if PCL_VERSION_COMPARE(>=,1,5,0)
  hr.setDimension(2);
#endif
#endif

  //hr.setAlpha(0.1);  // only for ConcaveHull
  hr.setInputCloud(cloud_proj_);
  cloud_hull_.reset(new Cloud());
  hr.reconstruct(*cloud_hull_);


  if (cloud_hull_->points.empty()) {
    logger->log_warn(name(), "[L %u] convex hull of table empty, skipping loop", loop_count_);
    TIMETRACK_ABORT(ttc_convex_hull_);
    TIMETRACK_ABORT(ttc_full_loop_);
    set_position(table_pos_if_, false);
    return;
  }

  TIMETRACK_INTER(ttc_convex_hull_, ttc_simplify_polygon_)

  CloudPtr simplified_polygon = simplify_polygon(cloud_hull_, 0.02);
  *simplified_polygon_ = *simplified_polygon;
  //logger->log_debug(name(), "Original polygon: %zu  simplified: %zu", cloud_hull_->points.size(),
  //                  simplified_polygon->points.size());
  *cloud_hull_ = *simplified_polygon;

  TIMETRACK_INTER(ttc_simplify_polygon_, ttc_find_edge_)

#ifdef HAVE_VISUAL_DEBUGGING
  TabletopVisualizationThreadBase::V_Vector4f good_hull_edges;
  good_hull_edges.resize(cloud_hull_->points.size() * 2);
#endif

  try {

    // Get transform Input camera -> base_link
    tf::StampedTransform t;
    fawkes::Time input_time(0,0);
    //pcl_utils::get_time(input_, input_time);
    tf_listener->lookup_transform(cfg_base_frame_, input_->header.frame_id,
                                  input_time, t);

    tf::Quaternion q = t.getRotation();
    Eigen::Affine3f affine_cloud =
      Eigen::Translation3f(t.getOrigin().x(), t.getOrigin().y(), t.getOrigin().z())
      * Eigen::Quaternionf(q.w(), q.x(), q.y(), q.z());

    // Transform polygon cloud into base_link frame
    CloudPtr baserel_polygon_cloud(new Cloud());
    pcl::transformPointCloud(*cloud_hull_, *baserel_polygon_cloud, affine_cloud);

    // Setup plane normals for left, right, and lower frustrum
    // planes for line segment verification
    Eigen::Vector3f left_frustrum_normal =
      Eigen::AngleAxisf(  cfg_horizontal_va_ * 0.5, Eigen::Vector3f::UnitZ())
      * (-1. * Eigen::Vector3f::UnitY());

    Eigen::Vector3f right_frustrum_normal =
      Eigen::AngleAxisf(- cfg_horizontal_va_ * 0.5, Eigen::Vector3f::UnitZ())
      * Eigen::Vector3f::UnitY();

    Eigen::Vector3f lower_frustrum_normal =
      Eigen::AngleAxisf(cfg_vertical_va_ * 0.5, Eigen::Vector3f::UnitY())
      * Eigen::Vector3f::UnitZ();

    // point and good edge indexes of chosen candidate
    size_t pidx1, pidx2;
#ifdef HAVE_VISUAL_DEBUGGING
    size_t geidx1 = std::numeric_limits<size_t>::max();
    size_t geidx2 = std::numeric_limits<size_t>::max();
#endif
    // lower frustrum potential candidate
    size_t lf_pidx1, lf_pidx2;
    pidx1 = pidx2 = lf_pidx1 = lf_pidx2 = std::numeric_limits<size_t>::max();

    // Search for good edges and backup edge candidates
    // Good edges are the ones with either points close to
    // two separate frustrum planes, and hence the actual line is
    // well inside the frustrum, or with points inside the frustrum.
    // Possible backup candidates are lines with both points
    // close to the lower frustrum plane, i.e. lines cutoff by the
    // vertical viewing angle. If we cannot determine a suitable edge
    // otherwise we fallback to this line as it is a good rough guess
    // to prevent at least worst things during manipulation
    const size_t psize = cloud_hull_->points.size();
#ifdef HAVE_VISUAL_DEBUGGING
    size_t good_edge_points = 0;
#endif
    for (size_t i = 0; i < psize; ++i) {
      //logger->log_debug(name(), "Checking %zu and %zu of %zu", i, (i+1) % psize, psize);
      PointType &p1p = cloud_hull_->points[i          ];
      PointType &p2p = cloud_hull_->points[(i+1) % psize];

      Eigen::Vector3f p1(p1p.x, p1p.y, p1p.z);
      Eigen::Vector3f p2(p2p.x, p2p.y, p2p.z);

      PointType &br_p1p = baserel_polygon_cloud->points[i              ];
      PointType &br_p2p = baserel_polygon_cloud->points[(i + 1) % psize];

      // check if both end points are close to left or right frustrum plane
      if ( ! (((left_frustrum_normal.dot(p1) < 0.03) &&
               (left_frustrum_normal.dot(p2) < 0.03)) ||
              ((right_frustrum_normal.dot(p1) < 0.03) &&
               (right_frustrum_normal.dot(p2) < 0.03)) ) )
      {
        // candidate edge, i.e. it's not too close to left or right frustrum planes

        // check if both end points close to lower frustrum plane
        if ((lower_frustrum_normal.dot(p1) < 0.01) &&
            (lower_frustrum_normal.dot(p2) < 0.01)) {
          // it's a lower frustrum line, keep just in case we do not
          // find a better one
          if ( (lf_pidx1 == std::numeric_limits<size_t>::max()) ||
               is_polygon_edge_better(br_p1p, br_p2p,
                                      baserel_polygon_cloud->points[lf_pidx1], baserel_polygon_cloud->points[lf_pidx2]))
          {
            // there was no backup candidate, yet, or this one is closer
            // to the robot, take it.
            lf_pidx1 = i;
            lf_pidx2 = (i + 1) % psize;
          }

          continue;
        }

#ifdef HAVE_VISUAL_DEBUGGING
        // Remember as good edge for visualization
        for (unsigned int j = 0; j < 3; ++j)
          good_hull_edges[good_edge_points][j] = p1[j];
        good_hull_edges[good_edge_points][3] = 0.;
        ++good_edge_points;
        for (unsigned int j = 0; j < 3; ++j)
          good_hull_edges[good_edge_points][j] = p2[j];
        good_hull_edges[good_edge_points][3] = 0.;
        ++good_edge_points;
#endif

        if (pidx1 != std::numeric_limits<size_t>::max()) {
          // current best base-relative points
          PointType &cb_br_p1p = baserel_polygon_cloud->points[pidx1];
          PointType &cb_br_p2p = baserel_polygon_cloud->points[pidx2];

          if (! is_polygon_edge_better(cb_br_p1p, cb_br_p2p, br_p1p, br_p2p))
          {
            //logger->log_info(name(), "Skipping: cb(%f,%f)->(%f,%f) c(%f,%f)->(%f,%f)",
            //                 cb_br_p1p.x, cb_br_p1p.y, cb_br_p2p.x, cb_br_p2p.y,
            //                 br_p1p.x, br_p1p.y, br_p2p.x, br_p2p.y);
            continue;
          } else {
            //logger->log_info(name(), "Taking: cb(%f,%f)->(%f,%f) c(%f,%f)->(%f,%f)",
            //                 cb_br_p1p.x, cb_br_p1p.y, cb_br_p2p.x, cb_br_p2p.y,
            //                 br_p1p.x, br_p1p.y, br_p2p.x, br_p2p.y);
          }
        //} else {
          //logger->log_info(name(), "Taking because we had none");
        }

        // Was not sorted out, therefore promote candidate to current best
        pidx1 = i;
        pidx2 = (i + 1) % psize;
#ifdef HAVE_VISUAL_DEBUGGING
        geidx1 = good_edge_points - 2;
        geidx2 = good_edge_points - 1;
#endif
      }
    }

    //logger->log_debug(name(), "Current best is %zu -> %zu", pidx1, pidx2);

    // in the case we have a backup lower frustrum edge check if we should use it
    // Criteria:
    // 0. we have a backup point
    // 1. no other suitable edge was chosen at all
    // 2. angle(Y_axis, chosen_edge) > threshold
    // 3.. p1.x or p2.x > centroid.x
    if (lf_pidx1 != std::numeric_limits<size_t>::max()) {
      PointType &lp1p = baserel_polygon_cloud->points[lf_pidx1];
      PointType &lp2p = baserel_polygon_cloud->points[lf_pidx2];

      Eigen::Vector4f lp1(lp1p.x, lp1p.y, lp1p.z, 0.);
      Eigen::Vector4f lp2(lp2p.x, lp2p.y, lp2p.z, 0.);

      // None found at all
      if (pidx1 == std::numeric_limits<size_t>::max()) {
        pidx1 = lf_pidx1;
        pidx2 = lf_pidx2;

#ifdef HAVE_VISUAL_DEBUGGING
        good_hull_edges[good_edge_points][0] = cloud_hull_->points[lf_pidx1].x;
        good_hull_edges[good_edge_points][1] = cloud_hull_->points[lf_pidx1].y;
        good_hull_edges[good_edge_points][2] = cloud_hull_->points[lf_pidx1].z;
        geidx1 = good_edge_points++;

        good_hull_edges[good_edge_points][0] = cloud_hull_->points[lf_pidx2].x;
        good_hull_edges[good_edge_points][1] = cloud_hull_->points[lf_pidx2].y;
        good_hull_edges[good_edge_points][2] = cloud_hull_->points[lf_pidx2].z;
        geidx2 = good_edge_points++;
#endif

      } else {

        PointType &p1p = baserel_polygon_cloud->points[pidx1];
        PointType &p2p = baserel_polygon_cloud->points[pidx2];

        Eigen::Vector4f p1(p1p.x, p1p.y, p1p.z, 0.);
        Eigen::Vector4f p2(p2p.x, p2p.y, p2p.z, 0.);

        // Unsuitable "good" line until now?
        if (//(pcl::getAngle3D(p2 - p1, Eigen::Vector4f::UnitZ()) > M_PI * 0.5) ||
            (p1[0] > baserel_table_centroid[0]) || (p2[0] > baserel_table_centroid[0]))
        {
          //logger->log_warn(name(), "Choosing backup candidate!");

          pidx1 = lf_pidx1;
          pidx2 = lf_pidx2;

#ifdef HAVE_VISUAL_DEBUGGING
          good_hull_edges[good_edge_points][0] = cloud_hull_->points[lf_pidx1].x;
          good_hull_edges[good_edge_points][1] = cloud_hull_->points[lf_pidx1].y;
          good_hull_edges[good_edge_points][2] = cloud_hull_->points[lf_pidx1].z;
          geidx1 = good_edge_points++;

          good_hull_edges[good_edge_points][0] = cloud_hull_->points[lf_pidx2].x;
          good_hull_edges[good_edge_points][1] = cloud_hull_->points[lf_pidx2].y;
          good_hull_edges[good_edge_points][2] = cloud_hull_->points[lf_pidx2].z;
          geidx2 = good_edge_points++;
#endif
        }
      }
    }

    //logger->log_info(name(), "Chose %zu -> %zu", pidx1, pidx2);

#ifdef HAVE_VISUAL_DEBUGGING
    if (good_edge_points > 0) {
      good_hull_edges[geidx1][3] = 1.0;
      good_hull_edges[geidx2][3] = 1.0;
    }
    good_hull_edges.resize(good_edge_points);
#endif

    TIMETRACK_END(ttc_find_edge_);

    model_cloud_hull_.reset(new Cloud());
    if ( cfg_table_model_enable_ &&
	 (pidx1 != std::numeric_limits<size_t>::max()) &&
	 (pidx2 != std::numeric_limits<size_t>::max()) )
    {
      TIMETRACK_START(ttc_transform_);

      // Calculate transformation parameters based on determined
      // convex hull polygon segment we decided on as "the table edge"
      PointType &p1p = cloud_hull_->points[pidx1];
      PointType &p2p = cloud_hull_->points[pidx2];

      Eigen::Vector3f p1(p1p.x, p1p.y, p1p.z);
      Eigen::Vector3f p2(p2p.x, p2p.y, p2p.z);

      // Normal vectors for table model and plane
      Eigen::Vector3f model_normal = Eigen::Vector3f::UnitZ();
      Eigen::Vector3f normal(coeff->values[0], coeff->values[1], coeff->values[2]);
      normal.normalize(); // just in case

      Eigen::Vector3f table_centroid_3f =
	Eigen::Vector3f(table_centroid[0], table_centroid[1], table_centroid[2]);

      // Rotational parameters to align table to polygon segment
      Eigen::Vector3f p1_p2 = p2 - p1;
      Eigen::Vector3f p1_p2_center = (p2 + p1) * 0.5;
      p1_p2.normalize();
      Eigen::Vector3f p1_p2_normal_cross = p1_p2.cross(normal);
      p1_p2_normal_cross.normalize();

      // For N=(A,B,C), and hessian Ax+By+Cz+D=0 and N dot X=(Ax+By+Cz)
      // we get N dot X + D = 0 -> -D = N dot X
      double nD = - p1_p2_normal_cross.dot(p1_p2_center);
      double p1_p2_centroid_dist = p1_p2_normal_cross.dot(table_centroid_3f) + nD;
      if (p1_p2_centroid_dist < 0) {
	// normal points to the "wrong" side fo our purpose
	p1_p2_normal_cross *= -1;
      }

      Eigen::Vector3f table_center =
	p1_p2_center + p1_p2_normal_cross * (cfg_table_model_width_ * 0.5);

      for (unsigned int i = 0; i < 3; ++i)  table_centroid[i] = table_center[i];
      table_centroid[3] = 0.;

      // calculate table corner points
      std::vector<Eigen::Vector3f> tpoints(4);
      tpoints[0] = p1_p2_center + p1_p2 * (cfg_table_model_length_ * 0.5);
      tpoints[1] = tpoints[0] + p1_p2_normal_cross * cfg_table_model_width_;
      tpoints[3] = p1_p2_center - p1_p2 * (cfg_table_model_length_ * 0.5);
      tpoints[2] = tpoints[3] + p1_p2_normal_cross * cfg_table_model_width_;

      model_cloud_hull_->points.resize(4);
      model_cloud_hull_->height = 1;
      model_cloud_hull_->width = 4;
      model_cloud_hull_->is_dense = true;
      for (int i = 0; i < 4; ++i) {
	model_cloud_hull_->points[i].x = tpoints[i][0];
	model_cloud_hull_->points[i].y = tpoints[i][1];
	model_cloud_hull_->points[i].z = tpoints[i][2];
      }
      //std::sort(model_cloud_hull_->points.begin(),
      //          model_cloud_hull_->points.end(), comparePoints2D<PointType>);

      // Rotational parameters to rotate table model from camera to
      // determined table position in 3D space
      Eigen::Vector3f rotaxis = model_normal.cross(normal);
      rotaxis.normalize();
      double angle = acos(normal.dot(model_normal));

      // Transformation to translate model from camera center into actual pose
      Eigen::Affine3f affine =
	Eigen::Translation3f(table_centroid.x(), table_centroid.y(),
			     table_centroid.z())
	* Eigen::AngleAxisf(angle, rotaxis);

      Eigen::Vector3f
	model_p1(-cfg_table_model_width_ * 0.5, cfg_table_model_length_ * 0.5, 0.),
	model_p2(-cfg_table_model_width_ * 0.5, -cfg_table_model_length_ * 0.5, 0.);
      model_p1 = affine * model_p1;
      model_p2 = affine * model_p2;

      // Calculate the vector between model_p1 and model_p2
      Eigen::Vector3f model_p1_p2 = model_p2 - model_p1;
      model_p1_p2.normalize();
      // Calculate rotation axis between model_p1 and model_p2
      Eigen::Vector3f model_rotaxis = model_p1_p2.cross(p1_p2);
      model_rotaxis.normalize();
      double angle_p1_p2 = acos(model_p1_p2.dot(p1_p2));
      //logger->log_info(name(), "Angle: %f  Poly (%f,%f,%f) -> (%f,%f,%f)  model (%f,%f,%f) -> (%f,%f,%f)",
      //                 angle_p1_p2, p1.x(), p1.y(), p1.z(), p2.x(), p2.y(), p2.z(),
      //                 model_p1.x(), model_p1.y(), model_p1.z(), model_p2.x(), model_p2.y(), model_p2.z());

      // Final full transformation of the table within the camera coordinate frame
      affine =
	Eigen::Translation3f(table_centroid.x(), table_centroid.y(),
			     table_centroid.z())
	* Eigen::AngleAxisf(angle_p1_p2, model_rotaxis)
	* Eigen::AngleAxisf(angle, rotaxis);


      // Just the rotational part
      Eigen::Quaternionf qt;
      qt = Eigen::AngleAxisf(angle_p1_p2, model_rotaxis)
	* Eigen::AngleAxisf(angle, rotaxis);

      // Set position again, this time with the rotation
      set_position(table_pos_if_, true, table_centroid, qt);

      TIMETRACK_INTER(ttc_transform_, ttc_transform_model_)

	// to show fitted table model
	CloudPtr table_model = generate_table_model(cfg_table_model_length_, cfg_table_model_width_, cfg_table_model_step_);
      pcl::transformPointCloud(*table_model, *table_model_, affine.matrix());
      //*table_model_ = *model_cloud_hull_;
      //*table_model_ = *table_model;
      table_model_->header.frame_id = input_->header.frame_id;

      TIMETRACK_END(ttc_transform_model_);
    }

  } catch (Exception &e) {
    set_position(table_pos_if_, false);
    logger->log_warn(name(), "Failed to transform convex hull cloud, exception follows");
    logger->log_warn(name(), e);
    TIMETRACK_ABORT(ttc_find_edge_);
  }

  TIMETRACK_START(ttc_extract_non_plane_);
  // Extract all non-plane points
  cloud_filt_.reset(new Cloud());
  extract_.setNegative(true);
  extract_.filter(*cloud_filt_);

  TIMETRACK_INTER(ttc_extract_non_plane_, ttc_polygon_filter_);

  // Check if the viewpoint, i.e. the input point clouds frame origin,
  // if above or below the table centroid. If it is above, we want to point
  // the normal towards the viewpoint in the next steps, otherwise it
  // should point away from the sensor. "Above" is relative to the base link
  // frame, i.e. the frame that is based on the ground support plane with the
  // Z axis pointing upwards
  bool viewpoint_above = true;
  try {
    tf::Stamped<tf::Point>
      origin(tf::Point(0, 0, 0), fawkes::Time(0, 0), input_->header.frame_id);
    tf::Stamped<tf::Point> baserel_viewpoint;
    tf_listener->transform_point(cfg_base_frame_, origin, baserel_viewpoint);

    viewpoint_above = (baserel_viewpoint.z() > table_centroid[2]);
  } catch (tf::TransformException &e) {
    logger->log_warn(name(), "[L %u] could not transform viewpoint to base link",
		     loop_count_);
  }

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
    viewpoint_above
    ? (coeff->values[3] > 0 ? pcl::ComparisonOps::GT : pcl::ComparisonOps::LT)
    : (coeff->values[3] < 0 ? pcl::ComparisonOps::GT : pcl::ComparisonOps::LT);
  pcl_utils::PlaneDistanceComparison<PointType>::ConstPtr
    above_comp(new pcl_utils::PlaneDistanceComparison<PointType>(coeff, op));
  pcl::ConditionAnd<PointType>::Ptr
    above_cond(new pcl::ConditionAnd<PointType>());
  above_cond->addComparison(above_comp);
  pcl::ConditionalRemoval<PointType> above_condrem(above_cond);
  above_condrem.setInputCloud(cloud_filt_);
  cloud_above_.reset(new Cloud());
  above_condrem.filter(*cloud_above_);

  //printf("Before: %zu  After: %zu\n", cloud_filt_->points.size(),
  //       cloud_above_->points.size());
  if (cloud_filt_->points.size() < cfg_cluster_min_size_) {
    //logger->log_warn(name(), "Less points than cluster min size");
    TIMETRACK_ABORT(ttc_polygon_filter_);
    TIMETRACK_ABORT(ttc_full_loop_);
    return;
  }

  // Extract only points on the table plane
  pcl::PointIndices::Ptr polygon(new pcl::PointIndices());

  pcl::ConditionAnd<PointType>::Ptr
    polygon_cond(new pcl::ConditionAnd<PointType>());

  pcl_utils::PolygonComparison<PointType>::ConstPtr
    inpoly_comp(new pcl_utils::PolygonComparison<PointType>(
      (model_cloud_hull_ && ! model_cloud_hull_->points.empty()) ? *model_cloud_hull_ : *cloud_hull_));
  polygon_cond->addComparison(inpoly_comp);

  // build the filter
  pcl::ConditionalRemoval<PointType> condrem(polygon_cond);
  condrem.setInputCloud(cloud_above_);
  //condrem.setKeepOrganized(true);
  cloud_objs_.reset(new Cloud());
  condrem.filter(*cloud_objs_);

  //CloudPtr table_points(new Cloud());
  //condrem.setInputCloud(temp_cloud2);
  //condrem.filter(*table_points);

  // CLUSTERS
  // extract clusters of OBJECTS

  TIMETRACK_INTER(ttc_polygon_filter_, ttc_table_to_output_);

  std::vector<int> indices(cloud_proj_->points.size());
  for (uint i = 0; i < indices.size(); i++)
    indices[i] = i;
  ColorCloudPtr tmp_clusters = colorize_cluster(cloud_proj_, indices, table_color);
  tmp_clusters->height = 1;
  tmp_clusters->is_dense = false;
  tmp_clusters->width = cloud_proj_->points.size();

  TIMETRACK_INTER(ttc_table_to_output_, ttc_cluster_objects_);

  unsigned int object_count = 0;
  if (cloud_objs_->points.size() > 0) {
    //TODO: perform statistical outlier removal at this point before clustering.
    //Outlier removal
    pcl::StatisticalOutlierRemoval<PointType> sor;
    sor.setInputCloud(cloud_objs_);
    sor.setMeanK(5);
    sor.setStddevMulThresh(0.2);
    sor.filter(*cloud_objs_);
  }
  //OBJECTS
  std::vector<pcl::PointCloud<ColorPointType>::Ptr> tmp_obj_clusters(MAX_CENTROIDS);
  object_count = cluster_objects(cloud_objs_, tmp_clusters, tmp_obj_clusters);
  if (object_count == 0) {
    logger->log_info(name(), "No clustered points found");
  }

  TIMETRACK_INTER(ttc_hungarian_, ttc_old_centroids_)

  // age all old centroids
  for (OldCentroidVector::iterator it = old_centroids_.begin();
      it != old_centroids_.end(); it++) {
    it->age();
  }
  // delete centroids which are older than cfg_centroid_max_age_
  delete_old_centroids(old_centroids_, cfg_centroid_max_age_);
  // delete old centroids which are too close to current centroids
  delete_near_centroids(centroids_, old_centroids_, cfg_centroid_min_distance_);

  TIMETRACK_END(ttc_old_centroids_);

  // set all pos_ifs not in centroids_ to 'not visible'
  for (unsigned int i = 0; i < pos_ifs_.size(); i++) {
    if (!centroids_.count(i)) {
      set_position(pos_ifs_[i], false);
    }
  }

  // set positions of all visible centroids
  for (CentroidMap::iterator it = centroids_.begin(); it != centroids_.end(); it++) {
    set_position(pos_ifs_[it->first], true, it->second);
  }

  TIMETRACK_INTER(ttc_cluster_objects_, ttc_visualization_)

  *clusters_ = *tmp_clusters;
  fclusters_->header.frame_id = input_->header.frame_id;
  pcl_utils::copy_time(input_, fclusters_);
  pcl_utils::copy_time(input_, ftable_model_);
  pcl_utils::copy_time(input_, fsimplified_polygon_);

  for (unsigned int i = 0; i < f_obj_clusters_.size(); i++) {
    if (centroids_.count(i)) {
      *obj_clusters_[i] = *tmp_obj_clusters[i];
    }
    else {
      obj_clusters_[i]->clear();
      // add point to force update
      // TODO find proper way to update an empty cloud
//      obj_clusters_[i]->push_back(ColorPointType());
    }
    pcl_utils::copy_time(input_, f_obj_clusters_[i]);
  }

#ifdef HAVE_VISUAL_DEBUGGING
  if (visthread_) {
    Eigen::Vector4f normal;
    normal[0] = coeff->values[0];
    normal[1] = coeff->values[1];
    normal[2] = coeff->values[2];
    normal[3] = 0.;

    TabletopVisualizationThreadBase::V_Vector4f hull_vertices;
    hull_vertices.resize(cloud_hull_->points.size());
    for (unsigned int i = 0; i < cloud_hull_->points.size(); ++i) {
      hull_vertices[i][0] = cloud_hull_->points[i].x;
      hull_vertices[i][1] = cloud_hull_->points[i].y;
      hull_vertices[i][2] = cloud_hull_->points[i].z;
      hull_vertices[i][3] = 0.;
    }

    TabletopVisualizationThreadBase::V_Vector4f model_vertices;
    if (model_cloud_hull_ && ! model_cloud_hull_->points.empty()) {
      model_vertices.resize(model_cloud_hull_->points.size());
      for (unsigned int i = 0; i < model_cloud_hull_->points.size(); ++i) {
        model_vertices[i][0] = model_cloud_hull_->points[i].x;
        model_vertices[i][1] = model_cloud_hull_->points[i].y;
        model_vertices[i][2] = model_cloud_hull_->points[i].z;
        model_vertices[i][3] = 0.;
      }
    }

    visthread_->visualize(input_->header.frame_id,
			  table_centroid, normal, hull_vertices, model_vertices,
			  good_hull_edges, centroids_, cylinder_params_, obj_shape_confidence_, best_obj_guess_);
  }
#endif

  TIMETRACK_END(ttc_visualization_);
  TIMETRACK_END(ttc_full_loop_);

#ifdef USE_TIMETRACKER
  if (++tt_loopcount_ >= 5) {
    tt_loopcount_ = 0;
    tt_->print_to_stdout();
  }
#endif
}

std::vector<pcl::PointIndices>
TabletopObjectsThread::extract_object_clusters(CloudConstPtr input) {
  TIMETRACK_START(ttc_obj_extraction_);
  std::vector<pcl::PointIndices> cluster_indices;
  if (input->empty()) {
    TIMETRACK_ABORT(ttc_obj_extraction_);
    return cluster_indices;
  }
  // Creating the KdTree object for the search method of the extraction

       pcl::search::KdTree<PointType>::Ptr
       kdtree_cl(new pcl::search::KdTree<PointType>());
       kdtree_cl->setInputCloud(input);

       pcl::EuclideanClusterExtraction<PointType> ec;
       ec.setClusterTolerance(cfg_cluster_tolerance_);
       ec.setMinClusterSize(cfg_cluster_min_size_);
       ec.setMaxClusterSize(cfg_cluster_max_size_);
       ec.setSearchMethod(kdtree_cl);
       ec.setInputCloud(input);
       ec.extract(cluster_indices);

       //logger->log_debug(name(), "Found %zu clusters", cluster_indices.size());
       TIMETRACK_END(ttc_obj_extraction_);
       return cluster_indices;
}

int TabletopObjectsThread::next_id() {
  if (free_ids_.empty()) {
    logger->log_debug(name(), "free_ids is empty");
    return -1;
  }
  int id = free_ids_.front();
  free_ids_.pop_front();
  return id;
}

unsigned int
TabletopObjectsThread::cluster_objects(CloudConstPtr input_cloud,
    ColorCloudPtr tmp_clusters,
    std::vector<ColorCloudPtr> &tmp_obj_clusters) {
  unsigned int object_count = 0;
  std::vector<pcl::PointIndices> cluster_indices = extract_object_clusters(input_cloud);
  std::vector<pcl::PointIndices>::const_iterator it;
  unsigned int num_points = 0;
  for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    num_points += it->indices.size();

  CentroidMap tmp_centroids;

  if (num_points > 0) {
    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> new_centroids(MAX_CENTROIDS);


    unsigned int centroid_i = 0;

    std::vector<double> init_likelihoods;
    init_likelihoods.resize(NUM_KNOWN_OBJS_ + 1, 0.0);
    for (uint i = 0; i < MAX_CENTROIDS; i++)
      obj_likelihoods_[i] = init_likelihoods;

    for (it = cluster_indices.begin();
        it != cluster_indices.end() && centroid_i < MAX_CENTROIDS;
        ++it, ++centroid_i)
    {

      logger->log_debug(name(), "********************Processing obj_%u********************",
      centroid_i);

      //Centroids in cam frame:
      //pcl::compute3DCentroid(*cloud_objs_, it->indices, centroids[centroid_i]);

      // TODO fix this; we only want to copy the cluster, the color is incorrect
      ColorCloudPtr single_cluster =
          colorize_cluster(input_cloud, it->indices, cluster_colors[centroid_i]);
      single_cluster->header.frame_id = input_cloud->header.frame_id;
      single_cluster->width = it->indices.size();
      single_cluster->height = 1;

      ColorCloudPtr obj_in_base_frame(new ColorCloud());
      obj_in_base_frame->header.frame_id = cfg_base_frame_;
      obj_in_base_frame->width = it->indices.size();
      obj_in_base_frame->height = 1;
      obj_in_base_frame->points.resize(it->indices.size());

      // don't add cluster here since the id is wrong
      //*obj_clusters_[obj_i++] = *single_cluster;

pcl_utils::transform_pointcloud(cfg_base_frame_, *single_cluster,
        *obj_in_base_frame, *tf_listener);

      pcl::compute3DCentroid(*obj_in_base_frame, new_centroids[centroid_i]);

      if (cfg_cylinder_fitting_) {
        new_centroids[centroid_i] = fit_cylinder(obj_in_base_frame,
            new_centroids[centroid_i], centroid_i);
      }


    }
    object_count = centroid_i;
    new_centroids.resize(object_count);


    // save cylinder fitting variables
    // to temporary variables to be able to reassign IDs
    CentroidMap cylinder_params(cylinder_params_);
    std::map<uint, signed int> best_obj_guess(best_obj_guess_);
    std::map<uint, double> obj_shape_confidence(obj_shape_confidence_);
    std::map<uint, std::vector<double> > obj_likelihoods(obj_likelihoods_);
    cylinder_params_.clear();
    best_obj_guess_.clear();
    obj_shape_confidence_.clear();
    obj_likelihoods_.clear();

    std::map<uint, int> assigned_ids;
    if (cfg_track_objects_) {
      assigned_ids = track_objects(new_centroids);
    }
    else { //! cfg_track_objects_
      for (unsigned int i = 0; i < new_centroids.size(); i++) {
        assigned_ids[i] = i;
      }
    }

    // reassign IDs
    for (uint i = 0; i < new_centroids.size(); i++) {
      int assigned_id;
      try {
        assigned_id = assigned_ids.at(i);
      }
      catch (const std::out_of_range& e) {
        logger->log_error(name(), "Object %d was not assigned", i);
        // drop centroid
        assigned_id = -1;
      }
      if (assigned_id == -1)
        continue;
      tmp_centroids[assigned_id] = new_centroids[i];
      cylinder_params_[assigned_id] = cylinder_params[i];
      obj_shape_confidence_[assigned_id] = obj_shape_confidence[i];
      best_obj_guess_[assigned_id] = best_obj_guess[i];
      obj_likelihoods_[assigned_id] = obj_likelihoods[i];
      ColorCloudPtr colorized_cluster = colorize_cluster(input_cloud,
          cluster_indices[i].indices,
          cluster_colors[assigned_id % MAX_CENTROIDS]);
      *tmp_clusters += *colorized_cluster;
      tmp_obj_clusters[assigned_id] = colorized_cluster;

    }

    // remove all centroids too high above the table
    remove_high_centroids(table_centroid, tmp_centroids);

    if (object_count > 0)
      first_run_ = false;
  }
  else {
    logger->log_info(name(), "No clustered points found");
    // save all centroids to old centroids
    for (CentroidMap::iterator it = centroids_.begin(); it != centroids_.end(); it++) {
      old_centroids_.push_back(OldCentroid(it->first, it->second));
    }
  }
  centroids_ = tmp_centroids;
  return object_count;
}

TabletopObjectsThread::ColorCloudPtr TabletopObjectsThread::colorize_cluster (
    CloudConstPtr input_cloud,
    const std::vector<int> &cluster,
    const uint8_t color[]) {
  ColorCloudPtr result(new ColorCloud());
  result->resize(cluster.size());
  result->header.frame_id = input_cloud->header.frame_id;
  uint i = 0;
  for (std::vector<int>::const_iterator it = cluster.begin(); it != cluster.end(); ++it, ++i) {
    ColorPointType &p1 = result->points.at(i);
    const PointType &p2 = input_cloud->points.at(*it);
    p1.x = p2.x;
    p1.y = p2.y;
    p1.z = p2.z;
    p1.r = color[0];
    p1.g = color[1];
    p1.b = color[2];
  }
  return result;
}

bool
TabletopObjectsThread::compute_bounding_box_scores(
    Eigen::Vector3f& cluster_dim,
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> >& scores)
{

  scores.resize(NUM_KNOWN_OBJS_);

  for (int i = 0; i < NUM_KNOWN_OBJS_; i++) {
    scores[i][0] = compute_similarity(cluster_dim[0],
        known_obj_dimensions_[i][0]);
    scores[i][1] = compute_similarity(cluster_dim[1],
        known_obj_dimensions_[i][1]);
    scores[i][2] = compute_similarity(cluster_dim[2],
        known_obj_dimensions_[i][2]);
  }
  return true;
}

double
TabletopObjectsThread::compute_similarity(double d1, double d2)
{
  return exp(-50.0 * ((d1 - d2) * (d1 - d2)));
}

void
TabletopObjectsThread::set_position(fawkes::Position3DInterface *iface,
                                    bool is_visible, const Eigen::Vector4f &centroid,
                                    const Eigen::Quaternionf &attitude)
{
  tf::Stamped<tf::Pose> baserel_pose;
  try{
    tf::Stamped<tf::Pose>
      spose(tf::Pose(tf::Quaternion(attitude.x(), attitude.y(), attitude.z(), attitude.w()),
                     tf::Vector3(centroid[0], centroid[1], centroid[2])),
            fawkes::Time(0, 0), input_->header.frame_id);
    tf_listener->transform_pose(cfg_result_frame_, spose, baserel_pose);
    iface->set_frame(cfg_result_frame_.c_str());
  } catch (Exception &e) {
    is_visible = false;
  }

  int visibility_history = iface->visibility_history();
  if (is_visible) {
    if (visibility_history >= 0) {
      iface->set_visibility_history(visibility_history + 1);
    } else {
      iface->set_visibility_history(1);
    }
    tf::Vector3 &origin = baserel_pose.getOrigin();
    tf::Quaternion quat = baserel_pose.getRotation();
    double translation[3] = { origin.x(), origin.y(), origin.z() };
    double rotation[4] = { quat.x(), quat.y(), quat.z(), quat.w() };
    iface->set_translation(translation);
    iface->set_rotation(rotation);

  } else {
    if (visibility_history <= 0) {
      iface->set_visibility_history(visibility_history - 1);
    } else {
      iface->set_visibility_history(-1);
      double translation[3] = { 0, 0, 0 };
      double rotation[4] = { 0, 0, 0, 1 };
      iface->set_translation(translation);
      iface->set_rotation(rotation);
    }
  }
  iface->write();
}


TabletopObjectsThread::CloudPtr
TabletopObjectsThread::generate_table_model(const float length, const float width,
                                            const float thickness, const float step,
                                            const float max_error)
{
  CloudPtr c(new Cloud());

  const float length_2    = fabs(length)    * 0.5;
  const float width_2     = fabs(width)     * 0.5;
  const float thickness_2 = fabs(thickness) * 0.5;

  // calculate table points
  const unsigned int l_base_num = std::max(2u, (unsigned int)floor(length / step));
  const unsigned int num_w = l_base_num +
    ((length < l_base_num * step) ? 0 : ((length - l_base_num * step) > max_error ? 2 : 1));
  const unsigned int w_base_num = std::max(2u, (unsigned int)floor(width / step));
  const unsigned int num_h = w_base_num +
    ((width < w_base_num * step) ? 0 : ((width - w_base_num * step) > max_error ? 2 : 1));
  const unsigned int t_base_num = std::max(2u, (unsigned int)floor(thickness / step));
  const unsigned int num_t = t_base_num +
    ((thickness < t_base_num * step) ? 0 : ((thickness - t_base_num * step) > max_error ? 2 : 1));

  //logger->log_debug(name(), "Generating table model %fx%fx%f (%ux%ux%u=%u points)",
  //                  length, width, thickness,
  //                  num_w, num_h, num_t, num_t * num_w * num_h);

  c->height = 1;
  c->width = num_t * num_w * num_h;
  c->is_dense = true;
  c->points.resize(num_t * num_w * num_h);

  unsigned int idx = 0;
  for (unsigned int t = 0; t < num_t; ++t) {
    for (unsigned int w = 0; w < num_w; ++w) {
      for (unsigned int h = 0; h < num_h; ++h) {
        PointType &p = c->points[idx++];

        p.x = h * step - width_2;
        if ((h == num_h - 1) && fabs(p.x - width_2) > max_error) p.x = width_2;

        p.y = w * step - length_2;
        if ((w == num_w - 1) && fabs(p.y - length_2) > max_error)  p.y = length_2;

        p.z = t * step - thickness_2;
        if ((t == num_t - 1) && fabs(p.z - thickness_2) > max_error)  p.z = thickness_2;
      }
    }
  }

  return c;
}

TabletopObjectsThread::CloudPtr
TabletopObjectsThread::generate_table_model(const float length, const float width,
                                            const float step, const float max_error)
{
  CloudPtr c(new Cloud());

  const float length_2     = fabs(length)     * 0.5;
  const float width_2    = fabs(width)    * 0.5;

  // calculate table points
  const unsigned int l_base_num = std::max(2u, (unsigned int)floor(length / step));
  const unsigned int num_w = l_base_num +
    ((length < l_base_num * step) ? 0 : ((length - l_base_num * step) > max_error ? 2 : 1));
  const unsigned int w_base_num = std::max(2u, (unsigned int)floor(width / step));
  const unsigned int num_h = w_base_num +
    ((width < w_base_num * step) ? 0 : ((width - w_base_num * step) > max_error ? 2 : 1));

  //logger->log_debug(name(), "Generating table model %fx%f (%ux%u=%u points)",
  //                  length, width, num_w, num_h, num_w * num_h);

  c->height = 1;
  c->width = num_w * num_h;
  c->is_dense = true;
  c->points.resize(num_w * num_h);

  unsigned int idx = 0;
  for (unsigned int w = 0; w < num_w; ++w) {
    for (unsigned int h = 0; h < num_h; ++h) {
      PointType &p = c->points[idx++];

      p.x = h * step - width_2;
      if ((h == num_h - 1) && fabs(p.x - width_2) > max_error) p.x = width_2;

      p.y = w * step - length_2;
      if ((w == num_w - 1) && fabs(p.y - length_2) > max_error)  p.y = length_2;

      p.z = 0.;
    }
  }

  return c;
}


TabletopObjectsThread::CloudPtr
TabletopObjectsThread::simplify_polygon(CloudPtr polygon, float dist_threshold)
{
  const float sqr_dist_threshold = dist_threshold * dist_threshold;
  CloudPtr result(new Cloud());
  const size_t psize = polygon->points.size();
  result->points.resize(psize);
  size_t res_points = 0;
  size_t i_dist = 1;
  for (size_t i = 1; i <= psize; ++i) {
    PointType &p1p = polygon->points[i - i_dist        ];

    if (i == psize) {
      if (result->points.empty()) {
        // Simplification failed, got something too "line-ish"
        return polygon;
      }
    }

    PointType &p2p = polygon->points[i % psize];
    PointType &p3p = (i == psize) ? result->points[0] : polygon->points[(i + 1) % psize];

    Eigen::Vector4f p1(p1p.x, p1p.y, p1p.z, 0);
    Eigen::Vector4f p2(p2p.x, p2p.y, p2p.z, 0);
    Eigen::Vector4f p3(p3p.x, p3p.y, p3p.z, 0);

    Eigen::Vector4f line_dir = p3 - p1;

    double sqr_dist = pcl::sqrPointToLineDistance(p2, p1, line_dir);
    if (sqr_dist < sqr_dist_threshold) {
      ++i_dist;
    } else {
      i_dist = 1;
      result->points[res_points++] = p2p;
    }
  }

  result->header.frame_id = polygon->header.frame_id;
  result->header.stamp = polygon->header.stamp;
  result->width  = res_points;
  result->height = 1;
  result->is_dense = false;
  result->points.resize(res_points);

  return result;
}

void
TabletopObjectsThread::convert_colored_input()
{
  converted_input_->header.seq      = colored_input_->header.seq;
  converted_input_->header.frame_id = colored_input_->header.frame_id;
  converted_input_->header.stamp    = colored_input_->header.stamp;
  converted_input_->width           = colored_input_->width;
  converted_input_->height          = colored_input_->height;
  converted_input_->is_dense        = colored_input_->is_dense;

  const size_t size = colored_input_->points.size();
  converted_input_->points.resize(size);
  for (size_t i = 0; i < size; ++i) {
    const ColorPointType &in = colored_input_->points[i];
    PointType &out           = converted_input_->points[i];

    out.x = in.x;
    out.y = in.y;
    out.z = in.z;
  }
}

void TabletopObjectsThread::delete_old_centroids(OldCentroidVector centroids,
  unsigned int age)
{
  centroids.erase(
      std::remove_if(
          centroids.begin(),
          centroids.end(),
          [&](const OldCentroid &centroid)->bool {
            if (centroid.getAge() > age) {
              free_ids_.push_back(centroid.getId());
              return true;
            }
            return false;
          }), centroids.end());
}

void TabletopObjectsThread::delete_near_centroids(CentroidMap reference,
  OldCentroidVector centroids, float min_distance)
{
  centroids.erase(
      std::remove_if(
          centroids.begin(),
          centroids.end(),
          [&](const OldCentroid &old)->bool {
            for (CentroidMap::const_iterator it = reference.begin(); it != reference.end(); it++) {
              if (pcl::distances::l2(it->second, old.getCentroid()) < min_distance) {
                free_ids_.push_back(old.getId());
                return true;
              }
            }
            return false;
          }), centroids.end());
}

void
TabletopObjectsThread::remove_high_centroids(Eigen::Vector4f table_centroid,
  CentroidMap centroids) {
  tf::Stamped<tf::Point> sp_baserel_table;
  tf::Stamped<tf::Point> sp_table(
      tf::Point(table_centroid[0], table_centroid[1], table_centroid[2]),
      fawkes::Time(0, 0), input_->header.frame_id);
  try {
    tf_listener->transform_point(cfg_base_frame_, sp_table, sp_baserel_table);
    for (CentroidMap::iterator it = centroids.begin(); it != centroids.end();) {
      try {
        tf::Stamped<tf::Point> sp_baserel_centroid(
            tf::Point(it->second[0], it->second[1], it->second[2]),
            fawkes::Time(0, 0), cfg_base_frame_);
        float d = sp_baserel_centroid.z() - sp_baserel_table.z();
        if (d > cfg_centroid_max_height_) {
          //logger->log_debug(name(), "remove centroid %u, too high (d=%f)", it->first, d);
          free_ids_.push_back(it->first);
          centroids.erase(it++);
        } else
          it++;
      } catch (tf::TransformException &e) {
        // simply keep the centroid if we can't transform it
        it++;
      }
    }
  } catch (tf::TransformException &e) {
    // keep all centroids if transformation of the table fails
  }
}

Eigen::Vector4f TabletopObjectsThread::fit_cylinder(
  ColorCloudConstPtr obj_in_base_frame, Eigen::Vector4f const &centroid,
  uint const &centroid_i)
{
  Eigen::Vector4f new_centroid(centroid);
  ColorPointType pnt_min, pnt_max;
  Eigen::Vector3f obj_dim;
  std::vector < Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>
      > obj_size_scores;
  pcl::getMinMax3D(*obj_in_base_frame, pnt_min, pnt_max);
  obj_dim[0] = fabs(pnt_max.x - pnt_min.x);
  obj_dim[1] = fabs(pnt_max.y - pnt_min.y);
  obj_dim[2] = fabs(pnt_max.z - pnt_min.z);
  compute_bounding_box_scores(obj_dim, obj_size_scores);

  logger->log_debug(name(), "Computed object dimensions: %f %f %f", obj_dim[0],
      obj_dim[1], obj_dim[2]);
  logger->log_debug(name(), "Size similarity to known objects:");
  for (int os = 0; os < NUM_KNOWN_OBJS_; os++) {
    logger->log_debug(name(), "** Cup %i: %f in x, %f in y, %f in z.", os,
        obj_size_scores[os][0], obj_size_scores[os][1], obj_size_scores[os][2]);
    obj_likelihoods_[centroid_i][os] = obj_size_scores[os][0]
        * obj_size_scores[os][1] * obj_size_scores[os][2];
  }

  //Fit cylinder:
  pcl::NormalEstimation < ColorPointType, pcl::Normal > ne;
  pcl::SACSegmentationFromNormals < ColorPointType, pcl::Normal > seg;
  pcl::ExtractIndices < ColorPointType > extract;
  pcl::ExtractIndices < pcl::Normal > extract_normals;
  pcl::PointCloud<pcl::Normal>::Ptr obj_normals(
      new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<ColorPointType>::Ptr tree_cyl(
      new pcl::search::KdTree<ColorPointType>());
  pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);

  // Estimate point normals
  ne.setSearchMethod(tree_cyl);
  ne.setInputCloud(obj_in_base_frame);
  ne.setKSearch(10);
  ne.compute(*obj_normals);

  ///////////////////////////////////////////////////////////////
  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_CYLINDER);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight(0.1);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.07);
  seg.setRadiusLimits(0, 0.12);

  seg.setInputCloud(obj_in_base_frame);
  seg.setInputNormals(obj_normals);

  // Obtain the cylinder inliers and coefficients

  seg.segment(*inliers_cylinder, *coefficients_cylinder);
  //std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;
  //Getting max and min z values from cylinder inliers.
  extract.setInputCloud(obj_in_base_frame);
  extract.setIndices(inliers_cylinder);
  extract.setNegative(false);
  pcl::PointCloud<ColorPointType>::Ptr cloud_cylinder_baserel(
      new pcl::PointCloud<ColorPointType>());
  extract.filter(*cloud_cylinder_baserel);

  cylinder_params_[centroid_i][0] = 0;
  cylinder_params_[centroid_i][1] = 0;
  if (cloud_cylinder_baserel->points.empty()) {
    logger->log_debug(name(), "No cylinder inliers!!");
    obj_shape_confidence_[centroid_i] = 0.0;
  }
  else {

    if (!tf_listener->frame_exists(cloud_cylinder_baserel->header.frame_id)) {
      return centroid;
    }

    obj_shape_confidence_[centroid_i] = (double) (cloud_cylinder_baserel->points
      .size()) / (obj_in_base_frame->points.size() * 1.0);
    logger->log_debug(name(), "Cylinder fit confidence = %zu/%zu = %f",
        cloud_cylinder_baserel->points.size(), obj_in_base_frame->points.size(),
        obj_shape_confidence_[centroid_i]);

    ColorPointType pnt_min;
    ColorPointType pnt_max;
    pcl::getMinMax3D(*cloud_cylinder_baserel, pnt_min, pnt_max);
    if (cfg_verbose_cylinder_fitting_) {
      logger->log_debug(name(),
          "Cylinder height according to cylinder inliers: %f",
          pnt_max.z - pnt_min.z);
      logger->log_debug(name(), "Cylinder height according to bounding box: %f",
          obj_dim[2]);
      logger->log_debug(name(),
          "Cylinder radius according to cylinder fitting: %f",
          (*coefficients_cylinder).values[6]);
      logger->log_debug(name(),
          "Cylinder radius according to bounding box y: %f", obj_dim[1] / 2);
    }
    //Cylinder radius:
    //cylinder_params_[centroid_i][0] = (*coefficients_cylinder).values[6];
    cylinder_params_[centroid_i][0] = obj_dim[1] / 2;
    //Cylinder height:
    //cylinder_params_[centroid_i][1] = (pnt_max->z - pnt_min->z);
    cylinder_params_[centroid_i][1] = obj_dim[2];

    //cylinder_params_[centroid_i][2] = table_inclination_;

    //Overriding computed centroids with estimated cylinder center:
    new_centroid[0] = pnt_min.x + 0.5 * (pnt_max.x - pnt_min.x);
    new_centroid[1] = pnt_min.y + 0.5 * (pnt_max.y - pnt_min.y);
    new_centroid[2] = pnt_min.z + 0.5 * (pnt_max.z - pnt_min.z);
  }

  signed int detected_obj_id = -1;
  double best_confidence = 0.0;
  if (cfg_verbose_cylinder_fitting_) {
    logger->log_debug(name(), "Shape similarity = %f",
        obj_shape_confidence_[centroid_i]);
  }
  for (int os = 0; os < NUM_KNOWN_OBJS_; os++) {
    if (cfg_verbose_cylinder_fitting_) {
      logger->log_debug(name(), "** Similarity to known cup %i:", os);
      logger->log_debug(name(), "Size similarity  = %f",
          obj_likelihoods_[centroid_i][os]);
      obj_likelihoods_[centroid_i][os] =
        (0.6 * obj_likelihoods_[centroid_i][os])
            + (0.4 * obj_shape_confidence_[centroid_i]);
      logger->log_debug(name(), "Overall similarity = %f",
          obj_likelihoods_[centroid_i][os]);
    }
    if (obj_likelihoods_[centroid_i][os] > best_confidence) {
      best_confidence = obj_likelihoods_[centroid_i][os];
      detected_obj_id = os;
    }
  }
  if (cfg_verbose_cylinder_fitting_) {
    logger->log_debug(name(),
        "********************Object Result********************");
  }
  if (best_confidence > 0.6) {
    best_obj_guess_[centroid_i] = detected_obj_id;

    if (cfg_verbose_cylinder_fitting_) {
      logger->log_debug(name(),
          "MATCH FOUND!! -------------------------> Cup number %i",
          detected_obj_id);
    }
  }
  else {
    best_obj_guess_[centroid_i] = -1;
    if (cfg_verbose_cylinder_fitting_) {
      logger->log_debug(name(), "No match found.");
    }
  }
  if (cfg_verbose_cylinder_fitting_) {
    logger->log_debug(name(),
        "*****************************************************");
  }

  return new_centroid;
}

/**
 * calculate reassignment of IDs using the hungarian mehtod such that the total
 * distance all centroids moved is minimal
 * @param new_centroids current centroids which need correct ID assignment
 * @return map containing the new assignments, new_centroid -> ID
 * will be -1 if object is dropped, it's the caller's duty to remove any
 * reference to the centroid
 */
std::map<unsigned int, int>
TabletopObjectsThread::track_objects(
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> new_centroids)
{
  std::map<uint, int> final_assignment;
  if (first_run_) {
    // get a new id for every object since we didn't have objects before
    for (unsigned int i = 0; i < new_centroids.size(); i++) {
      final_assignment[i] = next_id();
    }
    return final_assignment;
  }
  else { // !first_run_
    TIMETRACK_START(ttc_hungarian_);
    hungarian_problem_t hp;
    // obj_ids: the id of the centroid in column i is saved in obj_ids[i]
    std::vector<unsigned int> obj_ids(centroids_.size());
    // create cost matrix,
    // save new centroids in rows, last centroids in columns
    // distance between new centroid i and last centroid j in cost[i][j]
    hp.num_rows = new_centroids.size();
    hp.num_cols = centroids_.size();
    hp.cost = (int**) calloc(hp.num_rows, sizeof(int*));
    for (int i = 0; i < hp.num_rows; i++)
      hp.cost[i] = (int*) calloc(hp.num_cols, sizeof(int));
    for (int row = 0; row < hp.num_rows; row++) { // new centroids
      unsigned int col = 0;
      for (CentroidMap::iterator col_it = centroids_.begin();
          col_it != centroids_.end(); col_it++, col++) { // old centroids
        double distance = pcl::distances::l2(new_centroids[row],
            col_it->second);
        hp.cost[row][col] = (int) (distance * 1000);
        obj_ids[col] = col_it->first;
      }
    }
    HungarianMethod solver;
    solver.init(hp.cost, hp.num_rows, hp.num_cols,
        HUNGARIAN_MODE_MINIMIZE_COST);
    solver.solve();
    // get assignments
    int assignment_size;
    int *assignment = solver.get_assignment(assignment_size);
    int id;
    for (int row = 0; row < assignment_size; row++) {
      if (row >= hp.num_rows) { // object has disappeared
        id = obj_ids.at(assignment[row]);
        old_centroids_.push_back(OldCentroid(id, centroids_.at(id)));
        continue;
      }
      else if (assignment[row] >= hp.num_cols) { // object is new or has reappeared
        bool assigned = false;
        // first, check if there is an old centroid close enough
        for (OldCentroidVector::iterator it = old_centroids_.begin();
            it != old_centroids_.end(); it++) {
          if (pcl::distances::l2(new_centroids[row], it->getCentroid())
              <= cfg_centroid_max_distance_) {
            id = it->getId();
            old_centroids_.erase(it);
            assigned = true;
            break;
          }
        }
        if (!assigned) {
          // we still don't have an id, create as new object
          id = next_id();
        }
      }
      else { // object has been assigned to an existing id
        id = obj_ids[assignment[row]];
        // check if centroid was moved further than cfg_centroid_max_distance_
        // this can happen if a centroid appears and another one disappears in the same loop
        // (then, the old centroid is assigned to the new one)
        if (pcl::distances::l2(centroids_[id], new_centroids[row])
            > cfg_centroid_max_distance_) {
          // save the centroid because we don't use it now
          old_centroids_.push_back(OldCentroid(id, centroids_[id]));
          id = -1;
        }
      }
      final_assignment[row] = id;
    }
    return final_assignment;
  }
}

#ifdef HAVE_VISUAL_DEBUGGING
void
TabletopObjectsThread::set_visualization_thread(TabletopVisualizationThreadBase *visthread)
{
  visthread_ = visthread;
}
#endif

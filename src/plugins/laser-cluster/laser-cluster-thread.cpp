
/***************************************************************************
 *  laser-cluster-thread.cpp - Thread to detect a cluster in 2D laser data
 *
 *  Created: Sun Apr 21 01:27:10 2013
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

#include "laser-cluster-thread.h"
#include "cluster_colors.h"

#include <pcl_utils/utils.h>
#include <pcl_utils/comparisons.h>
#include <utils/time/wait.h>
#include <utils/math/angle.h>
#include <baseapp/run.h>
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
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>

#include <interfaces/Position3DInterface.h>
#include <interfaces/SwitchInterface.h>
#include <interfaces/LaserClusterInterface.h>

#include <iostream>
#include <limits>
#include <cmath>

using namespace std;

/** @class LaserClusterThread "laser-cluster-thread.h"
 * Main thread of laser-cluster plugin.
 * @author Tim Niemueller
 */

using namespace fawkes;


/** Constructor.
 * @param cfg_name configuration name
 * @param cfg_prefix configuration path prefix
 */
LaserClusterThread::LaserClusterThread(std::string &cfg_name, std::string &cfg_prefix)
  : Thread("LaserClusterThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS),
    TransformAspect(TransformAspect::ONLY_LISTENER)
{
  set_name("LaserClusterThread(%s)", cfg_name.c_str());
  cfg_name_   = cfg_name;
  cfg_prefix_ = cfg_prefix;
}


/** Destructor. */
LaserClusterThread::~LaserClusterThread()
{
}


void
LaserClusterThread::init()
{
  cfg_line_removal_ = config->get_bool(cfg_prefix_+"line_removal/enable");
  if (cfg_line_removal_) {
    cfg_segm_max_iterations_ =
      config->get_uint(cfg_prefix_+"line_removal/segmentation_max_iterations");
    cfg_segm_distance_threshold_ =
      config->get_float(cfg_prefix_+"line_removal/segmentation_distance_threshold");
    cfg_segm_min_inliers_ =
      config->get_uint(cfg_prefix_+"line_removal/segmentation_min_inliers");
    cfg_segm_sample_max_dist_ =
      config->get_float(cfg_prefix_+"line_removal/segmentation_sample_max_dist");
    cfg_line_min_length_ =
      config->get_float(cfg_prefix_+"line_removal/min_length");
  }
  cfg_switch_tolerance_      = config->get_float(cfg_prefix_+"switch_tolerance");
  cfg_cluster_tolerance_     = config->get_float(cfg_prefix_+"clustering/tolerance");
  cfg_cluster_min_size_      = config->get_uint(cfg_prefix_+"clustering/min_size");
  cfg_cluster_max_size_      = config->get_uint(cfg_prefix_+"clustering/max_size");
  cfg_input_pcl_             = config->get_string(cfg_prefix_+"input_cloud");
  cfg_result_frame_          = config->get_string(cfg_prefix_+"result_frame");

  cfg_use_bbox_   = false;
  cfg_bbox_min_x_ = 0.0;
  cfg_bbox_max_x_ = 0.0;
  try {
    cfg_bbox_min_x_   = config->get_float(cfg_prefix_+"bounding-box/min_x");
    cfg_bbox_max_x_   = config->get_float(cfg_prefix_+"bounding-box/max_x");
    cfg_bbox_min_y_   = config->get_float(cfg_prefix_+"bounding-box/min_y");
    cfg_bbox_max_y_   = config->get_float(cfg_prefix_+"bounding-box/max_y");
    cfg_use_bbox_     = true;
  } catch (Exception &e) {} // ignored, use default
  cfg_offset_x_               = config->get_float(cfg_prefix_+"offsets/x");
  cfg_offset_y_               = config->get_float(cfg_prefix_+"offsets/y");
  cfg_offset_z_               = config->get_float(cfg_prefix_+"offsets/z");
  cfg_max_num_clusters_      = config->get_uint(cfg_prefix_+"max_num_clusters");

  cfg_selection_mode_ = SELECT_MIN_ANGLE;
  try {
    std::string selmode = config->get_string(cfg_prefix_+"cluster_selection_mode");
    if (selmode == "min-angle") {
      cfg_selection_mode_ = SELECT_MIN_ANGLE;
    } else if (selmode == "min-dist") {
      cfg_selection_mode_ = SELECT_MIN_DIST;
    } else {
      logger->log_warn(name(), "Invalid selection mode, using min angle");
    }
  } catch (Exception &e) {} // ignored, use default

  current_max_x_ = cfg_bbox_max_x_;

  finput_ = pcl_manager->get_pointcloud<PointType>(cfg_input_pcl_.c_str());
  input_ = pcl_utils::cloudptr_from_refptr(finput_);

  try {
    double rotation[4] = {0., 0., 0., 1.};
    cluster_pos_ifs_.resize(cfg_max_num_clusters_, NULL);
    for (unsigned int i = 0; i < cfg_max_num_clusters_; ++i) {
      cluster_pos_ifs_[i] =
	blackboard->open_for_writing_f<Position3DInterface>("/laser-cluster/%s/%u",
							      cfg_name_.c_str(), i + 1);
      cluster_pos_ifs_[i]->set_rotation(rotation);
      cluster_pos_ifs_[i]->write();
    }

    switch_if_ = NULL;
    switch_if_ =
      blackboard->open_for_writing_f<SwitchInterface>("/laser-cluster/%s", cfg_name_.c_str());

    config_if_ = NULL;
    config_if_ = blackboard->open_for_writing_f<LaserClusterInterface>("/laser-cluster/%s",
								       cfg_name_.c_str());

    config_if_->set_max_x(current_max_x_);
    if (cfg_selection_mode_ == SELECT_MIN_DIST) {
      config_if_->set_selection_mode(LaserClusterInterface::SELMODE_MIN_DIST);
    } else {
      config_if_->set_selection_mode(LaserClusterInterface::SELMODE_MIN_ANGLE);
    }
    config_if_->write();

    bool autostart = true;
    try {
      autostart = config->get_bool(cfg_prefix_+"auto-start");
    } catch (Exception &e) {} // ignored, use default
    switch_if_->set_enabled(autostart);
    switch_if_->write();
  } catch (Exception &e) {
    for (size_t i = 0; i < cluster_pos_ifs_.size(); ++i) {
      blackboard->close(cluster_pos_ifs_[i]);
    }
    blackboard->close(switch_if_);
    blackboard->close(config_if_);
    throw;
  }

  fclusters_ = new pcl::PointCloud<ColorPointType>();
  fclusters_->header.frame_id = finput_->header.frame_id;
  fclusters_->is_dense = false;
  char *output_cluster_name;
  if (asprintf(&output_cluster_name, "/laser-cluster/%s", cfg_name_.c_str()) != -1) {
    output_cluster_name_ = output_cluster_name;
    free(output_cluster_name);
    pcl_manager->add_pointcloud<ColorPointType>(output_cluster_name_.c_str(), fclusters_);
  }
  clusters_ = pcl_utils::cloudptr_from_refptr(fclusters_);

  fclusters_labeled_ = new pcl::PointCloud<LabelPointType>();
  fclusters_labeled_->header.frame_id = finput_->header.frame_id;
  fclusters_labeled_->is_dense = false;
  char *output_cluster_labeled_name;
  if (asprintf(&output_cluster_labeled_name,
	       "/laser-cluster/%s-labeled", cfg_name_.c_str()) != -1) {
    output_cluster_labeled_name_ = output_cluster_labeled_name;
    free(output_cluster_labeled_name);
    pcl_manager->add_pointcloud<LabelPointType>(output_cluster_labeled_name_.c_str(),
						fclusters_labeled_);
  }
  clusters_labeled_ = pcl_utils::cloudptr_from_refptr(fclusters_labeled_);

  seg_.setOptimizeCoefficients(true);
  seg_.setModelType(pcl::SACMODEL_LINE);
  seg_.setMethodType(pcl::SAC_RANSAC);
  seg_.setMaxIterations(cfg_segm_max_iterations_);
  seg_.setDistanceThreshold(cfg_segm_distance_threshold_);

  loop_count_ = 0;

#ifdef USE_TIMETRACKER
  tt_ = new TimeTracker();
  tt_loopcount_ = 0;
  ttc_full_loop_          = tt_->add_class("Full Loop");
  ttc_msgproc_            = tt_->add_class("Message Processing");
  ttc_extract_lines_      = tt_->add_class("Line Extraction");
  ttc_clustering_         = tt_->add_class("Clustering");
#endif
}


void
LaserClusterThread::finalize()
{
  input_.reset();
  clusters_.reset();
  clusters_labeled_.reset();

  pcl_manager->remove_pointcloud(output_cluster_name_.c_str());
  
  for (size_t i = 0; i < cluster_pos_ifs_.size(); ++i) {
    blackboard->close(cluster_pos_ifs_[i]);
  }
  blackboard->close(switch_if_);
  blackboard->close(config_if_);

  finput_.reset();
  fclusters_.reset();
  fclusters_labeled_.reset();
}

void
LaserClusterThread::loop()
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

  while (! config_if_->msgq_empty()) {
    if (LaserClusterInterface::SetMaxXMessage *msg = config_if_->msgq_first_safe(msg))
    {
      if (msg->max_x() < 0.0) {
	logger->log_info(name(), "Got cluster max x less than zero, setting config default %f",
			 cfg_bbox_max_x_);
	current_max_x_ = cfg_bbox_max_x_;
      } else {
	current_max_x_ = msg->max_x();
      }
    }

    else if (LaserClusterInterface::SetSelectionModeMessage *msg =
	     config_if_->msgq_first_safe(msg))
    {
      if (msg->selection_mode() == LaserClusterInterface::SELMODE_MIN_DIST) {
	cfg_selection_mode_ = SELECT_MIN_DIST;
      } else if (msg->selection_mode() == LaserClusterInterface::SELMODE_MIN_ANGLE) {
	cfg_selection_mode_ = SELECT_MIN_ANGLE;
      } else {
	logger->log_error(name(), "Unknown cluster selection mode received, ignoring");
      }
      config_if_->set_selection_mode(msg->selection_mode());
      config_if_->write();
    }

    config_if_->msgq_pop();
  }

  if (! switch_if_->is_enabled()) {
    //TimeWait::wait(250000);
    for (unsigned int i = 0; i < cfg_max_num_clusters_; ++i) {
      set_position(cluster_pos_ifs_[i], false);
    }
    return;
  }

  TIMETRACK_INTER(ttc_msgproc_, ttc_extract_lines_);


  if (input_->points.size() <= 10) {
    // this can happen if run at startup. Since tabletop threads runs continuous
    // and not synchronized with main loop, but point cloud acquisition thread is
    // synchronized, we might start before any data has been read
    //logger->log_warn(name(), "Empty voxelized point cloud, omitting loop");
    //TimeWait::wait(50000);
    return;
  }

  CloudPtr noline_cloud(new Cloud());

  // Erase non-finite points
  pcl::PassThrough<PointType> passthrough;
  if (current_max_x_ > 0.) {
    passthrough.setFilterFieldName("x");
    passthrough.setFilterLimits(cfg_bbox_min_x_, current_max_x_);
  }
  passthrough.setInputCloud(input_);
  passthrough.filter(*noline_cloud);

  //logger->log_info(name(), "[L %u] total: %zu   finite: %zu",
  //		     loop_count_, input_->points.size(), noline_cloud->points.size());

  pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

  if (cfg_line_removal_) {
    std::list<CloudPtr> restore_pcls;

    while (noline_cloud->points.size () > cfg_cluster_min_size_) {
      // Segment the largest planar component from the remaining cloud
      //logger->log_info(name(), "[L %u] %zu points left",
      //	               loop_count_, noline_cloud->points.size());
      
      pcl::search::KdTree<PointType>::Ptr
	search(new pcl::search::KdTree<PointType>);
      search->setInputCloud(noline_cloud);
      seg_.setSamplesMaxDist(cfg_segm_sample_max_dist_, search); 
      seg_.setInputCloud(noline_cloud);
      seg_.segment(*inliers, *coeff);
      if (inliers->indices.size () == 0) {
	// no line found
	break;
      }

      // check for a minimum number of expected inliers
      if ((double)inliers->indices.size() < cfg_segm_min_inliers_) {
	//logger->log_warn(name(), "[L %u] no more lines (%zu inliers, required %u)",
	//		   loop_count_, inliers->indices.size(), cfg_segm_min_inliers_);
	break;
      }

      float length = calc_line_length(noline_cloud, inliers, coeff);

      if (length < cfg_line_min_length_) {
	// we must remove the points for now to continue filtering,
	// but must restore them later
	// Remove the linear inliers, extract the rest
	CloudPtr cloud_line(new Cloud());
	pcl::ExtractIndices<PointType> extract;
	extract.setInputCloud(noline_cloud);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*cloud_line);
	restore_pcls.push_back(cloud_line);

      }

      // Remove the linear inliers, extract the rest
      CloudPtr cloud_f(new Cloud());
      pcl::ExtractIndices<PointType> extract;
      extract.setInputCloud(noline_cloud);
      extract.setIndices(inliers);
      extract.setNegative(true);
      extract.filter(*cloud_f);
      *noline_cloud = *cloud_f;
    }

    for (CloudPtr cloud : restore_pcls) {
      *noline_cloud += *cloud;
    }
  }

  {
    CloudPtr tmp_cloud(new Cloud());
    // Erase non-finite points
    pcl::PassThrough<PointType> passthrough;
    passthrough.setInputCloud(noline_cloud);
    passthrough.filter(*tmp_cloud);

    if (noline_cloud->points.size() != tmp_cloud->points.size()) {
      //logger->log_error(name(), "[L %u] new non-finite points total: %zu   finite: %zu",
      //	          loop_count_, noline_cloud->points.size(), tmp_cloud->points.size());
      *noline_cloud = *tmp_cloud;
    }
  }

  // What remains in the cloud are now potential clusters

  TIMETRACK_INTER(ttc_extract_lines_, ttc_clustering_);

  clusters_->points.resize(noline_cloud->points.size());
  clusters_->height = 1;
  clusters_->width  = noline_cloud->points.size();

  clusters_labeled_->points.resize(noline_cloud->points.size());
  clusters_labeled_->height = 1;
  clusters_labeled_->width  = noline_cloud->points.size();

  // copy points and set to white
  for (size_t p = 0; p < clusters_->points.size(); ++p) {
    ColorPointType &out_point = clusters_->points[p];
    LabelPointType &out_lab_point = clusters_labeled_->points[p];
    PointType &in_point  = noline_cloud->points[p];
    out_point.x = out_lab_point.x = in_point.x;
    out_point.y = out_lab_point.y = in_point.y;
    out_point.z = out_lab_point.z = in_point.z;
    out_point.r = out_point.g = out_point.b = 1.0;
    out_lab_point.label = 0;
  }

  //logger->log_info(name(), "[L %u] remaining: %zu",
  //		   loop_count_, noline_cloud->points.size());

  std::vector<pcl::PointIndices> cluster_indices;
  if (noline_cloud->points.size() > 0) {
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<PointType>::Ptr
      kdtree_cl(new pcl::search::KdTree<PointType>());
    kdtree_cl->setInputCloud(noline_cloud);

    pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance(cfg_cluster_tolerance_);
    ec.setMinClusterSize(cfg_cluster_min_size_);
    ec.setMaxClusterSize(cfg_cluster_max_size_);
    ec.setSearchMethod(kdtree_cl);
    ec.setInputCloud(noline_cloud);
    ec.extract(cluster_indices);

    //logger->log_info(name(), "Found %zu clusters", cluster_indices.size());

    unsigned int i = 0;
    for (auto cluster : cluster_indices) {
      //Eigen::Vector4f centroid;
      //pcl::compute3DCentroid(*noline_cloud, cluster.indices, centroid);

      //logger->log_info(name(), "  Cluster %u with %zu points at (%f, %f, %f)",
      //	         i, cluster.indices.size(), centroid.x(), centroid.y(), centroid.z());

      // color points of cluster
      for (auto ci : cluster.indices) {
	ColorPointType &out_point = clusters_->points[ci];
	out_point.r = ignored_cluster_color[0];
	out_point.g = ignored_cluster_color[1];;
	out_point.b = ignored_cluster_color[2];;
      }

      ++i;
    }

  } else {
    //logger->log_info(name(), "Filter left no points for clustering");
  }


  if (! cluster_indices.empty()) {
    std::vector<ClusterInfo> cinfos;

    for (unsigned int i = 0; i < cluster_indices.size(); ++i) {
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*noline_cloud, cluster_indices[i].indices, centroid);
      if ( !cfg_use_bbox_ ||
	   ((centroid.x() >= cfg_bbox_min_x_) && (centroid.x() <= cfg_bbox_max_x_) &&
	    (centroid.y() >= cfg_bbox_min_y_) && (centroid.y() <= cfg_bbox_max_y_)))
      {
	ClusterInfo info;
	info.angle = std::atan2(centroid.y(), centroid.x());
	info.dist  = centroid.norm();
	info.index = i;
	info.centroid = centroid;
	cinfos.push_back(info);
      } else {
	/*
	logger->log_info(name(), "[L %u] Cluster %u out of bounds (%f,%f) "
			 "not in ((%f,%f),(%f,%f))",
			 loop_count_, centroid.x(), centroid.y(),
			 cfg_bbox_min_x_, cfg_bbox_max_x_,
			 cfg_bbox_min_y_, cfg_bbox_max_y_);
	*/
      }
    }

    if (! cinfos.empty()) {
      if (cfg_selection_mode_ == SELECT_MIN_ANGLE) {
	std::sort(cinfos.begin(), cinfos.end(),
		  [](const ClusterInfo & a, const ClusterInfo & b) -> bool
		  { 
		    return a.angle < b.angle; 
		  });
      } else if (cfg_selection_mode_ == SELECT_MIN_DIST) {
	std::sort(cinfos.begin(), cinfos.end(),
		  [](const ClusterInfo & a, const ClusterInfo & b) -> bool
		  { 
		    return a.dist < b.dist; 
		  });
      } else {
	logger->log_error(name(), "Invalid selection mode, cannot select cluster");
      }	


      unsigned int i;
      for (i = 0; i < std::min(cinfos.size(), (size_t)cfg_max_num_clusters_); ++i) {
	// color points of cluster
	for (auto ci : cluster_indices[cinfos[i].index].indices) {
	  ColorPointType &out_point = clusters_->points[ci];
	  LabelPointType &out_lab_point = clusters_labeled_->points[ci];
	  out_point.r = cluster_colors[i][0];
	  out_point.g = cluster_colors[i][1];;
	  out_point.b = cluster_colors[i][2];;
	  out_lab_point.label = i;
	}

	set_position(cluster_pos_ifs_[i], true, cinfos[i].centroid);	
      }
      for (unsigned int j = i; j < cfg_max_num_clusters_; ++j) {
	set_position(cluster_pos_ifs_[j], false);
      }
    } else {
      //logger->log_warn(name(), "No acceptable cluster found, %zu clusters",
      //	         cluster_indices.size());
      for (unsigned int i = 0; i < cfg_max_num_clusters_; ++i) {
	set_position(cluster_pos_ifs_[i], false);
      }
    }
  } else {
    //logger->log_warn(name(), "No clusters found, %zu remaining points",
    //	     noline_cloud->points.size());
    for (unsigned int i = 0; i < cfg_max_num_clusters_; ++i) {
      set_position(cluster_pos_ifs_[i], false);
    }
  }

  //*clusters_ = *tmp_clusters;
  if (finput_->header.frame_id == "") {
    logger->log_debug(name(), "Empty frame ID");
  }
  fclusters_->header.frame_id = finput_->header.frame_id;
  pcl_utils::copy_time(finput_, fclusters_);
  fclusters_labeled_->header.frame_id = finput_->header.frame_id;
  pcl_utils::copy_time(finput_, fclusters_labeled_);

  TIMETRACK_END(ttc_clustering_);
  TIMETRACK_END(ttc_full_loop_);

#ifdef USE_TIMETRACKER
  if (++tt_loopcount_ >= 5) {
    tt_loopcount_ = 0;
    tt_->print_to_stdout();
  }
#endif
}


void
LaserClusterThread::set_position(fawkes::Position3DInterface *iface,
				 bool is_visible, const Eigen::Vector4f &centroid,
				 const Eigen::Quaternionf &attitude)
{
	tf::Stamped<tf::Pose> baserel_pose;

	if (input_->header.frame_id.empty()) {
		is_visible = false;
	} else {
		try{
			// Note that we add a correction offset to the centroid X value.
			// This offset is determined empirically and just turns out to be helpful
			// in certain situations.

			tf::Stamped<tf::Pose>
				spose(tf::Pose(tf::Quaternion(attitude.x(), attitude.y(), attitude.z(), attitude.w()),
				               tf::Vector3(centroid[0], centroid[1], centroid[2])),
				      fawkes::Time(0, 0), input_->header.frame_id);
			tf_listener->transform_pose(cfg_result_frame_, spose, baserel_pose);
			iface->set_frame(cfg_result_frame_.c_str());
		} catch (tf::TransformException &e) {
			if (fawkes::runtime::uptime() >= tf_listener->get_cache_time()) {
				logger->log_warn(name(),"Transform exception:");
				logger->log_warn(name(),e);
			}
			is_visible = false;
		}
	}

  int visibility_history = iface->visibility_history();
  if (is_visible) {

    tf::Vector3 &origin = baserel_pose.getOrigin();
    tf::Quaternion quat = baserel_pose.getRotation();

    Eigen::Vector4f baserel_centroid;
    baserel_centroid[0] = origin.x();
    baserel_centroid[1] = origin.y();
    baserel_centroid[2] = origin.z();
    baserel_centroid[3] = 0.;
    
    //we have to subtract the previously added offset to be
    //able to compare against the current centroid
    Eigen::Vector4f last_centroid(iface->translation(0) - cfg_offset_x_,
				  iface->translation(1) - cfg_offset_y_,
				  iface->translation(2) - cfg_offset_z_, 0.);
    bool different_cluster =
      fabs((last_centroid - baserel_centroid).norm()) > cfg_switch_tolerance_;

    if (! different_cluster && visibility_history >= 0) {
      iface->set_visibility_history(visibility_history + 1);
    } else {
      iface->set_visibility_history(1);
    }

    //add the offset and publish
    double translation[3] = { origin.x() + cfg_offset_x_,
			      origin.y() + cfg_offset_y_,
			      origin.z() + cfg_offset_z_ };
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


float
LaserClusterThread::calc_line_length(CloudPtr cloud, pcl::PointIndices::Ptr inliers,
				     pcl::ModelCoefficients::Ptr coeff)
{
  if (inliers->indices.size() < 2)  return 0.;

  CloudPtr cloud_line(new Cloud());
  CloudPtr cloud_line_proj(new Cloud());

  pcl::ExtractIndices<PointType> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*cloud_line);

  // Project the model inliers
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

    return (ptv_1 - ptv_2).norm();
  } else {
    return 0.f;
  }
}

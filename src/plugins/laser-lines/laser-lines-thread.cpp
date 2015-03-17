
/***************************************************************************
 *  laser-lines-thread.cpp - Thread to detect lines in 2D laser data
 *
 *  Created: Fri May 23 18:12:14 2014
 *  Copyright  2011-2014  Tim Niemueller [www.niemueller.de]
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

#include "laser-lines-thread.h"
#include "line_func.h"
#include "line_colors.h"

#include <pcl_utils/utils.h>
#include <pcl_utils/comparisons.h>
#include <utils/time/wait.h>
#include <utils/math/angle.h>
#ifdef USE_TIMETRACKER
#  include <utils/time/tracker.h>
#endif
#include <utils/time/tracker_macros.h>
#include <baseapp/run.h>

#include <interfaces/Position3DInterface.h>
#include <interfaces/SwitchInterface.h>
#include <interfaces/LaserLineInterface.h>

#ifdef HAVE_VISUAL_DEBUGGING
#  include <ros/ros.h>
#  include <visualization_msgs/MarkerArray.h>
#endif

#include <iostream>
#include <limits>
#include <cmath>

using namespace std;

#define CFG_PREFIX "/laser-lines/"

/** @class LaserLinesThread "laser-lines-thread.h"
 * Main thread of laser-lines plugin.
 * @author Tim Niemueller
 */

using namespace fawkes;


/** Constructor. */
LaserLinesThread::LaserLinesThread()
  : Thread("LaserLinesThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS),
    TransformAspect(TransformAspect::ONLY_LISTENER)
{
}


/** Destructor. */
LaserLinesThread::~LaserLinesThread()
{
}


void
LaserLinesThread::init()
{
  cfg_segm_max_iterations_ =
    config->get_uint(CFG_PREFIX"line_segmentation_max_iterations");
  cfg_segm_distance_threshold_ =
    config->get_float(CFG_PREFIX"line_segmentation_distance_threshold");
  cfg_segm_sample_max_dist_ =
    config->get_float(CFG_PREFIX"line_segmentation_sample_max_dist");
  cfg_segm_min_inliers_ =
    config->get_uint(CFG_PREFIX"line_segmentation_min_inliers");
  cfg_min_length_ =
    config->get_float(CFG_PREFIX"line_min_length");
  cfg_max_length_ =
    config->get_float(CFG_PREFIX"line_max_length");
  cfg_min_dist_ =
    config->get_float(CFG_PREFIX"line_min_distance");
  cfg_max_dist_ =
    config->get_float(CFG_PREFIX"line_max_distance");
  cfg_cluster_tolerance_ =
    config->get_float(CFG_PREFIX"line_cluster_tolerance");
  cfg_cluster_quota_ =
    config->get_float(CFG_PREFIX"line_cluster_quota");

  cfg_switch_tolerance_ =
    config->get_float(CFG_PREFIX"switch_tolerance");

  cfg_input_pcl_             = config->get_string(CFG_PREFIX"input_cloud");
  cfg_result_frame_          = config->get_string(CFG_PREFIX"result_frame");
  cfg_max_num_lines_         = config->get_uint(CFG_PREFIX"max_num_lines");

  finput_ = pcl_manager->get_pointcloud<PointType>(cfg_input_pcl_.c_str());
  input_ = pcl_utils::cloudptr_from_refptr(finput_);

  try {
    //double rotation[4] = {0., 0., 0., 1.};
    line_ifs_.resize(cfg_max_num_lines_, NULL);
    for (unsigned int i = 0; i < cfg_max_num_lines_; ++i) {
      char *tmp;
      if (asprintf(&tmp, "/laser-lines/%u", i + 1) != -1) {
        // Copy to get memory freed on exception
        std::string id = tmp;
        free(tmp);

	line_ifs_[i] =
	  blackboard->open_for_writing<LaserLineInterface>(id.c_str());
	/*
	line_ifs_[i]->set_rotation(rotation);
	line_ifs_[i]->write();
	*/
      }
    }

    switch_if_ = NULL;
    switch_if_ = blackboard->open_for_writing<SwitchInterface>("laser-lines");

    bool autostart = true;
    try {
      autostart = config->get_bool(CFG_PREFIX"auto-start");
    } catch (Exception &e) {} // ignored, use default
    switch_if_->set_enabled(autostart);
    switch_if_->write();
  } catch (Exception &e) {
    for (size_t i = 0; i < line_ifs_.size(); ++i) {
      blackboard->close(line_ifs_[i]);
    }
    blackboard->close(switch_if_);
    throw;
  }

  flines_ = new pcl::PointCloud<ColorPointType>();
  flines_->header.frame_id = finput_->header.frame_id;
  flines_->is_dense = false;
  pcl_manager->add_pointcloud<ColorPointType>("laser-lines", flines_);
  lines_ = pcl_utils::cloudptr_from_refptr(flines_);

  loop_count_ = 0;

#ifdef USE_TIMETRACKER
  tt_ = new TimeTracker();
  tt_loopcount_ = 0;
  ttc_full_loop_          = tt_->add_class("Full Loop");
  ttc_msgproc_            = tt_->add_class("Message Processing");
  ttc_extract_lines_      = tt_->add_class("Line Extraction");
  ttc_clustering_         = tt_->add_class("Clustering");
#endif
#ifdef HAVE_VISUAL_DEBUGGING
  vispub_ = new ros::Publisher();
  *vispub_ = rosnode->advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100);
  last_id_num_ = 0;
#endif
}


void
LaserLinesThread::finalize()
{
#ifdef HAVE_VISUAL_DEBUGGING
  vispub_->shutdown();
  delete vispub_;
#endif

  input_.reset();
  lines_.reset();

  pcl_manager->remove_pointcloud("laser-lines");
  
  for (size_t i = 0; i < line_ifs_.size(); ++i) {
    blackboard->close(line_ifs_[i]);
  }
  blackboard->close(switch_if_);

  finput_.reset();
  flines_.reset();
}

void
LaserLinesThread::loop()
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
      // line data is now invalid
      for (unsigned int i = 0; i < cfg_max_num_lines_; ++i) {
	line_ifs_[i]->set_visibility_history(0);
	line_ifs_[i]->write();
      }

      switch_if_->set_enabled(false);
      switch_if_->write();
    }

    switch_if_->msgq_pop();
  }

  if (! switch_if_->is_enabled()) {
    //TimeWait::wait(250000);
    return;
  }

  TIMETRACK_INTER(ttc_msgproc_, ttc_extract_lines_);

  if (input_->points.size() <= 10) {
    // this can happen if run at startup. Since thread runs continuous
    // and not synchronized with main loop, but point cloud acquisition thread is
    // synchronized, we might start before any data has been read
    //logger->log_warn(name(), "Empty voxelized point cloud, omitting loop");
    //TimeWait::wait(50000);

    for (unsigned int i = 0; i < cfg_max_num_lines_; ++i) {
      set_line(line_ifs_[i], false);
    }

    return;
  }

  //logger->log_info(name(), "[L %u] total: %zu   finite: %zu",
  //		     loop_count_, input_->points.size(), in_cloud->points.size());

  std::vector<LineInfo> linfos =
    calc_lines<PointType>(input_,
			  cfg_segm_min_inliers_, cfg_segm_max_iterations_,
			  cfg_segm_distance_threshold_, cfg_segm_sample_max_dist_,
			  cfg_cluster_tolerance_, cfg_cluster_quota_,
			  cfg_min_length_, cfg_max_length_, cfg_min_dist_, cfg_max_dist_);


  TIMETRACK_INTER(ttc_extract_lines_, ttc_clustering_);

  size_t num_points = 0;
  for (size_t i = 0; i < linfos.size(); ++i) {
    num_points += linfos[i].cloud->points.size();
  }

  lines_->points.resize(num_points);
  lines_->height = 1;
  lines_->width  = num_points;

  // sort lines by bearing to stabilize IDs
  std::sort(linfos.begin(), linfos.end(),
	    [](const LineInfo &l1, const LineInfo &l2) -> bool
	    {
	      return l1.bearing < l2.bearing;
	    });

  // set line parameters
  size_t oi = 0;
  unsigned int line_if_idx = 0;
  for (size_t i = 0; i < linfos.size(); ++i) {
    const LineInfo &info = linfos[i];

    if (line_if_idx < cfg_max_num_lines_) {
      set_line(line_ifs_[line_if_idx++], true, finput_->header.frame_id, info);
    }

    for (size_t p = 0; p < info.cloud->points.size(); ++p) {
      ColorPointType &out_point = lines_->points[oi++];
      PointType &in_point  = info.cloud->points[p];
      out_point.x = in_point.x;
      out_point.y = in_point.y;
      out_point.z = in_point.z;

      if (i < MAX_LINES) {
	out_point.r = line_colors[i][0];
	out_point.g = line_colors[i][1];
	out_point.b = line_colors[i][2];
      } else {
	out_point.r = out_point.g = out_point.b = 1.0;
      }
    }
  }

  for (unsigned int i = line_if_idx; i < cfg_max_num_lines_; ++i) {
    set_line(line_ifs_[i], false);
  }

#ifdef HAVE_VISUAL_DEBUGGING
  publish_visualization(linfos);
#endif

  //*lines_ = *tmp_lines;
  if (finput_->header.frame_id == "" &&
      fawkes::runtime::uptime() >= tf_listener->get_cache_time())
  {
    logger->log_error(name(), "Empty frame ID");
  }
  flines_->header.frame_id = finput_->header.frame_id;
  pcl_utils::copy_time(finput_, flines_);

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
LaserLinesThread::set_line(fawkes::LaserLineInterface *iface,
			   bool is_visible,
			   const std::string &frame_id,
			   const LineInfo &info)
{
  int visibility_history = iface->visibility_history();
  if (is_visible) {
    Eigen::Vector3f old_point_on_line(iface->point_on_line(0),
				      iface->point_on_line(1),
				      iface->point_on_line(2));
    float diff = (old_point_on_line - info.base_point).norm();

    if (visibility_history >= 0 && (diff <= cfg_switch_tolerance_)) {
      iface->set_visibility_history(visibility_history + 1);
    } else {
      iface->set_visibility_history(1);
    }

    //add the offset and publish
    float if_point_on_line[3] =
      { info.base_point[0], info.base_point[1], info.base_point[2] };
    float if_line_direction[3] =
      { info.line_direction[0], info.line_direction[1], info.line_direction[2] };
    float if_end_point_1[3] =
      { info.end_point_1[0], info.end_point_1[1], info.end_point_1[2] };
    float if_end_point_2[3] =
      { info.end_point_2[0], info.end_point_2[1], info.end_point_2[2] };
    iface->set_point_on_line(if_point_on_line);
    iface->set_line_direction(if_line_direction);
    iface->set_frame_id(frame_id.c_str());
    iface->set_bearing(info.bearing);
    iface->set_length(info.length);
    iface->set_end_point_1(if_end_point_1);
    iface->set_end_point_2(if_end_point_2);
  } else {
    if (visibility_history <= 0) {
      iface->set_visibility_history(visibility_history - 1);
    } else {
      iface->set_visibility_history(-1);
      float zero_vector[3] = { 0, 0, 0 };
      iface->set_point_on_line(zero_vector);
      iface->set_line_direction(zero_vector);
      iface->set_end_point_1(zero_vector);
      iface->set_end_point_2(zero_vector);
      iface->set_bearing(0);
      iface->set_length(0);
      iface->set_frame_id("");
    }
  }
  iface->write();  
}


#ifdef HAVE_VISUAL_DEBUGGING
void
LaserLinesThread::publish_visualization(const std::vector<LineInfo> &linfos)
{
  visualization_msgs::MarkerArray m;
  unsigned int idnum = 0;

  for (size_t i = 0; i < linfos.size(); ++i) {
    const LineInfo &info = linfos[i];
    
    /*
    visualization_msgs::Marker basevec;
    basevec.header.frame_id = finput_->header.frame_id;
    basevec.header.stamp = ros::Time::now();
    basevec.ns = "laser_lines";
    basevec.id = idnum++;
    basevec.type = visualization_msgs::Marker::ARROW;
    basevec.action = visualization_msgs::Marker::ADD;
    basevec.points.resize(2);
    basevec.points[0].x = basevec.points[0].y = basevec.points[0].z = 0.;
    basevec.points[1].x = info.point_on_line[0];
    basevec.points[1].y = info.point_on_line[1];
    basevec.points[1].z = info.point_on_line[2];
    basevec.scale.x = 0.02;
    basevec.scale.y = 0.04;
    basevec.color.r = 1.0;
    basevec.color.g = basevec.color.b = 0.;
    basevec.color.a = 1.0;
    basevec.lifetime = ros::Duration(2, 0);
    m.markers.push_back(basevec);
    */

    visualization_msgs::Marker dirvec;
    dirvec.header.frame_id = finput_->header.frame_id;
    dirvec.header.stamp = ros::Time::now();
    dirvec.ns = "laser_lines";
    dirvec.id = idnum++;
    dirvec.type = visualization_msgs::Marker::ARROW;
    dirvec.action = visualization_msgs::Marker::ADD;
    dirvec.points.resize(2);
    dirvec.points[0].x = info.base_point[0];
    dirvec.points[0].y = info.base_point[1];
    dirvec.points[0].z = info.base_point[2];
    dirvec.points[1].x = info.base_point[0] + info.line_direction[0];
    dirvec.points[1].y = info.base_point[1] + info.line_direction[1];
    dirvec.points[1].z = info.base_point[2] + info.line_direction[2];
    dirvec.scale.x = 0.02;
    dirvec.scale.y = 0.04;
    dirvec.color.r = 0.0;
    dirvec.color.g = 1.0;
    dirvec.color.b = 0.f;
    dirvec.color.a = 1.0;
    dirvec.lifetime = ros::Duration(2, 0);
    m.markers.push_back(dirvec);

    visualization_msgs::Marker testvec;
    testvec.header.frame_id = finput_->header.frame_id;
    testvec.header.stamp = ros::Time::now();
    testvec.ns = "laser_lines";
    testvec.id = idnum++;
    testvec.type = visualization_msgs::Marker::ARROW;
    testvec.action = visualization_msgs::Marker::ADD;
    testvec.points.resize(2);
    testvec.points[0].x = 0; //info.point_on_line[0];
    testvec.points[0].y = 0; //info.point_on_line[1];
    testvec.points[0].z = 0; //info.point_on_line[2];
    testvec.points[1].x = info.base_point[0];
    testvec.points[1].y = info.base_point[1];
    testvec.points[1].z = info.base_point[2];
    testvec.scale.x = 0.02;
    testvec.scale.y = 0.04;
    testvec.color.r = line_colors[i][0] / 255.;
    testvec.color.g = line_colors[i][1] / 255.;
    testvec.color.b = line_colors[i][2] / 255.;
    testvec.color.a = 1.0;
    testvec.lifetime = ros::Duration(2, 0);
    m.markers.push_back(testvec);

    char *tmp;
    if (asprintf(&tmp, "L_%zu", i+1) != -1) {
      // Copy to get memory freed on exception
      std::string id = tmp;
      free(tmp);

      visualization_msgs::Marker text;
      text.header.frame_id = finput_->header.frame_id;
      text.header.stamp = ros::Time::now();
      text.ns = "laser_lines";
      text.id = idnum++;
      text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::Marker::ADD;
      text.pose.position.x = info.base_point[0];
      text.pose.position.y = info.base_point[1];
      text.pose.position.z = info.base_point[2] + .15;
      text.pose.orientation.w = 1.;
      text.scale.z = 0.15;
      text.color.r = text.color.g = text.color.b = 1.0f;
      text.color.a = 1.0;
      text.lifetime = ros::Duration(2, 0);
      text.text = id;
      m.markers.push_back(text);
    }

    if (cfg_min_length_ >= 0. || cfg_max_length_ >= 0.) {
      visualization_msgs::Marker sphere_ep_1;
      sphere_ep_1.header.frame_id = finput_->header.frame_id;
      sphere_ep_1.header.stamp = ros::Time::now();
      sphere_ep_1.ns = "laser_lines";
      sphere_ep_1.id = idnum++;
      sphere_ep_1.type = visualization_msgs::Marker::SPHERE;
      sphere_ep_1.action = visualization_msgs::Marker::ADD;
      sphere_ep_1.pose.position.x = info.end_point_1[0];
      sphere_ep_1.pose.position.y = info.end_point_1[1];
      sphere_ep_1.pose.position.z = info.end_point_1[2];
      sphere_ep_1.pose.orientation.w = 1.;
      sphere_ep_1.scale.x = 0.05;
      sphere_ep_1.scale.y = 0.05;
      sphere_ep_1.scale.z = 0.05;
      sphere_ep_1.color.r = line_colors[i][0] / 255.;
      sphere_ep_1.color.g = line_colors[i][1] / 255.;
      sphere_ep_1.color.b = line_colors[i][2] / 255.;
      sphere_ep_1.color.a = 1.0;
      sphere_ep_1.lifetime = ros::Duration(2, 0);
      m.markers.push_back(sphere_ep_1);

      visualization_msgs::Marker sphere_ep_2;
      sphere_ep_2.header.frame_id = finput_->header.frame_id;
      sphere_ep_2.header.stamp = ros::Time::now();
      sphere_ep_2.ns = "laser_lines";
      sphere_ep_2.id = idnum++;
      sphere_ep_2.type = visualization_msgs::Marker::SPHERE;
      sphere_ep_2.action = visualization_msgs::Marker::ADD;
      sphere_ep_2.pose.position.x = info.end_point_2[0];
      sphere_ep_2.pose.position.y = info.end_point_2[1];
      sphere_ep_2.pose.position.z = info.end_point_2[2];
      sphere_ep_2.pose.orientation.w = 1.;
      sphere_ep_2.scale.x = 0.05;
      sphere_ep_2.scale.y = 0.05;
      sphere_ep_2.scale.z = 0.05;
      sphere_ep_2.color.r = line_colors[i][0] / 255.;
      sphere_ep_2.color.g = line_colors[i][1] / 255.;
      sphere_ep_2.color.b = line_colors[i][2] / 255.;
      sphere_ep_2.color.a = 1.0;
      sphere_ep_2.lifetime = ros::Duration(2, 0);
      m.markers.push_back(sphere_ep_2);

      visualization_msgs::Marker lineseg;
      lineseg.header.frame_id = finput_->header.frame_id;
      lineseg.header.stamp = ros::Time::now();
      lineseg.ns = "laser_lines";
      lineseg.id = idnum++;
      lineseg.type = visualization_msgs::Marker::LINE_LIST;
      lineseg.action = visualization_msgs::Marker::ADD;
      lineseg.points.resize(2);
      lineseg.points[0].x = info.end_point_1[0];
      lineseg.points[0].y = info.end_point_1[1];
      lineseg.points[0].z = info.end_point_1[2];
      lineseg.points[1].x = info.end_point_2[0];
      lineseg.points[1].y = info.end_point_2[1];
      lineseg.points[1].z = info.end_point_2[2];
      lineseg.scale.x = 0.02;
      lineseg.scale.y = 0.04;
      lineseg.color.r = line_colors[i][0] / 255.;
      lineseg.color.g = line_colors[i][1] / 255.;
      lineseg.color.b = line_colors[i][2] / 255.;
      lineseg.color.a = 1.0;
      lineseg.lifetime = ros::Duration(2, 0);
      m.markers.push_back(lineseg);
    }
  }

  for (size_t i = idnum; i < last_id_num_; ++i) {
    visualization_msgs::Marker delop;
    delop.header.frame_id = finput_->header.frame_id;
    delop.header.stamp = ros::Time::now();
    delop.ns = "laser_lines";
    delop.id = i;
    delop.action = visualization_msgs::Marker::DELETE;
    m.markers.push_back(delop);
  }
  last_id_num_ = idnum;

  vispub_->publish(m);
}
#endif

/***************************************************************************
 *  map_lasergen_thread.cpp - Thread to generate laser data from map
 *
 *  Created: Thu Aug 23 18:43:39 2012
 *  Copyright  2012-2014  Tim Niemueller [www.niemueller.de]
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

#include "map_lasergen_thread.h"
#include "amcl_utils.h"

#include <utils/math/angle.h>
#include <baseapp/run.h>

using namespace fawkes;

/** @class MapLaserGenThread "map_lasergen_thread.h"
 * Generate laser data from map and position.
 * @author Tim Niemueller
 */

/** Constructor. */
MapLaserGenThread::MapLaserGenThread()
  : Thread("MapLaserGenThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE),
    TransformAspect(TransformAspect::BOTH_DEFER_PUBLISHER, "Map Laser Odometry")
{
  map_ = NULL;
}

/** Destructor. */
MapLaserGenThread::~MapLaserGenThread()
{
}

void MapLaserGenThread::init()
{
  laser_pose_set_ = false;

  bool have_custom_map = false;
  try {
      config->get_string(AMCL_CFG_PREFIX"map-lasergen/map_file");
      have_custom_map = true;
  } catch (Exception &e) {} // ignored, use AMCL map

  logger->log_info(name(), "Using %s map configuration",
		   have_custom_map ? "custom" : "AMCL");
  fawkes::amcl::read_map_config(config, cfg_map_file_, cfg_resolution_, cfg_origin_x_,
				cfg_origin_y_, cfg_origin_theta_, cfg_occupied_thresh_,
				cfg_free_thresh_,
				have_custom_map ? AMCL_CFG_PREFIX"map-lasergen/" : AMCL_CFG_PREFIX);

  cfg_use_current_pose_ = false;
  try {
    cfg_use_current_pose_ = config->get_bool(AMCL_CFG_PREFIX"map-lasergen/use_current_pos");
  } catch (Exception &e) {} // ignored, use default

  cfg_laser_ifname_ = config->get_string(AMCL_CFG_PREFIX"map-lasergen/laser_interface_id");
  odom_frame_id_  = config->get_string("/frames/odom");
  base_frame_id_  = config->get_string("/frames/base");
  laser_frame_id_ = config->get_string(AMCL_CFG_PREFIX"map-lasergen/laser_frame_id");

  if (cfg_use_current_pose_) {
    cfg_pose_ifname_ = config->get_string(AMCL_CFG_PREFIX"map-lasergen/pose_interface_id");
    cur_pose_if_ = blackboard->open_for_reading<Position3DInterface>(cfg_pose_ifname_.c_str());
  } else {
    pos_x_     = config->get_float(AMCL_CFG_PREFIX"map-lasergen/pos_x");
    pos_y_     = config->get_float(AMCL_CFG_PREFIX"map-lasergen/pos_y");
    pos_theta_ = config->get_float(AMCL_CFG_PREFIX"map-lasergen/pos_theta");
  }

  std::vector<std::pair<int, int> > free_space_indices;
  map_ = fawkes::amcl::read_map(cfg_map_file_.c_str(),
				cfg_origin_x_, cfg_origin_y_, cfg_resolution_,
				cfg_occupied_thresh_, cfg_free_thresh_, free_space_indices);

  map_width_  = map_->size_x;
  map_height_ = map_->size_y;

  logger->log_info(name(), "Size: %ux%u (%zu of %u cells free, this are %.1f%%)",
		   map_width_, map_height_, free_space_indices.size(),
                   map_width_ * map_height_,
		   (float)free_space_indices.size() / (float)(map_width_ * map_height_) * 100.);

  laser_if_ = blackboard->open_for_writing<Laser360Interface>(cfg_laser_ifname_.c_str());
  gt_pose_if_ = blackboard->open_for_writing<Position3DInterface>("Map LaserGen Groundtruth");

  cfg_add_noise_ = false;
  try {
    cfg_add_noise_ = config->get_bool(AMCL_CFG_PREFIX"map-lasergen/add_gaussian_noise");
  } catch (Exception &e) {}; // ignored
  if (cfg_add_noise_) {
#ifndef HAVE_RANDOM
    throw Exception("Noise has been enabled but C++11 <random> no available at compile time");
#else
    cfg_noise_sigma_ = config->get_float(AMCL_CFG_PREFIX"map-lasergen/noise_sigma");
    std::random_device rd;
    noise_rg_ = std::mt19937(rd());
    noise_nd_ = std::normal_distribution<float>(0.0, cfg_noise_sigma_);
#endif
  }

  cfg_send_zero_odom_ = false;
  try {
    cfg_send_zero_odom_ = config->get_bool(AMCL_CFG_PREFIX"map-lasergen/send_zero_odom");
  } catch (Exception &e) {}; // ignored
  if (cfg_send_zero_odom_) {
    logger->log_info(name(), "Enabling zero odometry publishing");
    tf_enable_publisher();
  }

  laser_if_->set_frame(laser_frame_id_.c_str());
}


void
MapLaserGenThread::loop()
{

  if (!laser_pose_set_) {
    if (set_laser_pose()) {
      laser_pose_set_ = true;

      if (! cfg_use_current_pose_) {
	// only need to set it once, let's do it here
	laser_pos_x_ = pos_x_ + laser_pose_.getOrigin().x();
	laser_pos_y_ = pos_y_ + laser_pose_.getOrigin().y();
	laser_pos_theta_ = pos_theta_ + tf::get_yaw(laser_pose_.getRotation());
      }
    } else {
      if (fawkes::runtime::uptime() >= tf_listener->get_cache_time()) {
	logger->log_warn(name(), "Could not determine laser pose, skipping loop");
      }
      return;
    }
  }

  if (cfg_use_current_pose_) {
    cur_pose_if_->read();
    pos_x_     = cur_pose_if_->translation(0);
    pos_y_     = cur_pose_if_->translation(1);
    pos_theta_ = tf::get_yaw(tf::Quaternion(cur_pose_if_->rotation(0),
					    cur_pose_if_->rotation(1),
					    cur_pose_if_->rotation(2),
					    cur_pose_if_->rotation(3)));

    //logger->log_info(name(), "Pos: (%f,%f,%f)", pos_x_, pos_y_, pos_theta_);

    const float lx = laser_pose_.getOrigin().x();
    const float ly = laser_pose_.getOrigin().y();
    laser_pos_theta_ = pos_theta_ + tf::get_yaw(laser_pose_.getRotation());
    const float sin_theta = sinf(laser_pos_theta_);
    const float cos_theta = cosf(laser_pos_theta_);
    laser_pos_x_ = pos_x_ + (lx * cos_theta - ly * sin_theta);
    laser_pos_y_ = pos_y_ + (lx * sin_theta + ly * cos_theta);
  }

  tf::Quaternion q(tf::create_quaternion_from_yaw(pos_theta_));
  gt_pose_if_->set_translation(0, pos_x_);
  gt_pose_if_->set_translation(1, pos_y_);
  gt_pose_if_->set_rotation(0, q.x());
  gt_pose_if_->set_rotation(1, q.y());
  gt_pose_if_->set_rotation(2, q.z());
  gt_pose_if_->set_rotation(3, q.w());
  gt_pose_if_->write();

  float dists[360];
  for (unsigned int i = 0; i < 360; ++i) {
    dists[i] = map_calc_range(map_, laser_pos_x_, laser_pos_y_,
			      normalize_rad(deg2rad(i) + laser_pos_theta_), 100.);
  }
#ifdef HAVE_RANDOM
  if (cfg_add_noise_) {
    for (unsigned int i = 0; i < 360; ++i) {
      dists[i] += noise_nd_(noise_rg_);
    }
  }
#endif
  laser_if_->set_distances(dists);
  laser_if_->write();


  if (cfg_send_zero_odom_) {
    tf::Transform
      tmp_tf(tf::create_quaternion_from_yaw(0), tf::Vector3(0,0,0));
    Time transform_expiration =
      (Time(clock) + 1.0);

    tf::StampedTransform tmp_tf_stamped(tmp_tf,
					transform_expiration,
					odom_frame_id_, base_frame_id_);
    tf_publisher->send_transform(tmp_tf_stamped);
  }
}

void MapLaserGenThread::finalize()
{
  if (map_) {
    map_free(map_);
    map_ = NULL;
  }

  blackboard->close(laser_if_);
  blackboard->close(gt_pose_if_);
  blackboard->close(cur_pose_if_);
}


bool
MapLaserGenThread::set_laser_pose()
{
  fawkes::Time now(clock);
  tf::Stamped<tf::Pose>
    ident(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0)),
          &now, laser_frame_id_);
  try {
    tf_listener->transform_pose(base_frame_id_, ident, laser_pose_);
  } catch (fawkes::Exception& e) {
    return false;
  }

  logger->log_debug(name(), "LaserTF: (%f,%f,%f)",
		    laser_pose_.getOrigin().x(), laser_pose_.getOrigin().y(),
		    tf::get_yaw(laser_pose_.getRotation()));

  return true;
}
